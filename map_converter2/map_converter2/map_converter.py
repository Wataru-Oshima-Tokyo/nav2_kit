import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import trimesh
import os

class MapConverter(Node):
    def __init__(self):
        super().__init__('map_converter')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('map_name', 'map')
        self.declare_parameter('mesh_type', 'stl')
        self.declare_parameter('export_dir', '.')
        self.declare_parameter('occupied_thresh', 1)
        self.declare_parameter('box_height', 5.0)
        
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.threshold = self.get_parameter('occupied_thresh').get_parameter_value().integer_value
        self.height = self.get_parameter('box_height').get_parameter_value().double_value
        qos = QoSProfile(depth=10)
        self.test_map_pub = self.create_publisher(OccupancyGrid, 'test_map', qos)
        self.subscription = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, qos)
        
    def map_callback(self, map_msg):
        self.get_logger().info('Received map')
        map_dims = (map_msg.info.height, map_msg.info.width)
        map_array = np.array(map_msg.data).reshape(map_dims)

        # set all -1 (unknown) values to 0 (unoccupied)
        map_array[map_array < 0] = 0
        contours = self.get_occupied_regions(map_array)
        meshes = [self.contour_to_mesh(c, map_msg.info) for c in contours]

        corners = list(np.vstack(contours))
        corners = [c[0] for c in corners]
        # self.publish_test_map(corners, map_msg.info, map_msg.header)
        mesh = trimesh.util.concatenate(meshes)

        # Export as STL or DAE
        mesh_type = self.get_parameter('mesh_type').get_parameter_value().string_value
        export_dir = self.get_parameter('export_dir').get_parameter_value().string_value
        if mesh_type == "stl":
            map_stl = self.map_name + ".stl"
            file_path = os.path.join(export_dir, map_stl)
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            with open(file_path, 'wb') as f:
                mesh.export(f, "stl")
            self.get_logger().info('Exported STL.  You can shut down this node now')
        elif mesh_type == "dae":
            map_dae = self.map_name + ".dae"
            file_path = os.path.join(export_dir, map_dae)
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            with open(file_path, 'wb') as f:
                f.write(trimesh.exchange.dae.export_collada(mesh))
            self.get_logger().info('Exported DAE.  You can shut down this node now')

    def publish_test_map(self, points, metadata, map_header):
        """
        For testing purposes, publishes a map highlighting certain points.
        points is a list of tuples (x, y) in the map's coordinate system.
        """
        test_map = np.zeros((metadata.height, metadata.width), dtype=np.int8)
        for x, y in points:
            test_map[y, x] = 127
            print("points")
        test_map_msg = OccupancyGrid()
        test_map_msg.header = map_header
        test_map_msg.header.stamp = self.get_clock().now().to_msg()
        test_map_msg.info = metadata
        test_map_msg.data = [np.int8(x) for x in np.ravel(test_map)]
        self.test_map_pub.publish(test_map_msg)

    def get_occupied_regions(self, map_array):
        """
        Get occupied regions of map
        """
        map_array = map_array.astype(np.uint8)
        _, thresh_map = cv2.threshold(
                map_array, self.threshold, 100, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(
                thresh_map, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        # Using cv2.RETR_CCOMP classifies external contours at top level of
        # hierarchy and interior contours at second level.  
        # If the whole space is enclosed by walls RETR_EXTERNAL will exclude
        # all interior obstacles e.g. furniture.
        # https://docs.opencv.org/trunk/d9/d8b/tutorial_py_contours_hierarchy.html
        hierarchy = hierarchy[0]
        corner_idxs = [i for i in range(len(contours)) if hierarchy[i][3] == -1]
        return [contours[i] for i in corner_idxs]

    def contour_to_mesh(self, contour, metadata):
        height = np.array([0, 0, self.height])
        s3 = 3**0.5 / 3.
        meshes = []
        for point in contour:
            x, y = point[0]
            vertices = []
            new_vertices = [
                    self.coords_to_loc((x, y), metadata),
                    self.coords_to_loc((x, y+1), metadata),
                    self.coords_to_loc((x+1, y), metadata),
                    self.coords_to_loc((x+1, y+1), metadata)]
            vertices.extend(new_vertices)
            vertices.extend([v + height for v in new_vertices])
            faces = [[0, 2, 4],
                     [4, 2, 6],
                     [1, 2, 0],
                     [3, 2, 1],
                     [5, 0, 4],
                     [1, 0, 5],
                     [3, 7, 2],
                     [7, 6, 2],
                     [7, 4, 6],
                     [5, 4, 7],
                     [1, 5, 3],
                     [7, 3, 5]]
            mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
            if not mesh.is_volume:
                self.get_logger().debug('Fixing mesh normals')
                mesh.fix_normals()
            meshes.append(mesh)
        mesh = trimesh.util.concatenate(meshes)
        mesh.remove_duplicate_faces()
        # mesh will still have internal faces.  Would be better to get
        # all duplicate faces and remove both of them, since duplicate faces
        # are guaranteed to be internal faces
        return mesh

    def coords_to_loc(self, coords, metadata):
        x, y = coords
        loc_x = x * metadata.resolution + metadata.origin.position.x
        loc_y = y * metadata.resolution + metadata.origin.position.y
        # TODO: transform (x*res, y*res, 0.0) by Pose map_metadata.origin
        # instead of assuming origin is at z=0 with no rotation wrt map frame
        return np.array([loc_x, loc_y, 0.0])

def main(args=None):
    rclpy.init(args=args)
    map_converter = MapConverter()
    rclpy.spin(map_converter)
    map_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
