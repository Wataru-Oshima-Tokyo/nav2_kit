# nav2_kit

### リポジトリ概要
このリポジトリはROS2を利用して、ナビゲーションをシミュレーションしたり、ナビゲーションの基盤として利用できるものです。

#### 主な機能:
- Gazeboのワールドを2次元マップから作成 (`map_builder`)
- Gazebo上でのワールド表示 (`sim_worlds2`, `articubot`, `champ`)
- 二輪、四輪（調整中）、四足歩行ロボット（調整中）のシミュレーション (`sim_worlds2`)
- 2D、3D、RGBカメラ、深度カメラをロボットへの取り付け (`sim_worlds2`)
- 3Dマップ生成とナビゲーション (`LIO_SAM`を利用, `robot_navigation`)
- 安全な衝突検知機能 (`robot_navigation`)

### インストール方法

#### 要件:
- OS: Ubuntu 22.04 
  - Docker使用時は[こちら](https://github.com/TechShare-inc/ros_here/tree/main/humble)
- RAM: 8GB以上
- CPU: 8コア
- GPU: なくても可 (ただし、シミュレーション使用時は推奨)

#### インストール手順:
```
mkdir -p <workspace name>/src
cd <workspace name>/src
git clone --recursive https://github.com/Wataru-Oshima-Tokyo/nav2_kit.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build 
```

### 実行例
このリポジトリでは`catmux`を用いることで、必要なノードを一度に起動できます。

#### `catmux` + `tmux`のインストール:
```
sudo apt update
sudo apt install -y tmux
sudo apt install python3-pip
python3 -m pip install catmux
```

#### 実行手順:
例: 二輪ロボットに3Dライダー、RGBカメラ、IMUを装備し、大成建設の赤坂ビルワールドに配置し、3Dマッピングとナビゲーションを開始する場合。
```
cd <workspace name>/src/nav2_kit/catmux
catmux_create_session slam_3d_diffbot_in_aksk.yaml 
```
※デフォルトではGazeboのワールド可視化はOFF。`slam_3d_diffbot_in_aksk.yaml`の`headless`を`True`に設定するとワールドが表示されます。

