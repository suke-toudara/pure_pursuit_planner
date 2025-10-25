# Pure Pursuit Planner for ROS2

ROS2用のPure Pursuitアルゴリズムを用いたパス追従プランナーです。状態管理機能を持ち、ロボットの動作を段階的に制御します。

## 機能

- **状態管理システム**: ロボットの動作を5つの状態で管理
- **初期回転制御**: パス追従開始前に進行方向に向きを合わせる
- **Pure Pursuitアルゴリズム**: 経路追従のための古典的な制御アルゴリズム
- **ゴール回転制御**: ゴール到達後に目標姿勢に向きを合わせる
- **状態のPublish**: 現在のロボット状態をリアルタイムで配信

## ロボットの状態

```cpp
enum class RobotState {
    STOP,           // 停止状態
    START_ROTATE,   // 開始回転状態（パスの最初の方向に向きを合わせる）
    FOLLOW,         // 通常のパス追従状態
    GOAL_ROTATE,    // ゴールに向かって回転する状態
    GOAL            // ゴール到達状態
};
```

### 状態遷移

1. **STOP** → **START_ROTATE**: 新しいパスを受信
2. **START_ROTATE** → **FOLLOW**: 初期方向への回転が完了
3. **FOLLOW** → **GOAL_ROTATE**: ゴール位置に到達
4. **GOAL_ROTATE** → **GOAL**: ゴール姿勢への回転が完了

## トピック

### Subscribed Topics

- `path` (nav_msgs/Path): 追従するパス
- `odom` (nav_msgs/Odometry): ロボットの現在位置と姿勢

### Published Topics

- `cmd_vel` (geometry_msgs/Twist): ロボットへの速度コマンド
- `robot_state` (std_msgs/String): 現在のロボット状態（文字列形式）

## パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|------------|------|----------|------|
| `lookahead_distance` | double | 1.0 | Lookahead距離 [m] |
| `max_linear_velocity` | double | 0.5 | 最大並進速度 [m/s] |
| `max_angular_velocity` | double | 1.0 | 最大回転速度 [rad/s] |
| `goal_tolerance` | double | 0.1 | ゴール到達判定の距離閾値 [m] |
| `angle_tolerance` | double | 0.1 | 角度一致判定の閾値 [rad] |
| `rotate_angular_velocity` | double | 0.5 | 回転時の角速度 [rad/s] |
| `control_frequency` | double | 10.0 | 制御ループの周波数 [Hz] |

## ビルド方法

```bash
# ワークスペースのsrcディレクトリに配置
cd ~/ros2_ws/src
git clone <repository_url> pure_pursuit_planner

# ワークスペースのルートでビルド
cd ~/ros2_ws
colcon build --packages-select pure_pursuit_planner

# 環境設定を読み込み
source install/setup.bash
```

## 使用方法

### 基本的な起動

```bash
ros2 launch pure_pursuit_planner pure_pursuit.launch.py
```

### パラメータを指定して起動

```bash
ros2 launch pure_pursuit_planner pure_pursuit.launch.py \
    lookahead_distance:=1.5 \
    max_linear_velocity:=0.8 \
    max_angular_velocity:=1.5
```

### ノードを直接実行

```bash
ros2 run pure_pursuit_planner pure_pursuit_planner_node \
    --ros-args \
    -p lookahead_distance:=1.5 \
    -p max_linear_velocity:=0.8
```

## 動作の流れ

1. **パス受信**: `path`トピックから目標経路を受信
2. **初期回転**: 最初の点と次の点を結ぶベクトルの方向に回転
3. **パス追従**: Pure Pursuitアルゴリズムでパスを追従
4. **ゴール回転**: ゴール位置に到達後、ゴールの姿勢に回転
5. **完了**: ゴール状態で停止

## 例: 状態のモニタリング

```bash
# ロボットの現在状態を確認
ros2 topic echo /robot_state

# 速度コマンドを確認
ros2 topic echo /cmd_vel
```

## アルゴリズムの詳細

### Pure Pursuit

Pure Pursuitアルゴリズムは、ロボットの前方にあるlookahead point（先読み点）に向かって曲率を計算し、その曲率に基づいて角速度を決定します。

```
曲率 = 2 * sin(角度差) / lookahead距離
角速度 = 曲率 * 並進速度
```

### 初期回転制御

パス追従を開始する前に、ロボットは最初の2点を結ぶベクトルの方向に向きを合わせます。これにより、スムーズにパス追従を開始できます。

```
目標角度 = atan2(点2.y - 点1.y, 点2.x - 点1.x)
```

## ライセンス

MIT License

## 作者

Generated with Claude Code
