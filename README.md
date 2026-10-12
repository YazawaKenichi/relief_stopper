# chain_stopper

通常は cmd_vel をそのまま通す。
特定のトピックに 1 が入ったら cmd_vel = 0 にする。

（ relief_stopper を改造したもので、残骸がかなり残ってる ）

## Overview

## Usage
## Prerequisites
- ROS2 Humble
- C++ 17

### Download and Install
1. ワークスペースにこのパッケージをクローン
    ``` cmd
    cd ~/ros2_ws/src/
    git clone https://github.com/yazawakenichi/chain_stopper
    ```
2. パッケージのビルド
    ``` cmd
    cd ~/ros2_ws/
    colcon build --packages-select chain_stopper
    ```
3. 実行ファイルのオーバーレイ
    ``` cmd
    source install/local_setup.bash
    ```

### Launching Node
`ros2 launch` を用いることでノードを起動することができます。

新しくターミナルを開いた場合、ノードを起動する前に `source install/local_setup.bash` することを忘れないでください。

``` cmd
ros2 launch chain_stopper chain_stopper_launch.py
```

## Package Description

#### Topic
|Name|Type|Description
|---|---|---
|`/`|``|

### Parameters
|Name|Type|Default|Description
|---|---|---|---
|`/sub_topic_name`|`std::string`|``|サブスクライブするトピック名前
|`/stop_lifecycle_name`|`std::string`|``|一時停止するライフサイクル名
|`/chain_threshold`|`double`|`1.0`|段差とする基準値

## License
このソフトウェアは、MIT License の下、再頒布および使用が許可されます。

(C) 2023 YAZAWA Kenichi

