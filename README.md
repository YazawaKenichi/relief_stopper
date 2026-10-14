# relief_stopper

<!--正規化された棒の角度の値を取得して、基準値と比較して特定のライフサイクルを一時停止-->
棒の角度の値を取得し、基準値と比較して、段差と判定したら /cmd_vel をゼロにして次段に渡す

## Overview

## Usage
## Prerequisites
- ROS2 Humble
- C++ 17

### Download and Install
1. ワークスペースにこのパッケージをクローン
    ``` cmd
    cd ~/ros2_ws/src/
    git clone https://github.com/yazawakenichi/relief_stopper
    ```
2. パッケージのビルド
    ``` cmd
    cd ~/ros2_ws/
    colcon build --packages-select relief_stopper
    ```
3. 実行ファイルのオーバーレイ
    ``` cmd
    source install/local_setup.bash
    ```

### Launching Node
`ros2 launch` を用いることでノードを起動することができます。

新しくターミナルを開いた場合、ノードを起動する前に `source install/local_setup.bash` することを忘れないでください。

``` cmd
ros2 launch relief_stopper relief_stopper_launch.py
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
|`/relief_threshold`|`double`|`1.0`|段差とする基準値

## License
このソフトウェアは、MIT License の下、再頒布および使用が許可されます。

(C) 2023 YAZAWA Kenichi

