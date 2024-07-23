# myagv の立ち上げ手順

## 概略
このAGVはSLAMしてから、Navigation stuck for ROS2(以下：Navigation)を用いて走行するものです。
以下にSLAMとNavigationの手順を示します。


## 実行手順
まずどちらも必要なlaunchファイルで立ち上げをします。
core_launch.pyの立ち上げです。これは、/cmd_velからモーターの制御量に変換し、arduinoに伝えるプログラムを起動します。

ディレクトリの移動
```
cd myagv_ws
```

launchファイルの起動
```
ros2 launch myagv_bringup core_launch.py
```

SLAMの起動手順
slam-toolboxの起動
```
ros2 launch slam_toolbox online_async_launch.py
```
rviz2
```
rviz2
```

rviz2を開いたあとに左下のAddを押して、By topicからMapを追加することで、SLAMしてできたMapを確認できます。

Navigationの起動手順

```
ros2 launch myagv_navigation myagv_navigation.launch.py
```

rviz2の起動
```
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```
