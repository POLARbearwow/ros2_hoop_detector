### 方式一：launch
```bash
ros2 launch hoop_detector hoop_system.launch.py 
```

### 方法二：手动加载组件
**终端 1: 启动组件容器**
首先，启动一个空的组件容器，它将作为后续加载组件的“家”。
```bash
ros2 run rclcpp_components component_container
```
> **注意**: 这个容器的默认节点名是 `/ComponentManager`。
> 
> 
**终端 2: 加载相机节点**
最后，将海康相机 (`hik_camera`) 组件加载到容器中。
```bash
ros2 component load /ComponentManager hik_camera hik_camera::HikCameraNode
```
**终端 3: 加载篮筐检测节点**
接下来，将篮筐检测 (`hoop_detector`) 组件加载到容器中。
```bash
ros2 component load /ComponentManager hoop_detector hoop_detector::HoopDetectorNode
```

