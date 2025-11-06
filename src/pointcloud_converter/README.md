# PointCloud Converter

将不带ring和time字段的PointCloud2消息转换为带有ring和time字段的PointCloud2消息（Velodyne格式）。

## 功能

- 订阅原始PointCloud2消息（包含x, y, z, intensity字段）
- 添加ring字段（激光环编号）
- 添加time字段（相对扫描起始时间）
- 发布完整的PointCloud2消息

## 编译

```bash
colcon build --packages-select pointcloud_converter
source install/setup.bash
```

## 使用方法

### 方法1：使用launch文件

```bash
ros2 launch pointcloud_converter pointcloud_converter.launch.py
```

### 方法2：使用自定义参数

```bash
ros2 launch pointcloud_converter pointcloud_converter.launch.py input_topic:=/your_input_topic output_topic:=/your_output_topic num_rings:=32
```

### 方法3：直接运行节点

```bash
ros2 run pointcloud_converter pointcloud_converter_node --ros-args \
  -p input_topic:=/points_raw \
  -p output_topic:=/velodyne_points \
  -p num_rings:=32
```

## 参数说明

- `input_topic` (默认: `/points_raw`): 输入的PointCloud2话题
- `output_topic` (默认: `/velodyne_points`): 输出的PointCloud2话题
- `num_rings` (默认: `32`): 激光雷达线数

## 消息格式

### 输入消息
- x, y, z: FLOAT32
- intensity: FLOAT32

### 输出消息
- x, y, z: FLOAT32
- intensity: FLOAT32
- ring: UINT16 (激光环编号，0到num_rings-1)
- time: FLOAT32 (相对时间，秒)

## 注意事项

1. ring字段计算基于点云数据的组织方式（height和width维度）
2. time字段假设一个完整扫描周期为0.1秒（10Hz）
3. 根据你的数据格式（height=1800, width=32），假设width对应线数
