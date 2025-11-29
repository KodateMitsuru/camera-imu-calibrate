# camera-imu-calibrate

## build && run

### 编译
```bash
cd build && cmake .. && make -j$(nproc)
```
### 运行

#### 多相机标定（Multi-camera calibration）

1. 指定模型和图像文件夹：
```bash
./calibrate camera \
  --dataset-name mydata \
  --models pinhole-radtan pinhole-equi \
  --image-folders /path/to/cam0/images /path/to/cam1/images \
  --target aprilgrid.yaml
```

2. 使用已有相机 YAML 配置文件：
```bash
./calibrate camera \
  --dataset-name mydata \
  --cams camchain.yaml \
  --target aprilgrid.yaml
```

常用额外参数示例：
```bash
# 导出相机姿态轨迹到 CSV
./calibrate camera --dataset-name mydata --models pinhole-radtan --image-folders /path/to/cam0 --target aprilgrid.yaml --export-poses
```

#### IMU-相机联合标定（IMU-camera calibration）

用于同时标定相机与 IMU 的空间外参与时间偏移：
```bash
./calibrate imu_camera \
  --dataset-name mydata \
  --cams camchain.yaml \
  --imu imu.yaml \
  --target aprilgrid.yaml
```

说明：IMU-相机 标定通常需要一个包含相机图像路径与/或内参的 `camchain.yaml`，以及至少一个 IMU 配置 `imu.yaml`（包含 CSV 路径）。

## usage

### Multi-camera

常用于仅标定相机内参与相机间外参（不含 IMU）。可以使用 `--models` + `--image-folders` 直接从图像文件夹构建相机，或使用已有的 `--cams` YAML 文件进行重标定。

| 参数 | 说明 | 示例 |
|------|------|------|
| `--dataset-name` | 数据集名称（输出文件前缀） | `--dataset-name mydata` |
| `--cams` | 相机配置文件（YAML，含图像路径），用于已知内参的重标定 | `--cams camchain.yaml` |
| `--models` | 按顺序指定每个相机的模型字符串（替代 `--cams` 的内参部分） | `--models pinhole-radtan pinhole-equi` |
| `--image-folders` | 与 `--models` 配合使用，为每个相机提供图像文件夹路径 | `--image-folders /path/to/cam0 /path/to/cam1` |
| `--target` | 标定板配置 | `--target aprilgrid.yaml` |
| `--time-range` | 时间范围[开始, 结束]秒 | `--time-range 10.0 60.0` |
| `--image-freq` | 图像提取频率Hz | `--image-freq 4.0` |
| `--max-iter` | 最大迭代次数 | `--max-iter 50` |
| `--verbose` | 详细输出 | `--verbose` |
| `--export-poses` | 导出相机姿态为 CSV（每个相机会生成 `<name>-poses-cam<N>.csv` 或当前实现为 cam0） | `--export-poses` |
| `--help` | 查看帮助 | `--help` |

示例：使用模型字符串并导出姿态
```bash
./calibrate camera \
  --dataset-name mydata \
  --models pinhole-radtan pinhole-equi \
  --image-folders /path/to/cam0/images /path/to/cam1/images \
  --target aprilgrid.yaml \
  --export-poses
```

### IMU-camera

用于同时标定相机与 IMU 的空间外参与时间偏移。通常需要 `--cams`（包含图像路径或内参）以及至少一个 `--imu` 配置。

| 参数 | 说明 | 示例 |
|------|------|------|
| `--dataset-name` | 数据集名称（输出文件前缀） | `--dataset-name mydata` |
| `--cams` | 相机配置文件（YAML，含图像路径/内参） | `--cams camchain.yaml` |
| `--imu` | IMU配置（含CSV路径，可多个） | `--imu imu0.yaml imu1.yaml` |
| `--target` | 标定板配置 | `--target aprilgrid.yaml` |
| `--imu-models` | IMU误差模型 | `--imu-models calibrated` |
| `--time-range` | 时间范围[开始, 结束]秒 | `--time-range 10.0 60.0` |
| `--no-time-calibration` | 禁用时间校准 | `--no-time-calibration` |
| `--recover-covariance` | 计算协方差 | `--recover-covariance` |
| `--max-iter` | 最大迭代次数 | `--max-iter 50` |
| `--verbose` | 详细输出 | `--verbose` |
| `--help` | 查看帮助 | `--help` |

示例：IMU-相机联合标定
```bash
./calibrate imu_camera \
  --dataset-name mydata \
  --cams camchain.yaml \
  --imu imu.yaml \
  --target aprilgrid.yaml
```

## config template

### camchain.yaml

```yaml
cam0:
  camera_model: pinhole
  intrinsics: [458.654, 457.296, 367.215, 248.375]
  distortion_model: radtan
  distortion_coeffs: [-0.283, 0.074, 0.0002, 1.76e-05]
  resolution: [752, 480]
  image_folder: /path/to/cam0/images  # 图像文件夹路径
  T_cam_imu:  # 初始相机到IMU变换（可选）
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
```

### imu.yaml

```yaml
accelerometer_noise_density: 0.006    # [m/s^2/sqrt(Hz)]
accelerometer_random_walk: 0.0002     # [m/s^3/sqrt(Hz)]
gyroscope_noise_density: 0.0004       # [rad/s/sqrt(Hz)]
gyroscope_random_walk: 4.0e-06        # [rad/s^2/sqrt(Hz)]
update_rate: 200.0                    # [Hz]
csv_file: /path/to/imu_data.csv       # IMU数据CSV文件
```

### IMU CSV

```csv
timestamp_ns,wx,wy,wz,ax,ay,az
1234567890000000,-0.001,0.002,-0.003,9.81,0.01,-0.02
```

### aprilgrid.yaml

```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088      # [m]
tagSpacing: 0.3     # 相对于tagSize的比例
```

## output

以下是常见输出文件（基于 `--dataset-name <name>`）：

| 文件 | 内容 |
|------|------|
| `<name>-camchain.yaml` | 相机链内参/外参（若使用 `--models` 会写入估计后的相机链） |
| `<name>-results-cam.txt` | 相机标定详细结果 |
| `<name>-report-cam.pdf` | 相机标定可视化报告 |
| `<name>-poses-cam0.csv` | 导出的相机姿态轨迹（若使用 `--export-poses`） |
| `<name>-camchain-imucam.yaml` | 相机-IMU 外参（在 IMU-相机 标定子命令中生成） |
| `<name>-imu.yaml` | 优化后的 IMU 参数（在 IMU-相机 标定子命令中生成） |
| `<name>-results-imucam.txt` | IMU-相机 详细结果 |
| `<name>-report-imucam.pdf` | IMU-相机 可视化报告 |

说明：可以使用 `--models` + `--image-folders` 直接指定相机模型与图像数据（更方便用于初次标定），或者使用 `--cams` 提供完整的 `camchain.yaml` 文件用于重标定。

---------

## References

kalibr: <https://github.com/ethz-asl/kalibr>

matplotplusplus: <https://github.com/alandefreitas/matplotplusplus>

rapidcsv: <https://github.com/d99kris/rapidcsv>

argparse: <https://github.com/p-ranav/argparse>

