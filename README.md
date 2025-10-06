# camera-imu-calibrate

## build && run

```bash
# 编译
cd build && cmake .. && make -j$(nproc)

# 运行
./camera_imu_calibrate \
    --dataset-name mydata \
    --cams camchain.yaml \
    --imu imu.yaml \
    --target aprilgrid.yaml
```

## usage

| 参数 | 说明 | 示例 |
|------|------|------|
| `--dataset-name` | 数据集名称（输出文件前缀） | `--dataset-name mydata` |
| `--cams` | 相机配置（含图像路径） | `--cams camchain.yaml` |
| `--imu` | IMU配置（含CSV路径，可多个） | `--imu imu0.yaml imu1.yaml` |
| `--target` | 标定板配置 | `--target aprilgrid.yaml` |
| `--time-range` | 时间范围[开始, 结束]秒 | `--time-range 10.0 60.0` |
| `--image-freq` | 图像提取频率Hz | `--image-freq 4.0` |
| `--imu-models` | IMU误差模型 | `--imu-models calibrated` |
| `--max-iter` | 最大迭代次数 | `--max-iter 50` |
| `--no-time-calibration` | 禁用时间校准 | `--no-time-calibration` |
| `--recover-covariance` | 计算协方差 | `--recover-covariance` |
| `--verbose` | 详细输出 | `--verbose` |
| `--export-poses` | 导出姿态 | `--export-poses` |
| `--help` | 查看帮助 | `--help` |

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

| 文件 | 内容 |
|------|------|
| `*-camchain-imucam.yaml` | 相机-IMU 外参 |
| `*-imu.yaml` | IMU 参数 |
| `*-results-imucam.txt` | 详细结果 |
| `*-report-imucam.pdf` | 可视化报告 |
| `*-poses-imucam-imu0.csv` | 姿态轨迹 |

---------

## References

kalibr: https://github.com/ethz-asl/kalibr
matplotplusplus: https://github.com/alandefreitas/matplotplusplus
rapidcsv: https://github.com/d99kris/rapidcsv
argparse: https://github.com/p-ranav/argparse