from pathlib import Path
import cv2
import csv
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

def extract_images_from_rosbag(bag_path, image_topic, out_folder):
    bag_path = Path(bag_path)
    out_folder = Path(out_folder)
    out_folder.mkdir(parents=True, exist_ok=True)

    typestore = get_typestore(Stores.ROS1_NOETIC)
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == image_topic]
        for conn in connections:
            for connection, timestamp, rawdata in reader.messages([conn]):
                msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
                # sensor_msgs/msg/Image
                if conn.msgtype == 'sensor_msgs/msg/Image':
                    # 解码为 numpy 数组
                    img = np.ndarray(
                        shape=(msg.height, msg.width, 3 if msg.encoding in ['rgb8', 'bgr8'] else 1),
                        dtype=np.uint8,
                        buffer=msg.data
                    )
                    # 灰度或彩色自动保存
                    if img.ndim == 3 and img.shape[2] == 3:
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    fname = out_folder / f'{timestamp}.png'
                    cv2.imwrite(str(fname), img)
                # sensor_msgs/msg/CompressedImage
                elif conn.msgtype == 'sensor_msgs/msg/CompressedImage':
                    img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)
                    if img.ndim == 3 and img.shape[2] == 3:
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    fname = out_folder / f'{timestamp}.png'
                    cv2.imwrite(str(fname), img)
                print(f'Saved: {fname}')
                
def extract_imu_to_csv(bag_path, imu_topic, out_csv):
    bag_path = Path(bag_path)
    out_csv = Path(out_csv)
    typestore = get_typestore(Stores.ROS1_NOETIC)
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == imu_topic]
        with open(out_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            # 写表头
            writer.writerow([
                'timestamp',
                'omega_x', 'omega_y', 'omega_z',
                'alpha_x', 'alpha_y', 'alpha_z'
            ])
            for conn in connections:
                for connection, timestamp, rawdata in reader.messages([conn]):
                    msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
                    # sensor_msgs/msg/Imu
                    if conn.msgtype == 'sensor_msgs/msg/Imu':
                        # 时间戳（秒）
                        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                        omega = [
                            msg.angular_velocity.x,
                            msg.angular_velocity.y,
                            msg.angular_velocity.z
                        ]
                        alpha = [
                            msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z
                        ]
                        writer.writerow([t] + omega + alpha)
                        # 可选：打印进度
                        print(f"{t:.6f}, {omega}, {alpha}")
# 用法示例
if __name__ == '__main__':
    bagfile = 'imu_april.bag'
    topic = '/cam0/image_raw'
    outdir = 'output_images'
    extract_images_from_rosbag(bagfile, topic, outdir)
    imu_topic = '/imu0'
    out_csv = 'output_imu.csv'
    extract_imu_to_csv(bagfile, imu_topic, out_csv)
