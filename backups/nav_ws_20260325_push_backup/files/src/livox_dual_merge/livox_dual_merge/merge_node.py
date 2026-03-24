import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class LivoxDualMergeNode(Node):
    def __init__(self):
        super().__init__('livox_dual_merge_node')
        self.declare_parameter('frame_id', 'base_link') # ????? base_link
        self.frame_id = self.get_parameter('frame_id').value

        self.last_lidar_1 = None
        self.last_lidar_2 = None
        self.last_imu = None
        self.last_stamp_ns = 0  # 保证时间戳单调递增，避免 FAST-LIO lidar loop back

        self.sub_lidar_1 = self.create_subscription(
            CustomMsg, '/livox/lidar_1', self.lidar_1_cb, 10)
        self.sub_lidar_2 = self.create_subscription(
            CustomMsg, '/livox/lidar_2', self.lidar_2_cb, 10)
        
        self.sub_imu_1 = self.create_subscription(
            Imu, '/livox/imu_1', self.imu_cb, 10)
        self.sub_imu_2 = self.create_subscription(
            Imu, '/livox/imu_2', self.imu_cb, 10)

        self.pub_lidar = self.create_publisher(PointCloud2, '/livox/lidar', 10)
        self.pub_imu = self.create_publisher(Imu, '/livox/imu', 10)
        
        self.get_logger().info("Livox Dual Merge Node Started (Output: PointCloud2)")

    def lidar_1_cb(self, msg):
        self.last_lidar_1 = msg
        self._try_publish_merged()

    def lidar_2_cb(self, msg):
        self.last_lidar_2 = msg
        self._try_publish_merged()

    def imu_cb(self, msg):
        msg.header.frame_id = self.frame_id
        self.pub_imu.publish(msg)

    def _try_publish_merged(self):
        if self.last_lidar_1 is None or self.last_lidar_2 is None:
            return
        # FAST-LIO mid360_handler 需要 LivoxPointXyzitl: x,y,z,intensity,tag,line
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
            PointField(name='line', offset=17, datatype=PointField.UINT8, count=1),
        ]
        
        points_list = []
        
        t1 = self.last_lidar_1.timebase
        t2 = self.last_lidar_2.timebase
        base_timebase = max(self.last_stamp_ns, min(t1, t2))  # 单调递增，避免 lidar loop back
        self.last_stamp_ns = base_timebase
        # Livox timebase 为纳秒 (见 livox_ros_driver2 lddc.cpp)
        stamp_sec = int(base_timebase // 1_000_000_000)
        stamp_nanosec = int(base_timebase % 1_000_000_000)

        header = Header()
        header.stamp.sec = stamp_sec
        header.stamp.nanosec = stamp_nanosec
        header.frame_id = self.frame_id

        for p in self.last_lidar_1.points:
            # LivoxPointXyzitl: x,y,z,intensity,tag,line (tag 来自 CustomPoint.tag)
            points_list.append([
                p.x, p.y, p.z,
                float(p.reflectivity),
                p.tag,
                p.line
            ])

        for p in self.last_lidar_2.points:
            points_list.append([
                p.x, p.y, p.z,
                float(p.reflectivity),
                p.tag,
                p.line
            ])

        cloud_msg = point_cloud2.create_cloud(header, fields, points_list)
        self.pub_lidar.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LivoxDualMergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
