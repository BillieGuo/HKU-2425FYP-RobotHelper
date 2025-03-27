# 问题分析：出现这个错误通常是因为 tf 树中存在不连通的部分，即 'world' 和 'camera_frame' 不在同一个 tf 树中。
# 虽然存在 world 到 "vx300s/ee_gripper_link" 的 tf 以及 "vx300s/ee_gripper_link" 到 camera_frame 的静态 tf，
# 但可能由于某些原因，这些 tf 没有正确连接成一个完整的树。
# 解决方案：
# 1. 确保静态 tf 发布器已经正确启动，并且 "vx300s/ee_gripper_link" 到 camera_frame 的静态 tf 已经正确发布。
# 2. 检查 tf 发布的频率和延迟，确保所有的 tf 消息都能及时更新。
# 3. 在代码中添加一些延迟，等待 tf 树稳定后再尝试获取变换。
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import time

class TfListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        # 创建一个TF2缓冲区
        self.tf_buffer = tf2_ros.Buffer()
        # 创建一个TF2监听器
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 等待一段时间，让tf树稳定
        time.sleep(2)

        # 创建一个定时器，每0.1秒调用一次print_transform函数
        self.timer = self.create_timer(0.1, self.print_transform)

    def print_transform(self):
        try:
            # 从TF2缓冲区获取world到camera_frame的变换
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'world',
                'camera_frame',
                rclpy.time.Time()
            )
            # 打印变换信息
            self.get_logger().info(
                f"Translation: ({transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z})"
                f"Rotation: ({transform.transform.rotation.x}, {transform.transform.rotation.y}, {transform.transform.rotation.z}, {transform.transform.rotation.w})"
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # 处理异常情况
            self.get_logger().warn(f"Could not transform world to camera_frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    tf_listener = TfListener()
    rclpy.spin(tf_listener)
    tf_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
