import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'pv',  # Must match Unity publisher topic
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('Image subscriber started, waiting for images...')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image (RGB8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Optionally convert to BGR for OpenCV display
            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            self.get_logger().info(f'timestamp {msg.header.stamp.sec} height: {msg.height} width: {msg.width}')
            # Show image
            cv2.imshow('Unity Image from ROS2', bgr_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
