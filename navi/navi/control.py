import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined  # type: ignore
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PX4IMUReader(Node):
    def __init__(self):
        super().__init__('px4_imu_reader')
        
        # Configure QoS profile for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.imu_subscriber = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.imu_callback,
            qos_profile
        )
        

        
        self.get_logger().info('PX4 IMU Data Reader node has been initialized')
        
    def imu_callback(self, msg):
        try:
            self.get_logger().info(
                f'\nRaw IMU Data:'
                f'\n  Accelerometer (m/s²):'
                f'\n    x: {msg.accelerometer_m_s2[0]:.4f}'
                f'\n    y: {msg.accelerometer_m_s2[1]:.4f}'
                f'\n    z: {msg.accelerometer_m_s2[2]:.4f}'
                f'\n  Gyroscope (rad/s):'
                f'\n    x: {msg.gyro_rad[0]:.4f}'
                f'\n    y: {msg.gyro_rad[1]:.4f}'
                f'\n    z: {msg.gyro_rad[2]:.4f}'
                f'\n  Timestamp: {msg.timestamp}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PX4IMUReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    except Exception as e:
        print(f'Caught exception: {str(e)}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()