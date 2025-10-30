import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import RegionOfInterest
# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class KinectRGBNode(Node):
    info_msg = None
    bridge = CvBridge()

    def __init__(self):
        super().__init__('kinect_rgb_node')
        
        # Create publishers for kinect data
        self.rgb_publisher = self.create_publisher(Image, '/rgb/image', 10)
        
        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.robotHandle = self.sim.getObject('/myRobot')
            self.colorCam=self.sim.getObject('/myRobot/kinect/rgb')

            self.sim.startSimulation()

            self.get_logger().info('Connected to CoppeliaSim successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return
        
        # Timer to publish kinect data every 10Hz
        self.timer_rgb = self.create_timer(0.2, self.publish_camera_rgb)

        self.get_logger().info('ROS2 → CoppeliaSim Kinect RGB node started.')
    
    def publish_camera_rgb(self):
        #try:
        # Get vision sensor RGB image from CoppeliaSim
        data, resolution = self.sim.getVisionSensorImg(self.colorCam)
        data = self.sim.transformImage(data,resolution, 4)
        
        #resX, resY = resolution
        #data =  np.frombuffer(data, dtype=np.uint8).reshape(resY, resX, 3)
        #data = cv2.flip(cv2.cvtColor(data, cv2.COLOR_BGR2RGB), 0)
        #data = data.tobytes()
        
        if data is not None:
            # Create image message
            rgb_msg = Image()

            # Fill image data
            rgb_msg.header.stamp = self.get_clock().now().to_msg()
            rgb_msg.header.frame_id = "kinect"
            rgb_msg.height = resolution[1]
            rgb_msg.width = resolution[0]
            rgb_msg.encoding = 'rgb8'
            rgb_msg.is_bigendian = 1
            rgb_msg.step = resolution[0]*3
            rgb_msg.data = data
            
            # Publish the message
            self.rgb_publisher.publish(rgb_msg)
        else:
            self.get_logger().warn('Kinect RGB image not found')
    
def main(args=None):
    rclpy.init(args=args)
    node = KinectRGBNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()