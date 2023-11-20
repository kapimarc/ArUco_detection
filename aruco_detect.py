# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import numpy as np
from cv2 import aruco
import cv2
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.Image=Image()
        self._amera_matrix=0
        self.dist_coeff=0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.subscription = self.create_subscription(Image,'camera/image_raw',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info, 10)
    
    def camera_info(self, msg):
        self.camera_matrix = np.reshape(msg.k, (3, 3))
        self.dist_coeff = np.reshape(msg.d,(1,5))
        
    def listener_callback(self, msg):
        self.Image=msg
        ####
        image=np.array(msg.data).reshape(msg.height,msg.width,3)
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        arucoParams = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict,parameters=arucoParams)
        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, 0.08, self.camera_matrix, self.dist_coeff)
        try:
            transform_stamped = self.tf_buffer.lookup_transform('map', 'camera_link', rclpy.time.Time())
            
        except:
            print("No data found")
            return
        if ids is not None:
            for i, id_aruco in enumerate(ids):
                aruco_pose = Pose()
                aruco_pose.position.x = float(tvec[i][0][0])
                aruco_pose.position.y = float(tvec[i][0][1])
                aruco_pose.position.z = float(tvec[i][0][2])

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec[i][0]))[0]
                #print(rot_matrix)
                r = R.from_matrix(rot_matrix[0:3, 0:3])
                quat = r.as_quat() 

                aruco_pose.orientation.x = quat[0]
                aruco_pose.orientation.y = quat[1]
                aruco_pose.orientation.z = quat[2]
                aruco_pose.orientation.w = quat[3]
                
                transformed_pose = tf2_geometry_msgs.do_transform_pose(aruco_pose, transform_stamped)
                print(f"Aruco ID: {id_aruco} Pose of arcuo: {transformed_pose.position}")
        
      
    
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
