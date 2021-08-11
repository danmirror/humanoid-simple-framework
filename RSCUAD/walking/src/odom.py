#! /usr/bin/env python
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import matplotlib.pyplot as plt 

rospy.init_node('odom_info')


odom_pub=rospy.Publisher ('/odom_info', Odometry)

rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom=Odometry()
header = Header()
header.frame_id='/odom'

model = GetModelStateRequest()
model.model_name='rscuad_a'

r = rospy.Rate(2)

while not rospy.is_shutdown():
   result = get_model_srv(model)
   rospy.loginfo(result.pose.position.x)
   
   plt.scatter(result.pose.position.x, result.pose.position.y)
   plt.pause(0.02)

   # rospy.logwarn("---\n")
   odom.pose.pose = result.pose
   odom.twist.twist = result.twist

   header.stamp = rospy.Time.now()
   odom.header = header

   odom_pub.publish (odom)


   r.sleep()
plt.show()