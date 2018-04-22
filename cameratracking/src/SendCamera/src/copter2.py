#!/usr/bin/env python
import rospy
import geometry_msgs.msg

def copter2():
    pub = rospy.Publisher("copters/1/pose", geometry_msgs.msg.Pose2D, queue_size=10)
    rospy.init_node('copter2', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
	x = 0.0
	y = 0.0
	theta = 2.665287

	
	position = geometry_msgs.msg.Pose2D(x,y,theta)
	#p1 = position(x, y, theta)

        rospy.loginfo(position)
        pub.publish(position)
        rate.sleep()

if __name__ == '__main__':
    try:
        copter2()
    except rospy.ROSInterruptException:
        pass
