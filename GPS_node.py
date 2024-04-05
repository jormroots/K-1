import rospy
from GPS_publisher import RosNMEADriver

if __name__ == '__main__':
    rospy.init_node('GPS_publisher')
    
    try:
        GPS_publisher = RosNMEADriver()
        GPS_publisher.run()
        
    except rospy.ROSInterruptException:
        pass