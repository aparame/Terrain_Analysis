#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
import csv




class JackalController:
    def __init__(self,path):
        rospy.init_node('jackal_data_collection', log_level=rospy.INFO)

        # Velocity publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.last_publish_time = rospy.get_time()
        self.r = rospy.Rate(0.5)
        self.r_data = rospy.Rate(60)
        self.path = path
        self.my_twist = Twist()

        # Odometry subscriber and data collection setup
        rospy.Subscriber('/imu_rear', Imu, self.imu_rear_callback)
        rospy.Subscriber('/imu_front', Imu, self.imu_front_callback)
        rospy.Subscriber('/clock', Clock, self.clock_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.csv_file = open(path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Front','Rear','Linear_vel','X_position','Y_position','Time_Secs','Time_Nanoseconds'])
    

        #Initialize callback variables:
        self.imu_rear = Imu()
        self.imu_front = Imu()

        self.clock = Clock()
        self.odom = Odometry()

    def clock_callback(self, msg):
        # Callback function for /odom topic
        self.clock = msg

    def odom_callback(self, msg):
        # Callback function for /odom topic
        self.odom = msg

    def imu_front_callback(self, msg):
        # Callback function for /odom topic
        self.imu_front = msg

    def imu_rear_callback(self, msg):
        # Callback function for /odom topic
        self.imu_rear = msg   
      
    def save_data(self):
        while not rospy.is_shutdown():
            # Save data to CSV file
            if self.imu_front is not None:
                self.csv_writer.writerow([self.imu_front.linear_acceleration.y, self.imu_rear.linear_acceleration.y,
                                          self.my_twist.linear.x,self.odom.pose.pose.position.x,self.odom.pose.pose.position.y,
                                          self.clock.clock.secs,self.clock.clock.nsecs])
                self.r_data.sleep()
                # print('Collecting Data')
            else:
                rospy.loginfo("data not yet received")


    def run(self):
        # Control loop

        while not rospy.is_shutdown():
            if self.path[-10:] == "static.csv":
                self.my_twist.linear.x = 0.0
                self.my_twist.angular.z = 0.0
            else:
                self.my_twist.linear.x = float(self.path[4:9])
                self.my_twist.angular.z = 0.0

            print("X Position:",self.odom.pose.pose.position.x)    
            self.pub.publish(self.my_twist)

            self.r.sleep()

    def __del__(self):
        # Close CSV file when the node is shutting down
        if self.csv_file is not None:
            self.csv_file.close()

def main():
    try:
        path = '450_0.750.csv'
        import threading
        jackal_controller = JackalController(path)
        thread1 = threading.Thread(target=jackal_controller.run)
        thread2 = threading.Thread(target=jackal_controller.save_data)

        thread1.start()
        thread2.start()

        thread1.join()
        thread2.join()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
