#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <string>
#include <thread>


class JackalController{

public:
    JackalController(const std::string& path): path_(path) {
        ros::NodeHandle nh;


        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
        sub_odom_ = nh.subscribe("/odom",1,&JackalController::odom_callback,this);
        sub_imu_front_left_ = nh.subscribe("/imu_front",1 &JackalController::imufrontcallback(),this);

        sub_imu_rear_left_ = nh.subscribe("/imu_rear",1 &JackalController::imurearcallback(),this);

        sub_clock = nh.subscribe("/clock",1, &JackalController::clock_callback(),this);

        csv_file_.open(path_);
        csv_file_ << "Front,Rear,Linear_vel,X_position,Y_position,Time_Secs,Time_Nanoseconds\n";
    }

    ~JackalController(){
        if (csv_file_.is_open()){
            csv_file_.close();
        }
    }

    void run(){
        while (ros::ok()) {
            geometry_msgs::Twist msg;

            if (path_.substr(path_.size() - 10) == "static.csv") {
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
            } else {
                msg.linear.x = std::stof(path_.substr(4, 5));
                msg.angular.z = 0.0;
            }

            pub.publish(msg);ros::init(argc,argv,"jackal_data_collection");
            ros::spinOnce();
            loop_rate_.sleep();
        }
    }

    void save_data(){
        while (ros::ok()){
            if(imu_data_received_){
                csv_file_ << imu_front_linear_acceleration.y << ","
                          << imu_rear_.linear_acceleration.y << ","
                          << twist_.linear.x << ","
                          << odom_.pose.pose.position.x << ","
                          << odom_.pose.pose.position.y << ","
                          << clock_.clock.secs << ","
                          << clock_.clock.nsecs << "\n";
            }

            ros::spinOnce();
            loop_rate_data.sleep();
        }
    }
        

private:
    //initialize the variables to their specific type
    ros::Publisher pub;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_imu_front_left_;
    ros::Subscriber sub_imu_front_right_;
    ros::Subscriber sub_imu_rear_left_;
    ros::Subscriber sub_imu_rear_right_;
    ros::Subscriber sub_clock;

    //Intitialize the variables to subscribe to specific ros msgs
    nav_msgs::Odometry odom_;
    sensor_msgs::Imu imu_front_left; sensor_msgs::Imu imu_front_right; sensor_msgs::Imu imu_rear_left; sensor_msgs::Imu imu_rear_right;
    rosgraph_msgs::Clock clock_;

    std::ofstream csv_file_;
    std::string path_;
    ros::Rate loop_rate_{0.5};
    ros::Rate loop_rate_data_{60};
    bool imu_data_received_ = false;

    //const means funciion will not change the value and & operator means msg is sent by reference from source rather than value, and ConstPtr is a smart pointer to nav_msgs::Odometry
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
        odom_ = *msg;
    }
    
    void imufrontleftcallback(const sensor_msgs::Imu::ConstPtr& msg){
        imu_front_left_ = *msg;
        imu_data_received = true;
    }   
    
    void imufrontrightcallback(const sensor_msgs::Imu::ConstPtr& msg){
        imu_front_right_ = *msg;
    }

    void imurearrightcallback(const sensor_msgs::Imu::ConstPtr& msg){
        imu_rear_right_ = *msg;
    }
    
    void imurearleftcallback(const sensor_msgs::Imu::ConstPtr& msg){
        imu_rear_left_ = *msg;
    }
    
    void clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg){
        clock_ = *msg;
    }


};

int main(int argc, char** argv){
    ros::init(argc,argv,"jackal_data_collection");

    if (argc != 2){
        std::cerr << "Usage: "<<argv[0]<<" csv_file_path"<<std::endl;
        return 1;
    }

    std::string path(argv[1]);  //path to the csv file

    
    JackalController controller(path);

    std::thread t1(&JackalController::run, &controller);
    std::thread t1(&JackalController::save_data, &controller);

    t1.join();
    t2.join();
    return 0;

}