#include "myagv_custom/movement.h"

float MAXSPIN = 45.0;
float MAXSPEED = 0.9;
ros::Publisher pub;
geometry_msgs::Twist msg;

// initialized ros publisher
void init_myagv(int argc, char *argv[]) {
    ros::init(argc,argv,"publisher");
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
}

// x and y speed in m/s, turn angle in degrees, length in seconds
void move_myagv(float xspeed, float yspeed, float turnangle, float length) {
    // convert speed to myagv units
    xspeed /= 0.9;
    yspeed /= 0.9; // might be wrong

    float spinspeed = xspeed * turnangle;
    spinspeed /= 45;

    if (xspeed > 1.0) std::cout<<"exceeds maximum x speed"<<std::endl;
    if (yspeed > 1.0) std::cout<<"exceeds maximum y speed"<<std::endl;
    if (spinspeed > 1.0) std::cout<<"exceeds maximum turn speed"<<std::endl;

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if (length <= 0) {
            msg.linear.x = 0; // forward back
            msg.linear.y = 0; // left right
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0; // spin

            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
            break;
        }

        msg.linear.x = xspeed; // forward back
        msg.linear.y = yspeed; // left right
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = spinspeed; // spin

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        length -= 0.1;
    }
}

void spin_myagv(float speed, float length) {
    speed /= 30;
    if (speed > 1.0) std::cout<<"exceeds maximum turn speed"<<std::endl;

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if (length <= 0) {
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;

            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
            break;
        }

        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = speed;

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        length -= 0.1;
    }
}