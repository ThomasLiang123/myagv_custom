#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include "ros/ros.h" 
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char* argv[]) {
    // publish to myagv - listener in myagv_odometry
    ros::init(argc,argv,"publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    ros::Rate loop_rate(10);
    geometry_msgs::Twist msg;

    // forward and back (x direction)
    // slowly accelerate to 1 speed and back down to 0, then to -1 and back to 0
    bool accelerating_forward = true;
    int times_switched = 0;
    float speed = 0.0;

    msg.linear.y = 0; // left right
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0; // spin

    while(ros::ok()) {
        // if switched twice and close to 0 speed then done
        if (times_switched >= 2 && speed > -0.1 && speed < 0.1) {
            speed = 0.0;
            msg.linear.x = speed; // forward back

            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();

            fflush(stdout);
            break;
        }

        // if magnitude of speed is >= 1.0 slow down
        if ((speed >= 1.0 && accelerating_forward) || (speed <= -1.0 && !accelerating_forward)) {
            accelerating_forward = !accelerating_forward;
            times_switched++;
        }
        
        msg.linear.x = speed; // forward back

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

        fflush(stdout);

        if (accelerating_forward) {
            speed += 0.1;
        } else {
            speed -= 0.1;
        }
    }

    // left and right (y direction)
    accelerating_forward = true;
    times_switched = 0;
    speed = 0.0;

    msg.linear.x = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;

    while(ros::ok()) {
        // if switched twice and close to 0 speed then done
        if (times_switched >= 2 && speed > -0.1 && speed < 0.1) {
            speed = 0.0;
            msg.linear.y = speed;

            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();

            fflush(stdout);
            break;
        }

        // if magnitude of speed is >= 1.0 slow down
        if ((speed >= 1.0 && accelerating_forward) || (speed <= -1.0 && !accelerating_forward)) {
            accelerating_forward = !accelerating_forward;
            times_switched++;
        }
        
        msg.linear.y = speed;

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

        fflush(stdout);

        if (accelerating_forward) {
            speed += 0.1;
        } else {
            speed -= 0.1;
        }
    }

    // spin (z direction)
    accelerating_forward = true;
    times_switched = 0;
    speed = 0.0;

    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;

    while(ros::ok()) {
        // if switched twice and close to 0 speed then done
        if (times_switched >= 2 && speed > -0.1 && speed < 0.1) {
            speed = 0.0;
            msg.angular.z = speed;

            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();

            fflush(stdout);
            break;
        }

        // if magnitude of speed is >= 1.0 slow down
        if ((speed >= 5.0 && accelerating_forward) || (speed <= -5.0 && !accelerating_forward)) {
            accelerating_forward = !accelerating_forward;
            times_switched++;
        }
        
        msg.angular.z = speed;

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

        fflush(stdout);

        if (accelerating_forward) {
            speed += 0.1;
        } else {
            speed -= 0.1;
        }
    }
    return 0;
}