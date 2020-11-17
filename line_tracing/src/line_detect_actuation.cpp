#include <ros/ros.h>
#include "core_msgs/line_segments.h"
#include "core_msgs/yolomsg.h"
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <std_msgs/Float64.h>
#include <chrono>
using namespace std;

ros::Publisher pub_right_front_wheel;
ros::Publisher pub_left_front_wheel;
ros::Publisher pub_left_rear_wheel;
ros::Publisher pub_right_rear_wheel;

std_msgs::Float64 right_front;
std_msgs::Float64 left_front;
std_msgs::Float64 left_rear;
std_msgs::Float64 right_rear;

int correct = 0;

bool isenabled = true;
bool stop = false;
bool start_zone = true;
double end_secs;
double begin_secs;

void decision_center(const core_msgs::line_segments::ConstPtr& msg) {
    if (!isenabled) return;
    if (stop) return;
    int count; //this is the number of actual data points received
    int array_size = msg->size;
    if (array_size == 0)
        return;

    count = array_size;
    cout <<"array_size: " << count << endl;
    cout << "st " << start_zone << endl;
    if (start_zone) {
      for (int i =0;i < array_size;i++)
      {
          if (isnan(msg->com_x[i]))
          {
              count=i;
              cout << "count = " << count << "    " << msg->com_x[0] << endl;
              break;
          }
      }

     if (count>0){
          double error = msg->com_x[0];
          right_front.data=25;
          left_front.data=25;
          left_rear.data=25;
          right_rear.data=25;

          if (abs(error)<55){
              correct = 0;
          }

          if (abs(error)>80 || correct==1){
              correct =1;
              if (error>0){
                  left_front.data = 30; //50
                  right_front.data = -25;//-48
                  left_rear.data = 0; //20
                  right_rear.data = 0; //-20
              }
              else{
                  right_front.data = 30;
                  left_front.data = -25;
                  right_rear.data = 0;//chaznge
                  left_rear.data = 0;
              }
            }
      }

  /*
          else if (abs(error)>80 || correct == 2){
              correct =2;
              if (error>0){
                  right_front.data = -20;
                  left_rear.data=20;
                  right_rear.data = -20;
              }
              else{
                  left_front.data = -20;
                  right_rear.data = 20;
                  left_rear.data = -20;
              }
          }
  */

          pub_left_front_wheel.publish(left_front);
          pub_right_front_wheel.publish(right_front);
          pub_left_rear_wheel.publish(left_rear);
          pub_right_rear_wheel.publish(right_rear);

          cout << left_front.data << "    " << right_front.data << "    " << right_rear.data
              << "    " << left_rear.data << endl;
    }
    else
    {
      for (int i =0;i < array_size;i++)
      {
          if (isnan(msg->com_x[i]))
          {
              count=i;
              cout << "count = " << count << "    " << msg->com_x[0] << endl;
              break;
          }
      }

     if (count>0){
          double error = msg->com_x[0];
          right_front.data=50;
          left_front.data=50;
          left_rear.data=50;
          right_rear.data=50;

          if (abs(error)<55){
              correct = 0;
          }

          if (abs(error)>80 || correct==1){
              correct =1;
              if (error>0){
                  left_front.data = 50; //50
                  right_front.data = -45;//-48
                  left_rear.data = 20; //20
                  right_rear.data = -20; //-20
              }
              else{
                  right_front.data = 50;
                  left_front.data = -45;
                  right_rear.data = 20;//chaznge
                  left_rear.data = -20;
              }
            }


  /*
          else if (abs(error)>80 || correct == 2){
              correct =2;
              if (error>0){
                  right_front.data = -20;
                  left_rear.data=20;
                  right_rear.data = -20;
              }
              else{
                  left_front.data = -20;
                  right_rear.data = 20;
                  left_rear.data = -20;
              }
          }
  */

          pub_left_front_wheel.publish(left_front);
          pub_right_front_wheel.publish(right_front);
          pub_left_rear_wheel.publish(left_rear);
          pub_right_rear_wheel.publish(right_rear);

          cout << left_front.data << "    " << right_front.data << "    " << right_rear.data
              << "    " << left_rear.data << endl;
            }
        }
     }

void emergency_stop(const core_msgs::yolomsg::ConstPtr& msg) {
    int detected_num = msg->num;

    if (detected_num>1){
      stop = true;
      start_zone =false;
      begin_secs = ros::Time::now().toSec();
      end_secs = 0;
      cout << "return is the "<<detected_num << endl;
      cout <<"time:  " << end_secs-begin_secs << endl;

      while(end_secs-begin_secs < 0.8){
        right_front.data = -15;
        left_front.data = -15;
        right_rear.data = -15;
        left_rear.data = -15;
        pub_left_front_wheel.publish(left_front);
        pub_right_front_wheel.publish(right_front);
        pub_left_rear_wheel.publish(left_rear);
        pub_right_rear_wheel.publish(right_rear);
        cout << left_front.data << "    " << right_front.data << "    " << right_rear.data
            << "    " << left_rear.data << endl;
        end_secs = ros::Time::now().toSec();
        cout <<"time:  " << end_secs-begin_secs << endl;
        cout << "2:::::"<<detected_num << endl;
        }
      }
    stop = false;
  }

int main (int argc, char **argv)
{
    ros::init (argc, argv, "line_detect_actuation_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/segments", 10, decision_center);
    ros::Subscriber sub2 = nh.subscribe("/detected_objects", 10, emergency_stop);
    pub_right_front_wheel = nh.advertise<std_msgs::Float64>("model20/right_front_wheel_velocity_controller/command", 10);
    pub_left_front_wheel = nh.advertise<std_msgs::Float64>("model20/left_front_wheel_velocity_controller/command", 10);
    pub_right_rear_wheel = nh.advertise<std_msgs::Float64>("model20/right_rear_wheel_velocity_controller/command", 10);
    pub_left_rear_wheel = nh.advertise<std_msgs::Float64>("model20/left_rear_wheel_velocity_controller/command", 10);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        ros::spinOnce();
        bool entrance_finished;
        if (isenabled && nh.getParam("/entrance_finished", entrance_finished)) {
            if (entrance_finished) {
                isenabled = false;
            }
        }

        loop_rate.sleep();
    }
    return 0;
}
