//  Devastator bot control module
//  October 23, 2018
//  DOST-ASTI, RDD Team
/////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

class Devastator {
public:
  Devastator();
  //Create a public class member to publish flags used for robot movement.
  void FlagPublisher(int x);

  //Set Get sensor data variables.
  float sonic_FF_data();
  float sonic_FD_data();
  float aIR_FR_data();
  float aIR_FL_data();
  float dIR_F_data();
  float dIR_BR_data();
  float dIR_BL_data();

  //Override and geometry variables.
  bool override_flag();
  float* vel_data();


private:
  //Override Flags and geometry data subscribers.
  void VelCallback(const geometry_msgs::Twist::ConstPtr& twist);
  void OverrideCallback(const std_msgs::Bool::ConstPtr& override_msg);

  //Sensor data subscribers
  void sonic_FF_Callback(const sensor_msgs::Range::ConstPtr& sonic_FF_msg);
  void sonic_FD_Callback(const sensor_msgs::Range::ConstPtr& sonic_FD_msg);
  void aIR_FR_Callback(const sensor_msgs::Range::ConstPtr& aIR_FR_msg);
  void aIR_FL_Callback(const sensor_msgs::Range::ConstPtr& aIR_FL_msg);
  void dIR_F_Callback(const sensor_msgs::Range::ConstPtr& dIR_F_msg);
  void dIR_BR_Callback(const sensor_msgs::Range::ConstPtr& dIR_BR_msg);
  void dIR_BL_Callback(const sensor_msgs::Range::ConstPtr& dIR_BL_msg);

  //ROS variables
  ros::NodeHandle nh_;
  ros::Publisher nav_flag_pub;
  ros::Publisher buzz_pub;
  ros::Subscriber vel_sub;
  ros::Subscriber override_sub;
  ros::Subscriber sonic_FF_sub;
  ros::Subscriber sonic_FD_sub;
  ros::Subscriber aIR_FR_sub;
  ros::Subscriber aIR_FL_sub;
  ros::Subscriber dIR_F_sub;
  ros::Subscriber dIR_BR_sub;
  ros::Subscriber dIR_BL_sub;
  ros::Subscriber detect_sub;

  //Set variables for sensor data
  float sonic_FD_range;
  float sonic_FF_range;
  float aIR_FR_range;
  float aIR_FL_range;
  float dIR_F_range;
  float dIR_BR_range;
  float dIR_BL_range;
  float linear_,angular_;
};


Devastator::Devastator()
{
  nav_flag_pub = nh_.advertise<std_msgs::Int32>("nav_flag", 1);
  buzz_pub = nh_.advertise<std_msgs::Bool>("buzz_flag", 1);
  override_sub = nh_.subscribe<std_msgs::Bool>("override_status", 10, &Devastator::OverrideCallback, this);
  vel_sub = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &Devastator::VelCallback, this);
  sonic_FF_sub = nh_.subscribe<sensor_msgs::Range>("sonic_FF", 10, &Devastator::sonic_FF_Callback,this);
  sonic_FD_sub = nh_.subscribe<sensor_msgs::Range>("sonic_FD", 10, &Devastator::sonic_FD_Callback,this);
  aIR_FR_sub = nh_.subscribe<sensor_msgs::Range>("aIR_FR", 10, &Devastator::aIR_FR_Callback,this);
  aIR_FL_sub = nh_.subscribe<sensor_msgs::Range>("aIR_FL", 10, &Devastator::aIR_FL_Callback,this);
  dIR_F_sub = nh_.subscribe<sensor_msgs::Range>("dIR_F", 10, &Devastator::dIR_F_Callback,this);
  dIR_BR_sub = nh_.subscribe<sensor_msgs::Range>("dIR_BR", 10, &Devastator::dIR_BR_Callback,this);
  dIR_BL_sub = nh_.subscribe<sensor_msgs::Range>("dIR_BL", 10, &Devastator::dIR_BL_Callback,this);
}


void Devastator::OverrideCallback(const std_msgs::Bool::ConstPtr& override_msg)
{
  override = override_msg->data;
}

//Get geometry data for robot displacement values from ROS geometry publisher.
void Devastator::VelCallback(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
  linear_ = vel_msg->linear.x;
  angular_ = vel_msg->angular.z;
}

//Get sensor values from individual sensor publishers.
void Devastator::sonic_FF_Callback(const sensor_msgs::Range::ConstPtr& sonic_FF_msg)
{
  sonic_FF_range = sonic_FF_msg->range;
}

void Devastator::sonic_FD_Callback(const sensor_msgs::Range::ConstPtr& sonic_FD_msg)
{
  sonic_FD_range = sonic_FD_msg->range;
}

void Devastator::aIR_FR_Callback(const sensor_msgs::Range::ConstPtr& aIR_FR_msg)
{
  aIR_FR_range = aIR_FR_msg->range;
}

void Devastator::aIR_FL_Callback(const sensor_msgs::Range::ConstPtr& aIR_FL_msg)
{
  aIR_FL_range = aIR_FL_msg->range;
}
void Devastator::dIR_F_Callback(const sensor_msgs::Range::ConstPtr& dIR_F_msg)
{
  dIR_F_range = dIR_F_msg->range;
}
void Devastator::dIR_BR_Callback(const sensor_msgs::Range::ConstPtr& dIR_BR_msg)
{
  dIR_BR_range = dIR_BR_msg->range;
}
void Devastator::dIR_BL_Callback(const sensor_msgs::Range::ConstPtr& dIR_BL_msg)
{
  dIR_BL_range = dIR_BL_msg->range;
}


//Get Functions
bool Devastator::override_flag()
{
  return override;
}

float* Devastator::vel_data()
{
  float* vel = new float[2];
  vel[0] = linear_;
  vel[1] = angular_;
  return vel;
}
float Devastator::sonic_FF_data()
{
  return sonic_FF_range;
}

float Devastator::sonic_FD_data()
{
  return sonic_FD_range;
}

float Devastator::aIR_FR_data()
{
  return aIR_FR_range;
}
float Devastator::aIR_FL_data()
{
  return aIR_FL_range;
}
float Devastator::dIR_F_data()
{
  return dIR_F_range;
}
float Devastator::dIR_BR_data()
{
  return dIR_BR_range;
}
float Devastator::dIR_BL_data()
{
  return dIR_BL_range;
}

void Devastator::FlagPublisher(int x)
{
  std_msgs::Int32 flag_data;
  flag_data.data = x;
  nav_flag_pub.publish(flag_data);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "devastator_control");
  ROS_INFO("Node Started...");
  Devastator devastator;
  ros::Rate r(10);
  while (ros::ok()){
    ros::spinOnce();

    //If no override is detected. Navigate autonomously.
    if (!devastator.override_flag())
    {
      float sonic_FF_zone = 0.30;
      float sonic_FD_zone = 0.40;
      float aIR_FR_zone = 0.30;
      float aIR_FL_zone = 0.30;
      float dIR_F_zone = 0.0;
      float dIR_BR_zone = 1.0;
      float dIR_BL_zone = 1.0;


      if (devastator.aIR_FR_data() > aIR_FR_zone && devastator.aIR_FL_data() > aIR_FL_zone)
      {
        if (devastator.sonic_FD_data() > sonic_FD_zone && devastator.sonic_FF_data() > sonic_FF_zone and devastator.dIR_F_data()==dIR_F_zone)
          devastator.FlagPublisher(1);
        else
        {
          if (devastator.dIR_BR_data() && devastator.dIR_BL_data())
          {
            devastator.FlagPublisher(5);
            if (devastator.aIR_FR_data() > devastator.aIR_FL_data())
              devastator.FlagPublisher(2);
            else if (devastator.aIR_FL_data() > devastator.aIR_FR_data())
              devastator.FlagPublisher(3);
            else
              devastator.FlagPublisher(4);
          }
          else
          {
            if (devastator.aIR_FR_data() > devastator.aIR_FL_data())
              devastator.FlagPublisher(2);
            else if (devastator.aIR_FL_data() > devastator.aIR_FR_data())
              devastator.FlagPublisher(3);
            else
              devastator.FlagPublisher(4);
          }
        }

      }

      else if (devastator.aIR_FR_data() > aIR_FR_zone && devastator.aIR_FL_data() <= aIR_FL_zone)
        devastator.FlagPublisher(2);

      else if (devastator.aIR_FR_data() <= aIR_FR_zone && devastator.aIR_FL_data() > aIR_FL_zone)
        devastator.FlagPublisher(3);

      else if (devastator.aIR_FR_data() <= aIR_FR_zone && devastator.aIR_FL_data() <= aIR_FL_zone)
      {
        if (devastator.dIR_BR_data() && devastator.dIR_BL_data())
          {
            devastator.FlagPublisher(5);
            if (devastator.aIR_FR_data() > devastator.aIR_FL_data())
              devastator.FlagPublisher(2);
            else if (devastator.aIR_FL_data() > devastator.aIR_FR_data())
              devastator.FlagPublisher(3);
            else
              devastator.FlagPublisher(4);
          }

        else if (!devastator.dIR_BR_data() && !devastator.dIR_BL_data())
        {
          if ((devastator.sonic_FD_data() > sonic_FD_zone && devastator.sonic_FF_data() > sonic_FF_zone && devastator.dIR_F_data()==dIR_F_zone) ||
          (devastator.sonic_FD_data() > sonic_FD_zone && devastator.sonic_FF_data() > sonic_FF_zone && devastator.dIR_F_data() != dIR_F_zone) ||
          (devastator.sonic_FD_data() > sonic_FD_zone && devastator.sonic_FF_data() <= sonic_FF_zone && devastator.dIR_F_data()==dIR_F_zone) ||
          (devastator.sonic_FD_data() <= sonic_FD_zone && devastator.sonic_FF_data() > sonic_FF_zone && devastator.dIR_F_data()==dIR_F_zone))
          {
            if (devastator.aIR_FR_data() > devastator.aIR_FL_data())
              devastator.FlagPublisher(2);
            else if (devastator.aIR_FL_data() > devastator.aIR_FR_data())
              devastator.FlagPublisher(3);
            else
              devastator.FlagPublisher(4);
          }
          else
            devastator.FlagPublisher(6);
        }

        else if ((!devastator.dIR_BR_data() && devastator.dIR_BL_data()) || (devastator.dIR_BR_data() && !devastator.dIR_BL_data()))
        {
          if (devastator.aIR_FR_data() > devastator.aIR_FL_data())
              devastator.FlagPublisher(2);
            else if (devastator.aIR_FL_data() > devastator.aIR_FR_data())
              devastator.FlagPublisher(3);
            else
              devastator.FlagPublisher(4);
        }
      }
      else
        devastator.FlagPublisher(5);
    }
    //Else navigate manually.
    else
    {
      float* vel = devastator.vel_data();

      if (vel[0] < 0.0) {
        devastator.FlagPublisher(5);
      }else if (vel[0] > 0.0) {
        devastator.FlagPublisher(1);
      }else if (vel[1] < 0.0) {
        devastator.FlagPublisher(2);
      }else if (vel[1] > 0.0) {
        devastator.FlagPublisher(3);
      }else
        devastator.FlagPublisher(6);
    }

    r.sleep();
  }
  return 0;
}
