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
  void BuzzPublisher();
  	
private:
  void VelCallback(const geometry_msgs::Twist::ConstPtr& twist);
  void OverrideCallback(const std_msgs::Bool::ConstPtr& override_msg);
  void DetectCallback(const std_msgs::Int32::ConstPtr& detect_msg);
  void AutoNav_Control();
  void ManualNav_Control();
  void AssessMove();
  void FlagPublisher(int x);
  
  
  void sonic_FF_Callback(const sensor_msgs::Range::ConstPtr& sonic_FF_msg);
  void sonic_FD_Callback(const sensor_msgs::Range::ConstPtr& sonic_FD_msg);
  void aIR_FR_Callback(const sensor_msgs::Range::ConstPtr& aIR_FR_msg);
  void aIR_FL_Callback(const sensor_msgs::Range::ConstPtr& aIR_FL_msg);
  void dIR_F_Callback(const sensor_msgs::Range::ConstPtr& dIR_F_msg);
  void dIR_BR_Callback(const sensor_msgs::Range::ConstPtr& dIR_BR_msg);
  void dIR_BL_Callback(const sensor_msgs::Range::ConstPtr& dIR_BL_msg);

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
  bool override;
  bool first_buzz;
  int detect;

  std_msgs::Int32 buzz_flag;
  
};


Devastator::Devastator()
{
  nav_flag_pub = nh_.advertise<std_msgs::Int32>("nav_flag", 1);
  buzz_pub = nh_.advertise<std_msgs::Bool>("buzz_flag", 1);

  override_sub = nh_.subscribe<std_msgs::Bool>("override_status", 10, &Devastator::OverrideCallback, this);
  detect_sub = nh_.subscribe<std_msgs::Int32>("Detect_Flag", 10, &Devastator::DetectCallback, this);
  vel_sub = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &Devastator::VelCallback, this);
  sonic_FF_sub = nh_.subscribe<sensor_msgs::Range>("sonic_FF", 10, &Devastator::sonic_FF_Callback,this);
  sonic_FD_sub = nh_.subscribe<sensor_msgs::Range>("sonic_FD", 10, &Devastator::sonic_FD_Callback,this);
  aIR_FR_sub = nh_.subscribe<sensor_msgs::Range>("aIR_FR", 10, &Devastator::aIR_FR_Callback,this);
  aIR_FL_sub = nh_.subscribe<sensor_msgs::Range>("aIR_FL", 10, &Devastator::aIR_FL_Callback,this);
  dIR_F_sub = nh_.subscribe<sensor_msgs::Range>("dIR_F", 10, &Devastator::dIR_F_Callback,this);
  dIR_BR_sub = nh_.subscribe<sensor_msgs::Range>("dIR_BR", 10, &Devastator::dIR_BR_Callback,this);
  dIR_BL_sub = nh_.subscribe<sensor_msgs::Range>("dIR_BL", 10, &Devastator::dIR_BL_Callback,this);
  
}

void Devastator::DetectCallback(const std_msgs::Int32::ConstPtr& detect_msg)
{
  detect = detect_msg->data;
  
  if (!detect) {
    first_buzz = true;
    AutoNav_Control();
  }
  else ManualNav_Control();
  
  if (detect && first_buzz){
    buzz_flag.data = true;
    
  }
  else {
    buzz_flag.data = false; 
  }

  buzz_pub.publish(buzz_flag);
}

void Devastator::OverrideCallback(const std_msgs::Bool::ConstPtr& override_msg)
{
  override = override_msg->data;
  if (override){
    buzz_flag.data = true;
    ManualNav_Control();
  }
  else {
    buzz_flag.data = false;
    AutoNav_Control();
  }
}

void Devastator::VelCallback(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
  linear_ = vel_msg->linear.x;
  angular_ = vel_msg->angular.z;
}

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

void Devastator::AutoNav_Control()
{
  float sonic_FF_zone = 0.30;
  float sonic_FD_zone = 0.30;
  float aIR_FR_zone = 0.30;
  float aIR_FL_zone = 0.30;
  float dIR_F_zone = 0.0;
  float dIR_BR_zone = 1.0;
  float dIR_BL_zone = 1.0;
  
  if (aIR_FR_range > aIR_FR_zone && aIR_FL_range > aIR_FL_zone)
  {
    if (sonic_FD_range > sonic_FD_zone && sonic_FF_range > sonic_FF_zone)
      FlagPublisher(1);
    else
    {
      if (dIR_BR_range == dIR_BR_zone && dIR_BL_range == dIR_BL_zone)
      {
        FlagPublisher(5);
        if (aIR_FR_range > aIR_FL_range)
          FlagPublisher(2);
        else if (aIR_FL_range > aIR_FR_range)
          FlagPublisher(3);
        else
          FlagPublisher(4);
      }
      else
      {
        if (aIR_FR_range > aIR_FL_range)
          FlagPublisher(2);
        else if (aIR_FL_range > aIR_FR_range)
          FlagPublisher(3);
        else
          FlagPublisher(4);
      }
    }
      
  }

  else if (aIR_FR_range > aIR_FR_zone && aIR_FL_range <= aIR_FL_zone)
    FlagPublisher(2);

  else if (aIR_FR_range <= aIR_FR_zone && aIR_FL_range > aIR_FL_zone)
    FlagPublisher(3);

  else if (aIR_FR_range <= aIR_FR_zone && aIR_FL_range <= aIR_FL_zone)
  {
    if (dIR_BR_range == dIR_BR_zone && dIR_BL_range == dIR_BL_zone)
      {
        FlagPublisher(5);
        if (aIR_FR_range > aIR_FL_range)
          FlagPublisher(2);
        else if (aIR_FL_range > aIR_FR_range)
          FlagPublisher(3);
        else
          FlagPublisher(4);
      }
    
    else if (dIR_BR_range != dIR_BR_zone && dIR_BL_range != dIR_BL_zone)
    {
      if ((sonic_FD_range > sonic_FD_zone && sonic_FF_range > sonic_FF_zone) ||
      (sonic_FD_range > sonic_FD_zone && sonic_FF_range > sonic_FF_zone) ||
      (sonic_FD_range > sonic_FD_zone && sonic_FF_range <= sonic_FF_zone) ||
      (sonic_FD_range <= sonic_FD_zone && sonic_FF_range > sonic_FF_zone))
      {
        if (aIR_FR_range > aIR_FL_range)
          FlagPublisher(2);
        else if (aIR_FL_range > aIR_FR_range)
          FlagPublisher(3);
        else
          FlagPublisher(4);
      }
      else
        FlagPublisher(6);
    }

    else if ((dIR_BR_range != dIR_BR_zone && dIR_BL_range == dIR_BL_zone) || (dIR_BR_range == dIR_BR_zone && dIR_BL_range != dIR_BL_zone))
    {
      if (aIR_FR_range > aIR_FL_range)
          FlagPublisher(2);
        else if (aIR_FL_range > aIR_FR_range)
          FlagPublisher(3);
        else
          FlagPublisher(4);
    }
  }
  else
    FlagPublisher(5);
  
}

void Devastator::ManualNav_Control()
{
  if (linear_ < 0.0) {
    FlagPublisher(5);
  }else if (linear_ > 0.0) {
    FlagPublisher(1);
  }else if (angular_ < 0.0) {
    FlagPublisher(2);
  }else if (angular_ > 0.0) {
    FlagPublisher(3);
  }else
    FlagPublisher(6);
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
    
    r.sleep();    
  }
  return 0;
}