#include <dji_sdk_demo/ACSL_mission.h>
#include "dji_sdk/dji_sdk.h"

using namespace DJI::OSDK;

#define MAX_TARGET 100

// service
ros::ServiceClient drone_activation_service;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;

ros::ServiceClient set_local_pos_reference;

// publisher
ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode = 255;
uint8_t current_gps_health = 0;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;
geometry_msgs::PointStamped local_position;

uint8_t g_flag_drone_local_set = 0;

int local_target_arr[MAX_TARGET][4] = {
    0,
};
int inputNum = 0;
int nowNum = 0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ACSL_mission");
  ros::NodeHandle nh;

  // service
  drone_activation_service = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

  // publisher
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

  // subscriber
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber gpsHealth = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  ros::Subscriber twistSub = nh.subscribe("turtle1/cmd_vel", 10, &twist_callback);

  ros::Subscriber ACSL_local_set = nh.subscribe("ACSL/local_position", 10, &ACSL_local_position_callback);
  ros::Subscriber ACSL_local_set_delta = nh.subscribe("ACSL/local_position_delta", 10, &ACSL_local_position_delta_callback);

  // Activate
  // if (drone_activate() == false)
  //  return -1;

  // Obtain Control Authority
  if (drone_obtainCtrlAuthority() == false)
    return -1;

  if (drone_set_local_position() == false)
    return -1;

  ROS_INFO("ACSL_TASK");

  while (1)
  {
    std::cout
        << "| Available commands:                                            |"
        << std::endl;
    std::cout
        << "| [a] Waypoint Mission                                           |"
        << std::endl;
    std::cout
        << "| [b] Hotpoint Mission                                           |"
        << std::endl;
    std::cout
        << "| [c] Land                                                       |"
        << std::endl;
    std::cout
        << "| [d] takeoff                                                    |"
        << std::endl;
    std::cout
        << "| [u] user control                                               |"
        << std::endl;
    std::cout
        << "| [k] user delta control                                         |"
        << std::endl;
    std::cout
        << "| [h] go home                                                    |"
        << std::endl;
    std::cout
        << "| [q] exit                                                       |"
        << std::endl;

    char inputChar;
    std::cin >> inputChar;
    switch (inputChar)
    {
    case 'a':
      break;
    case 'b':
      break;
    case 'c':
      drone_landing();
      break;
    case 'd':
      drone_takeoff();
      break;
    case 'u':
      drone_user_control();
      break;
    case 'k':
      drone_user_delta_control();
      break;
    case 'h':
      drone_gohome();
      break;
    default:
      return -1;
      break;
    }

    ros::spin();
  }

  return 0;
}
bool drone_user_delta_control(void)
{
  ROS_INFO("drone_user_control_delta");
  std::cout<< "you must publish ACSL/local_position_delta topic" << std::endl;
  ROS_INFO("you must publish ACSL/local_position_delta topic");
}

bool drone_user_control(void)
{
  ROS_INFO("drone_user_control");
  int inputValue[4] = {
      0,
  };

  std::cout << "you must publish ACSL/local_position topic" << std::endl;
  ROS_INFO("you must publish ACSL/local_position topic");
  /*
  std::cout << "number of local control position" << std::endl
            << ":";
  std::cin >> inputNum;

  for (int i = 0; i < inputNum; i++)
  {
    std::cout << "target position ["<<i<<"]"<<std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "control xCmd" << std::endl
              << ":";
    std::cin >> local_target_arr[i][0];
    std::cout << "control yCmd" << std::endl
              << ":";
    std::cin >> local_target_arr[i][1];
    std::cout << "control zCmd" << std::endl
              << ":";
    std::cin >> local_target_arr[i][2];
    std::cout << "control yaw" << std::endl
              << ":";
    std::cin >> local_target_arr[i][3];
    std::cout << "----------------------------------------" << std::endl;
  }*/

  //local_set_target(local_target_arr[0][0], local_target_arr[0][1], local_target_arr[0][2], local_target_arr[0][3]);


}

bool drone_obtainCtrlAuthority(void)
{
  ServiceAck ack = obtainCtrlAuthority();
  if (ack.result)
  {
    ROS_INFO("Obtain SDK control Authority successfully");
  }
  else
  {
    if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0)
    {
      ROS_INFO("Obtain SDK control Authority in progess, "
               "send the cmd again");
      obtainCtrlAuthority();
    }
    else
    {
      ROS_WARN("Failed Obtain SDK control Authority");
      return false;
    }
  }
  return true;
}

bool drone_activate(void)
{
  if (activate().result)
  {
    ROS_INFO("Activated successfully");
  }
  else
  {
    ROS_WARN("Failed activation");
    return false;
  }
  return true;
}

bool drone_takeoff(void)
{
  if (takeoff().result)
  {
    ROS_INFO("Takeoff command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending takeoff command");
    return false;
  }
}

bool drone_landing(void)
{
  ROS_INFO("land");
  if (land().result)
  {
    ROS_INFO("Land command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending land command");
    return false;
  }

  return true;
}

bool drone_gohome(void)
{
  ServiceAck ack = goHome();
  if (ack.result)
  {
    ROS_INFO("GoHome command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending goHome command");
    return false;
  }
  return true;
}

ServiceAck activate()
{
  dji_sdk::Activation activation;
  drone_activation_service.call(activation);
  if (!activation.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set,
             activation.response.cmd_id);
    ROS_WARN("ack.data: %i", activation.response.ack_data);
  }
  return ServiceAck(activation.response.result, activation.response.cmd_set,
                    activation.response.cmd_id, activation.response.ack_data);
}

ServiceAck obtainCtrlAuthority()
{
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 1;
  sdk_ctrl_authority_service.call(sdkAuthority);
  if (!sdkAuthority.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
             sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
  }
  return ServiceAck(sdkAuthority.response.result, sdkAuthority.response.cmd_set,
                    sdkAuthority.response.cmd_id,
                    sdkAuthority.response.ack_data);
}

ServiceAck takeoff()
{
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 4;
  drone_task_service.call(droneTaskControl);
  if (!droneTaskControl.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
             droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return ServiceAck(
      droneTaskControl.response.result, droneTaskControl.response.cmd_set,
      droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
}

ServiceAck land()
{
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 6;
  drone_task_service.call(droneTaskControl);
  if (!droneTaskControl.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
             droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return ServiceAck(
      droneTaskControl.response.result, droneTaskControl.response.cmd_set,
      droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
}

ServiceAck goHome()
{
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 1;
  drone_task_service.call(droneTaskControl);
  if (!droneTaskControl.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set, droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return {droneTaskControl.response.result, droneTaskControl.response.cmd_set,
          droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data};
}

// callback function

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg)
{
  //ROS_INFO("attitude_callback");
  current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  //ROS_INFO("local_position_callback ");
  current_local_pos = msg->point;

  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  local_position = *msg;
  double xCmd, yCmd, zCmd;
  if (g_flag_drone_local_set == 1)
  {
    //down sampling
    if (elapsed_time > ros::Duration(0.02))
    {
      start_time = ros::Time::now();

      if (current_gps_health > 3)
      {
        //ROS_INFO("gps_health is good");
        local_position_ctrl(xCmd, yCmd, zCmd);
      }
      else
      {
      }
    }
  }
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr &msg)
{
  //ROS_INFO("gps_health_callback");
  current_gps_health = msg->data;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  //ROS_INFO("gps_callbcak");
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr &msg)
{
  //ROS_INFO("flight_status_callbcak");
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr &msg)
{
  //ROS_INFO("display_mode_callback");
  display_mode = msg->data;
}

void twist_callback(const geometry_msgs::Twist &msg)
{
  //ROS_INFO("twist_callback");
}

void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd)
{
  //ROS_INFO("local_position_ctrl ");
  xCmd = target_offset_x - local_position.point.x;
  yCmd = target_offset_y - local_position.point.y;
  zCmd = target_offset_z;

  sensor_msgs::Joy controlPosYaw;
  controlPosYaw.axes.push_back(xCmd);
  controlPosYaw.axes.push_back(yCmd);
  controlPosYaw.axes.push_back(zCmd);
  controlPosYaw.axes.push_back(target_yaw);
  ctrlPosYawPub.publish(controlPosYaw);

  // 0.1m or 10cms is the minimum error to reach target in x y and z axes.
  // This error threshold will have to change depending on aircraft/payload/wind conditions.

  //ROS_INFO("\r\n xCmd = %f \r\n yCmd = %f \r\n zCmd = %f\r\n",xCmd,yCmd,zCmd);
  if (((std::abs(xCmd)) < 0.1) && ((std::abs(yCmd)) < 0.1) &&
      (local_position.point.z > (target_offset_z - 0.1)) && (local_position.point.z < (target_offset_z + 0.1)))
  {
      //ROS_INFO("user control finished");
  }
}

void ACSL_local_position_callback(const dji_sdk::ACSL_local_position::ConstPtr& msg)
{
  //ROS_INFO("ACSL_local_position_callback");
  local_set_target(msg->targetX, msg->targetY, msg->targetZ, msg->targetYaw);
  g_flag_drone_local_set = 1;
  ROS_INFO("MessageSequence : %d", msg->seq);
  ROS_INFO("targetX : %f", msg->targetX);
  ROS_INFO("targetY : %f", msg->targetY);
  ROS_INFO("targetZ : %f", msg->targetZ);
  ROS_INFO("targetYaw : %f", msg->targetYaw);

}

void ACSL_local_position_delta_callback(const dji_sdk::ACSL_local_position_delta::ConstPtr& msg)
{
  ROS_INFO("ACSL_local_position_delta_callback");
  
  g_flag_drone_local_set = 1;
  local_set_target(local_position.point.x + msg->deltaX, local_position.point.y + msg->deltaY, local_position.point.z + msg->deltaZ, target_yaw + msg->deltaYaw);
  ROS_INFO("MessageSequence : %d", msg->seq);
  ROS_INFO("targetX : %f", local_position.point.x + msg->deltaX);
  ROS_INFO("targetY : %f", local_position.point.y + msg->deltaY);
  ROS_INFO("targetZ : %f", local_position.point.z + msg->deltaZ);
  ROS_INFO("targetYaw : %f", target_yaw + msg->deltaYaw);

}

bool local_set_target(float x, float y, float z, float yaw)
{
  ROS_INFO("local_set_target");
  target_offset_x = x;
  target_offset_y = y;
  target_offset_z = z;
  target_yaw = yaw;
  return true;
}

bool drone_set_local_position(void)
{
  ROS_INFO("set_local_position");
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}