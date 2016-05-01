/*
 * RobotisManager.cpp
 *
 *  Created on: 2015. 11. 18.
 *      Author: zerom
 */


#include <ros/ros.h>

#include <unistd.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>

#include <robotis_controller_msgs/ControlWrite.h>
#include <robotis_controller_msgs/ControlTorque.h>
#include <robotis_controller_msgs/PublishPosition.h>

#include <robotis_controller/handler/GroupHandler.h>
#include <robotis_controller/RobotisController.h>

using namespace ROBOTIS;

RobotisController   *controller = new RobotisController();
GroupHandler        grp_handler(controller);

pthread_mutex_t     mutex = PTHREAD_MUTEX_INITIALIZER;
int                 syncwrite_addr;
int                 syncwrite_data_length;
std::vector <unsigned char> syncwrite_param;

std::vector <int>   publish_list;
bool publish_zmp = false;

int get_id_from_name(std::string name)
{
    int id = -1;

    if("_ALL_" == name)
        return 254;

    for(int i = 0; i < controller->idList.size(); i++)
    {
        id = controller->idList[i];
        if(controller->getDevice(id)->getJointName() == name)
            return id;
    }
    return -1;
}

void publish_position_callback(const robotis_controller_msgs::PublishPosition::ConstPtr& msg)
{
  if(msg->name.size() == 0 || msg->name.size() != msg->publish.size()) return;

  for(int i = 0; i < msg->name.size(); i++) {
    int id = get_id_from_name(msg->name[i]);
    if(id == -1) continue;

    if(msg->publish[i] == true) { // 6 bytes to send position,velocity and load
      grp_handler.pushBulkRead(id, controller->getDevice(id)->ADDR_PRESENT_POSITION,6);
      if ( std::find(publish_list.begin(), publish_list.end(), id) == publish_list.end() )
	publish_list.push_back(get_id_from_name(msg->name[i]));
    }
    else {
      grp_handler.deleteBulkRead(id);
      std::vector<int>::iterator iter = std::find(publish_list.begin(), publish_list.end(), id);
      if(iter != publish_list.end())
	publish_list.erase(iter);
    }
  }
}

void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    static int old_sec;
    int n = 0, id = -1;

    if(msg->name.size() == 0 || msg->name.size() != msg->position.size())
        return;

    pthread_mutex_lock(&mutex);
    syncwrite_param.clear();
    for(unsigned int idx = 0; idx < msg->name.size(); idx++)
    {
        id = get_id_from_name(msg->name[idx]);
        if(id != -1)
        {
            syncwrite_addr = controller->getDevice(id)->ADDR_GOAL_POSITION;
            syncwrite_data_length = controller->getDevice(id)->getAddrLength(syncwrite_addr);
            int pos     = controller->getDevice(id)->rad2Value(msg->position[idx]);
            syncwrite_param.resize(syncwrite_param.size() + syncwrite_data_length + 1);
            syncwrite_param[n++]  = id;
            if(syncwrite_data_length == 2)
            {
                syncwrite_param[n++]  = DXL_LOBYTE(pos);
                syncwrite_param[n++]  = DXL_HIBYTE(pos);
            }
            else if(syncwrite_data_length == 4)
            {
                syncwrite_param[n++]  = DXL_LOBYTE(DXL_LOWORD(pos));
                syncwrite_param[n++]  = DXL_HIBYTE(DXL_LOWORD(pos));
                syncwrite_param[n++]  = DXL_LOBYTE(DXL_HIWORD(pos));
                syncwrite_param[n++]  = DXL_HIBYTE(DXL_HIWORD(pos));
            }
        }
    }
    pthread_mutex_unlock(&mutex);
}

void control_write_callback(const robotis_controller_msgs::ControlWrite::ConstPtr& msg)
{
    switch(msg->length)
    {
    case 1:
        controller->write(get_id_from_name(msg->name), msg->addr, msg->value, LENGTH_1BYTE, 0);
        break;
    case 2:
        controller->write(get_id_from_name(msg->name), msg->addr, msg->value, LENGTH_2BYTE, 0);
        break;
    case 4:
        controller->write(get_id_from_name(msg->name), msg->addr, msg->value, LENGTH_4BYTE, 0);
        break;
    default:
        break;
    }
}

void control_torque_callback(const robotis_controller_msgs::ControlTorque::ConstPtr& msg)
{
    int n = 0, id = -1;

    if(msg->name.size() == 0 || msg->name.size() != msg->enable.size())
        return;

    pthread_mutex_lock(&mutex);
    syncwrite_param.clear();
    for(int i = 0; i < msg->name.size(); i++)
    {
        id = get_id_from_name(msg->name[i]);
        if(id != -1)
        {
            syncwrite_addr = controller->getDevice(id)->ADDR_TORQUE_ENABLE;
            syncwrite_data_length = controller->getDevice(id)->getAddrLength(syncwrite_addr);
            syncwrite_param.resize(syncwrite_param.size() + syncwrite_data_length + 1); // 2 : ID(1) + TORQUE_ENABLE(1)
            syncwrite_param[n++]  = id;
            if(msg->enable[i] == true)
                syncwrite_param[n++]  = 1;
            else
                syncwrite_param[n++]  = 0;
        }
    }
    pthread_mutex_unlock(&mutex);
}

void control_position_pgain_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  int n = 0, id = -1, pgain = 0;

  if(msg->name.size() == 0 || msg->name.size() != msg->effort.size())
    return;

  pthread_mutex_lock(&mutex);
  syncwrite_param.clear();
  for(unsigned int idx = 0; idx < msg->name.size(); idx++) {
    id = get_id_from_name(msg->name[idx].c_str());
    pgain = msg->effort[idx];
    if(id != -1) {
      controller->setPositionPGain(id, pgain);
    }
  }
  pthread_mutex_unlock(&mutex);
}

void publish_imu(ros::Publisher &pub)
{
  sensor_msgs::Imu msg;
  long gyro_x,gyro_y,gyro_z, acc_x,acc_y,acc_z;
  int err;

  controller->read(200, 42, &gyro_x, (LENGTH_TYPE) 2,&err); // P_GYRO_X_L
  controller->read(200, 40, &gyro_y, (LENGTH_TYPE) 2,&err); // P_GYRO_Y_L
  controller->read(200, 38, &gyro_z, (LENGTH_TYPE) 2, &err); // P_GYRO_Z_L
  controller->read(200, 44, &acc_x, (LENGTH_TYPE) 2,&err); // P_ACCEL_X_L
  controller->read(200, 46, &acc_y, (LENGTH_TYPE) 2,&err); // P_ACCEL_Y_L
  controller->read(200, 48, &acc_z, (LENGTH_TYPE) 2,&err); // P_ACCEL_Z_L

  double x,y,z,a,roll,pitch,yaw;
  y = -(acc_x - 512)/512.0; x = (acc_y - 512)/512.0; z = (acc_z - 512)/512.0;

  msg.linear_acceleration.x =x;
  msg.linear_acceleration.y =-y;
  msg.linear_acceleration.z =-z;

  a = sqrt(x*x+y*y+z*z);
  msg.header.frame_id = "base_link";
  msg.header.stamp = ros::Time::now();

  x = x/a; y = y/a; z = z/a;
  msg.orientation.x = x;
  msg.orientation.y = y;
  msg.orientation.z = z;
  msg.orientation.w = 1.0;

  msg.angular_velocity.x =gyro_x;
  msg.angular_velocity.y =gyro_y;
  msg.angular_velocity.z =gyro_z;

  roll = atan2(y,z);
  pitch = atan2(-x, sqrt(y*y + z*z));
  yaw = 0.0;

  tf::Quaternion q = tf::createQuaternionFromRPY(roll,pitch,yaw);

  msg.orientation.x = q.getX();
  msg.orientation.y = q.getY();
  msg.orientation.z = q.getZ();
  msg.orientation.w = q.getW();

  msg.orientation_covariance[0] = 2.89e-08;
  msg.orientation_covariance[4] = 2.89e-08;
  msg.orientation_covariance[8] = 2.89e-08;

  msg.angular_velocity_covariance[0] = 0.000144;
  msg.angular_velocity_covariance[4] = 0.000144;
  msg.angular_velocity_covariance[8] = 0.000144;

  msg.linear_acceleration_covariance[0] = 0.0096;
  msg.linear_acceleration_covariance[4] = 0.0096;
  msg.linear_acceleration_covariance[8] = 0.0096;

  pub.publish(msg);
}

void publish_zmp_from_fsr(ros::Publisher &rzmp_pub, ros::Publisher &lzmp_pub,
		 ros::Publisher &rfsr_pub, ros::Publisher &lfsr_pub)
{
  int err;
  geometry_msgs::WrenchStamped lfoot,rfoot;
  geometry_msgs::Pose2D lzmp,rzmp;
  long rzmp_x,rzmp_y, lzmp_x,lzmp_y;
  long fsr1,fsr2,fsr3,fsr4;

  /* ID_R_FSR = 111 ID_L_FSR = 112 in /robotis/Framework/include/FSR.h */
  controller->read(111, 26, &fsr1, (LENGTH_TYPE) 2,&err); // P_FSR1_L
  controller->read(111, 28, &fsr2, (LENGTH_TYPE) 2,&err); // P_FSR2_L
  controller->read(111, 30, &fsr3, (LENGTH_TYPE) 2,&err); // P_FSR3_L
  controller->read(111, 32, &fsr4, (LENGTH_TYPE) 2,&err); // P_FSR4_L    
  controller->read(111, 34, &rzmp_x, (LENGTH_TYPE) 1,&err); // P_FSR_X
  controller->read(111, 35, &rzmp_y, (LENGTH_TYPE) 1,&err); // P_FSR_Y

  rfoot.wrench.force.z = (fsr1 + fsr2 + fsr3 + fsr4)/1000.0; // N
  rfoot.wrench.torque.x = -41*(fsr1 - fsr2)/1000.0; // Nmm
  rfoot.wrench.torque.y = 25*(fsr3 - fsr4)/1000.0; // Nmm

  rfoot.header.stamp = ros::Time::now();
  rfoot.header.frame_id = "base_link";
  rfsr_pub.publish(rfoot);

  rzmp.x = 41*(127 - rzmp_y)/127.0; rzmp.y = 25*(127 - rzmp_x)/127.0;
  rzmp_pub.publish(rzmp);

  /*
  ROS_INFO("RFSR: %4d, %4d: %4d, %4d, %4d, %4d\r",
	   (int) rzmp_x, (int)rzmp_y, (int) fsr1, (int)fsr2, (int)fsr3,(int)fsr4);
  */
  controller->read(112, 26, &fsr1, (LENGTH_TYPE) 2,&err); // P_FSR1_L
  controller->read(112, 28, &fsr2, (LENGTH_TYPE) 2,&err); // P_FSR2_L
  controller->read(112, 30, &fsr3, (LENGTH_TYPE) 2,&err); // P_FSR3_L
  controller->read(112, 32, &fsr4, (LENGTH_TYPE) 2,&err); // P_FSR4_L    
  controller->read(112, 34, &lzmp_x, (LENGTH_TYPE) 1,&err); // P_FSR_X
  controller->read(112, 35, &lzmp_y, (LENGTH_TYPE) 1,&err); // P_FSR_Y

  lfoot.wrench.force.z = (fsr1 + fsr2 + fsr3 + fsr4)/1000.0; // N
  lfoot.wrench.torque.x = 41*(fsr1 - fsr2)/1000.0; // Nmm
  lfoot.wrench.torque.y = -25*(fsr3 - fsr4)/1000.0; // Nmm

  lfoot.header.stamp = ros::Time::now();
  lfoot.header.frame_id = "base_link";
  lfsr_pub.publish(lfoot);

  lzmp.x = 41*(lzmp_y - 127)/127.0; lzmp.y = 25*(lzmp_x - 127)/127.0;
  lzmp_pub.publish(lzmp);

  /*
  ROS_INFO("LFSR: %4d, %4d: %4d, %4d, %4d, %4d\r",
	   (int) lzmp_x, (int)lzmp_y, (int) fsr1, (int)fsr2, (int)fsr3,(int)fsr4);
  */
}

void *comm_thread_proc(void *param)
{
    ros::NodeHandle nh("~");

    ros::Publisher joint_states_pub;
    std::string topic_name;
    if(nh.getParam("publish_joint_topic_name", topic_name) == true)
        joint_states_pub = nh.advertise<sensor_msgs::JointState>(topic_name, 1);
    else
        joint_states_pub = nh.advertise<sensor_msgs::JointState>("/robot_joint_states", 1);

    ros::Publisher imu_pub;
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1);

    ros::Publisher rfsr_pub, lfsr_pub, rzmp_pub, lzmp_pub;
    if (publish_zmp){
      rfsr_pub = nh.advertise<geometry_msgs::WrenchStamped>("/foot_force_sensor/right", 1);
      lfsr_pub = nh.advertise<geometry_msgs::WrenchStamped>("/foot_force_sensor/left", 1);
      rzmp_pub = nh.advertise<geometry_msgs::Pose2D>("/zmp/right", 1);
      lzmp_pub = nh.advertise<geometry_msgs::Pose2D>("/zmp/left", 1);
    }

    ros::Publisher manager_ready_pub = nh.advertise<std_msgs::Bool>("/manager_ready", 10, true);

    int publish_rate = 125;
    if(nh.getParam("joint_state_publish_rate", publish_rate) == true)
    {
        if(publish_rate <= 0 || publish_rate > 125)
            publish_rate = 125;
    }
    else
    {
        publish_rate = 125;
    }
    ros::Rate control_rate(publish_rate);

    // check .launch file parameter
    if(controller->initialize() == false)
    {
        ROS_ERROR("robotis_controller initialize failed");
        return 0;
    }

    std_msgs::Bool ready;
    ready.data = true;
    manager_ready_pub.publish(ready);

    while(ros::ok()) {
      // Run BulkRead
      grp_handler.runBulkRead();

      if(syncwrite_param.size() > 0) {
	pthread_mutex_lock(&mutex);
	int r = grp_handler.syncWrite(syncwrite_addr, syncwrite_data_length, &syncwrite_param[0], syncwrite_param.size());
	pthread_mutex_unlock(&mutex);
      }
      syncwrite_param.clear();

      // publish joint states
      sensor_msgs::JointState joint_states;
      if(publish_list.size() > 0) {
	for(int i = 0; i < publish_list.size(); i++) {
	  int     _pos    = 0;
	  int     _id     = publish_list[i];
	  if(grp_handler.getReadData(_id, controller->getDevice(_id)->ADDR_PRESENT_POSITION, (long int*)&_pos, 2) == true) {
	    joint_states.name.push_back(controller->getDevice(_id)->getJointName());
	    joint_states.position.push_back(controller->getDevice(_id)->value2Rad(_pos));
	    /*
	    if(grp_handler.getReadData(_id, controller->getDevice(_id)->ADDR_PRESENT_VELOCITY, (long int*)&_pos, 2) == true) {
	      joint_states.velocity.push_back(_pos<1024?(-_pos):(_pos - 1024));
	    }
	    */
	    if(grp_handler.getReadData(_id, controller->getDevice(_id)->ADDR_PRESENT_LOAD, (long int*)&_pos, 2) == true) {
	      joint_states.effort.push_back(_pos<1024?(-_pos):(_pos - 1024));
	    }
	  }
	}
	joint_states.header.stamp = ros::Time::now();
	joint_states_pub.publish(joint_states);
      }
      
	publish_imu(imu_pub);
	if(publish_zmp)
	  publish_zmp_from_fsr(rzmp_pub, lzmp_pub, rfsr_pub, lfsr_pub);

        ros::spinOnce();
        control_rate.sleep();
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotis_manager");

    ros::NodeHandle nh("~");

    ros::Subscriber publish_position_sub    = nh.subscribe("/publish_position", 10, publish_position_callback);
    ros::Subscriber control_write_sub       = nh.subscribe("/control_write", 10, control_write_callback);
    ros::Subscriber control_torque_sub      = nh.subscribe("/control_torque", 10, control_torque_callback);
    ros::Subscriber control_pos_pgain_sub   = nh.subscribe("/control_pos_pgain", 10, control_position_pgain_callback);

    ros::Subscriber joint_states_sub;
    std::string topic_name;
    if(nh.getParam("subscribe_joint_topic_name", topic_name) == true)
        joint_states_sub = nh.subscribe(topic_name, 10, joint_states_callback);
    else
        joint_states_sub = nh.subscribe("/controller_joint_states", 10, joint_states_callback);

    nh.getParam("publish_zmp", publish_zmp);

    pthread_t comm_thread;
    if(pthread_create(&comm_thread, 0, comm_thread_proc, 0) != 0)
        exit(-1);

    while(ros::ok())
    { }

    return 0;
}

