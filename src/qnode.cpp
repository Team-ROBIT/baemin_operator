/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/baemin_operator/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace baemin_operator
{
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv)
{
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  delete timer10ms;
  delete timer1s;
  wait();
}

bool QNode::init()
{
  ros::init(init_argc, init_argv, "baemin_operator");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  readParams();

  for (int i = 0; i < img_topic.size(); i++)
  {
    img_sub_v.push_back(
        n.subscribe<sensor_msgs::Image>(img_topic[i], 1, boost::bind(&QNode::camCallback, this, _1, i)));
  }

  // Publishers
  eStop = n.advertise<std_msgs::Bool>("ESTOP", 1);
  comm_pub = n.advertise<std_msgs::Bool>("/comm_stat/operator", 1);

  // Subscribers
  comm_sub = n.subscribe<std_msgs::Bool>("/comm_stat/robot", 1, &QNode::commStatusCallback, this);
  BMS_sub = n.subscribe<hunter_msgs::HunterBmsStatus>("/BMS_status", 1, &QNode::BMSStatusCallback, this);
  hunter_status_sub = n.subscribe<hunter_msgs::HunterStatus>("/hunter_status", 1, &QNode::hunterStatusCallback, this);

  timer10ms = new QTimer(this);
  timer1s = new QTimer(this);
  connect(timer10ms, &QTimer::timeout, this, &QNode::onTimer10ms);
  connect(timer1s, &QTimer::timeout, this, &QNode::onTimer1s);
  timer10ms->start(10);
  timer1s->start(1000);

  start();
  return true;
}

void QNode::camCallback(const sensor_msgs::ImageConstPtr& msg, int num)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  img_raw[num] = cv_ptr->image;
  img_size[num] = msg->data.size();
  if (img_count[num] > 0)
  {
    ros::Duration duration = msg->header.stamp - last_img_time[num];
    double vFps = 1.0 / duration.toSec();
    fps[num] += vFps;
  }
  last_img_time[num] = msg->header.stamp;
  img_count[num]++;
  Q_EMIT sigRcvImg(num);
}

void QNode::run()
{
  ros::Rate loop_rate(33);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::readParams()
{
  std::string data;
  ros::param::get("/baemin_operator/cam1_topic", data);
  std::cout << "[baemin_operator] Cam 1 topic : " << data.c_str() << std::endl;
  img_topic.push_back(data);
  ros::param::get("/baemin_operator/cam2_topic", data);
  std::cout << "[baemin_operator] Cam 2 topic : " << data.c_str() << std::endl;
  img_topic.push_back(data);
  ros::param::get("/baemin_operator/cam3_topic", data);
  std::cout << "[baemin_operator] Cam 3 topic : " << data.c_str() << std::endl;
  img_topic.push_back(data);
}

void QNode::updateTopic()
{
  ros::master::V_TopicInfo topics;
  if (ros::master::getTopics(topics))
  {
    topicList.clear();
    for (const auto& topic : topics)
    {
      if (topic.datatype == "sensor_msgs/Image")
      {
        topicList << QString::fromStdString(topic.name);
      }
    }
    Q_EMIT sigReadTopic();
    return;
  }
  else
  {
    ROS_ERROR("Failed to retrieve topics.");
    return;
  }
}

void QNode::commStatusCallback(const std_msgs::BoolConstPtr& stat)
{
  comm_cnt = 0;
  Q_EMIT sigStatusUpdate(true);
}

void QNode::BMSStatusCallback(const hunter_msgs::HunterBmsStatusConstPtr& stat)
{
  battery_voltage = stat->battery_voltage;
  Q_EMIT sigBatteryUpdate();
}

void QNode::hunterStatusCallback(const hunter_msgs::HunterStatusConstPtr& stat)
{
  for (int i = 0; i < 3; i++)
  {
    rpm[i] = stat->motor_states[i].rpm;
  }
  Q_EMIT sigRPMUpdate();
}

void QNode::onTimer10ms()
{
  std_msgs::Bool boolean_msg;
  boolean_msg.data = true;
  comm_pub.publish(boolean_msg);
}

void QNode::onTimer1s()
{
  comm_cnt++;
  if (comm_cnt >= 5)
  {
    ROS_ERROR("COMMUNICATION LOST. PLEASE CHECK YOUR CONNECTION.");
    Q_EMIT sigStatusUpdate(false);
  }
}

void QNode::emergencyStop()
{
  std_msgs::Bool msg;
  msg.data = true;
  eStop.publish(msg);
}

void QNode::changeTopic(int num)
{
  img_sub_v[num].shutdown();
  ros::NodeHandle n;
  img_sub_v[num] = n.subscribe<sensor_msgs::Image>(img_topic[num], 1, boost::bind(&QNode::camCallback, this, _1, num));
}
}  // namespace baemin_operator
