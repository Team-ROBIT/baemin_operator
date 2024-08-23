/**
 * @file /include/baemin_operator/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef baemin_operator_QNODE_HPP_
#define baemin_operator_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QStringList>
#include <QTimer>

#include <hunter_msgs/HunterBmsStatus.h>
#include <hunter_msgs/HunterStatus.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace baemin_operator
{
/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  void emergencyStop();

  std::vector<std::string> img_topic;
  std::vector<ros::Subscriber> img_sub_v;
  size_t img_size[3];
  ros::Time last_img_time[3];
  int img_count[3] = {
    0,
  };
  float fps[3] = {
    0,
  };
  cv::Mat img_raw[3];

  QStringList topicList;

  double battery_voltage = 0.0;
  double rpm[3] = {
    0,
  };

  double imu[6] = {
    0,
  };

  double cmd_vel[6] = {
    0,
  };

  void changeTopic(int num);

Q_SIGNALS:
  void rosShutdown();
  void sigReadTopic();
  void sigRcvImg(int num);
  void sigStatusUpdate(bool status);
  void sigBatteryUpdate();
  void sigRPMUpdate();
  void sigIMUUpdate();
  void sigCMDUpdate();

public Q_SLOTS:
  void updateTopic();

private:
  int init_argc;
  char** init_argv;

  QTimer* timer10ms;
  QTimer* timer1s;

  void camCallback(const sensor_msgs::ImageConstPtr& msg, int num);
  void readParams();

  std::string imu_topic;

  ros::Publisher comm_pub;
  ros::Subscriber comm_sub;
  int comm_cnt = 0;
  void commStatusCallback(const std_msgs::BoolConstPtr& stat);

  ros::Publisher eStop;

  ros::Subscriber BMS_sub;
  ros::Subscriber hunter_status_sub;
  void BMSStatusCallback(const hunter_msgs::HunterBmsStatusConstPtr& stat);
  void hunterStatusCallback(const hunter_msgs::HunterStatusConstPtr& stat);

  ros::Subscriber imu_sub;
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_);

  ros::Subscriber cmd_vel_sub;
  void cmdCallback(const geometry_msgs::TwistConstPtr& cmd);

private slots:
  void onTimer10ms();
  void onTimer1s();
};

}  // namespace baemin_operator

#endif /* baemin_operator_QNODE_HPP_ */
