/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/baemin_operator/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace baemin_operator
{
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  setWindowIcon(QIcon(":/images/icon.png"));

  qnode.init();

  qRegisterMetaType<std::vector<double>>("std::vector<double>");

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(sigReadTopic()), this, SLOT(slotUpdateTopic()));
  QObject::connect(&qnode, SIGNAL(sigRcvImg(int)), this, SLOT(slotUpdateImage(int)));
  QObject::connect(&qnode, SIGNAL(sigStatusUpdate(bool)), this, SLOT(slotStatusUpdate(bool)));
  QObject::connect(&qnode, SIGNAL(sigBatteryUpdate()), this, SLOT(slotUpdateBattery()));
  QObject::connect(&qnode, SIGNAL(sigRPMUpdate()), this, SLOT(slotUpdateRPM()));
  qnode.updateTopic();
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
** Functions
*****************************************************************************/

void MainWindow::slotUpdateTopic()
{
  ui.topic_img1->clear();
  ui.topic_img2->clear();
  ui.topic_img3->clear();
  ui.topic_img1->addItems(qnode.topicList);
  ui.topic_img2->addItems(qnode.topicList);
  ui.topic_img3->addItems(qnode.topicList);
}

void MainWindow::slotUpdateImage(int num)
{
  QImage qImage((const unsigned char*)(qnode.img_raw[num].data), qnode.img_raw[num].cols, qnode.img_raw[num].rows,
                QImage::Format_RGB888);
  QPixmap qPixmap = QPixmap::fromImage(qImage.rgbSwapped());

  QLabel* targetLabel = nullptr;
  QLabel* targetFps = nullptr;
  QLabel* targetSize = nullptr;

  switch (num)
  {
    case 0:
      targetLabel = ui.img1;
      targetFps = ui.fps_main;
      targetSize = ui.img_size_main;
      break;
    case 1:
      targetLabel = ui.img2;
      targetFps = ui.fps_2;
      targetSize = ui.img_size_2;
      break;
    case 2:
      targetLabel = ui.img3;
      targetFps = ui.fps_3;
      targetSize = ui.img_size_3;
      break;
    default:
      break;
  }

  if (targetLabel)
  {
    qPixmap = qPixmap.scaled(targetLabel->size(), Qt::KeepAspectRatio);
    targetLabel->setPixmap(qPixmap);
    targetLabel->setAlignment(Qt::AlignCenter);
  }

  targetFps->setText(QString::number(qnode.fps[num] / qnode.img_count[num]));
  targetSize->setText(QString::number(qnode.img_size[num]));
}

void MainWindow::slotStatusUpdate(bool status)
{
  if (status)
  {
    ui.isconnected->setText("true");
    ui.isconnected->setStyleSheet("QLabel { color : green; }");
  }
  else if (!status)
  {
    ui.isconnected->setText("false");
    ui.isconnected->setStyleSheet("QLabel { color : red; }");
  }
}

void MainWindow::slotUpdateBattery()
{
  ui.battery_3->setText(QString::number(qnode.battery_voltage));
}

void MainWindow::slotUpdateRPM()
{
  ui.rpm_L->setText(QString::number(qnode.rpm[1]));
  ui.rpm_R->setText(QString::number(qnode.rpm[2]));
  ui.rpm_A->setText(QString::number(qnode.rpm[0]));
}

void MainWindow::on_topic_img1_currentIndexChanged(int index)
{
  QString topic = ui.topic_img1->currentText();
  qnode.img_topic[0] = topic.toStdString();
  qnode.changeTopic(0);
  std::cout << "[robot_operator] Changed cam 1 topic to : " << topic.toStdString() << std::endl;
}

void MainWindow::on_topic_img2_currentIndexChanged(int index)
{
  QString topic = ui.topic_img2->currentText();
  qnode.img_topic[1] = topic.toStdString();
  qnode.changeTopic(1);
  std::cout << "[robot_operator] Changed cam 2 topic to : " << topic.toStdString() << std::endl;
}

void MainWindow::on_topic_img3_currentIndexChanged(int index)
{
  QString topic = ui.topic_img3->currentText();
  qnode.img_topic[2] = topic.toStdString();
  qnode.changeTopic(2);
  std::cout << "[robot_operator] Changed cam 3 topic to : " << topic.toStdString() << std::endl;
}

void MainWindow::on_estop_clicked()
{
  qnode.emergencyStop();
}

}  // namespace baemin_operator
