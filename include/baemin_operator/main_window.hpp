/**
 * @file /include/baemin_operator/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef baemin_operator_MAIN_WINDOW_H
#define baemin_operator_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace baemin_operator
{
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = 0);
  ~MainWindow();

public Q_SLOTS:
  void slotUpdateTopic();
  void slotUpdateImage(int num);
  void slotStatusUpdate(bool status);
  void slotUpdateBattery();
  void slotUpdateRPM();

  void on_topic_img1_currentIndexChanged(int index);
  void on_topic_img2_currentIndexChanged(int index);
  void on_topic_img3_currentIndexChanged(int index);
  void on_estop_clicked();

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
};

}  // namespace baemin_operator

#endif  // baemin_operator_MAIN_WINDOW_H
