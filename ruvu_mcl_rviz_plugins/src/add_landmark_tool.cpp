//
// Copyright (c) 2021 RUVU Robotics
//
// @author Rokus Ottervanger
//

#include "./add_landmark_tool.hpp"

#include <ros/console.h>
#include <ruvu_mcl_msgs/LandmarkEntry.h>
#include <tf/transform_datatypes.h>

#include <QHBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <limits>
#include <string>
#include <vector>

namespace ruvu_mcl_rviz_plugins
{
//!
//! \brief The LandmarkIdDialog class for providing a dialog to set the landmark ID
//!
class LandmarkIdDialog : public QDialog
{
public:
  LandmarkIdDialog()
  {
    setLayout(new QHBoxLayout());

    id_input_ = new QSpinBox();
    id_input_->setRange(INT_MIN, INT_MAX);
    layout()->addWidget(id_input_);

    QPushButton * ok = new QPushButton("OK");
    layout()->addWidget(ok);
    connect(ok, &QPushButton::clicked, this, [this]() { accept(); });  // NOLINT
  }

  int getIdInput() { return id_input_->value(); }

private:
  QSpinBox * id_input_;
};

AddLandmarkTool::AddLandmarkTool()
{
  publisher_ = nh_.advertise<ruvu_mcl_msgs::LandmarkEntry>("add_landmark", 1);
}

void AddLandmarkTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Add landmark with ID");
}

void AddLandmarkTool::onPoseSet(double x, double y, double theta)
{
  ruvu_mcl_msgs::LandmarkEntry landmark;

  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Pose p(quat, tf::Point(x, y, 0.0));
  tf::poseTFToMsg(p, landmark.pose.pose);

  LandmarkIdDialog dialog;

  if (dialog.exec() == QDialog::Accepted) {
    int id = dialog.getIdInput();

    ROS_INFO_STREAM("Setting id: " << id);
    landmark.id = id;
    publisher_.publish(landmark);
  }
}
}  // namespace ruvu_mcl_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_mcl_rviz_plugins::AddLandmarkTool, rviz::Tool)
