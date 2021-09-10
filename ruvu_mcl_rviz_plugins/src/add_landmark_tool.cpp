//
// Copyright (c) 2021 RUVU Robotics
//
// @author Rokus Ottervanger
//

#include "./add_landmark_tool.hpp"

#include <ros/console.h>
#include <ruvu_mcl_msgs/AddLandmark.h>
#include <rviz/visualization_manager.h>
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
  client_ = nh_.serviceClient<ruvu_mcl_msgs::AddLandmark>("add_landmark");
}

void AddLandmarkTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Add landmark with ID");
}

void AddLandmarkTool::onPoseSet(double x, double y, double theta)
{
  ruvu_mcl_msgs::AddLandmark req;

  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Pose p(quat, tf::Point(x, y, 0.0));
  tf::poseTFToMsg(p, req.request.landmark.pose.pose);

  LandmarkIdDialog dialog;

  if (dialog.exec() == QDialog::Accepted) {
    int id = dialog.getIdInput();

    ROS_INFO_STREAM("Setting id: " << id);
    req.request.landmark.id = id;
    req.request.header.frame_id = context_->getFixedFrame().toStdString();
    client_.call(req);
  }
}
}  // namespace ruvu_mcl_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_mcl_rviz_plugins::AddLandmarkTool, rviz::Tool)
