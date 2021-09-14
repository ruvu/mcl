//
// Copyright (c) 2021 RUVU Robotics
//
// @author Rokus Ottervanger
//

#pragma once

#include <rviz/default_plugin/tools/pose_tool.h>

namespace rviz
{
class BoolProperty;
}

namespace ruvu_mcl_rviz_plugins
{
class AddLandmarkTool : public rviz::PoseTool
{
  Q_OBJECT
public:
  AddLandmarkTool();

  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void getPromptIdProperty();

private:
  ros::NodeHandle nh_;

  ros::ServiceClient client_;

  rviz::BoolProperty * id_prompt_property_;
  bool prompt_id_;
};
}  // namespace ruvu_mcl_rviz_plugins
