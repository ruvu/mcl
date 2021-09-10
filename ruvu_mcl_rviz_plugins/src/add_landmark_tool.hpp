//
// Copyright (c) 2021 RUVU Robotics
//
// @author Rokus Ottervanger
//

#pragma once

#include <rviz/default_plugin/tools/pose_tool.h>

namespace rviz
{
class StringProperty;
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

private:
  ros::NodeHandle nh_;

  ros::ServiceClient client_;
};
}  // namespace ruvu_mcl_rviz_plugins
