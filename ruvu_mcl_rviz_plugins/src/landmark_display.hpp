//
// Copyright (c) 2021 RUVU Robotics
//
// @author Rokus Ottervanger
//

#pragma once

#include <ruvu_mcl_msgs/LandmarkList.h>
#include <rviz/message_filter_display.h>

#include <memory>
#include <vector>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class Axes;
}  // namespace rviz

namespace ruvu_mcl_rviz_plugins
{
class LandmarkDisplay : public rviz::MessageFilterDisplay<ruvu_mcl_msgs::LandmarkList>
{
  Q_OBJECT
public:
  LandmarkDisplay();
  virtual ~LandmarkDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();

private Q_SLOTS:
  void updateAxisGeometry();

private:
  bool setTransform(std_msgs::Header const & header);
  void processMessage(const ruvu_mcl_msgs::LandmarkList::ConstPtr & msg);

  std::vector<std::shared_ptr<rviz::Axes>> visuals_;

  rviz::FloatProperty * axes_length_property_;
  rviz::FloatProperty * axes_radius_property_;
};

}  // namespace ruvu_mcl_rviz_plugins
