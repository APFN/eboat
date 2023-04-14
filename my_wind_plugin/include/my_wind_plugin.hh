#ifndef MY_PLUGIN_HH
#define MY_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
    class MyPlugin : public ModelPlugin
  {
      public:
          void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
          void OnUpdate(const common::UpdateInfo & /*_info*/);
      private:
          physics::ModelPtr m_model;
          ros::Publisher m_wind_speed_pub;
          event::ConnectionPtr m_update_connection;
  };


  GZ_REGISTER_MODEL_PLUGIN(MyPlugin)
}

#endif
