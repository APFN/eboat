// Auhtor : Eduardo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//

#ifndef _SAIL_CONTROL_PLUGIN_HH
#define _SAIL_CONTROL_PLUGIN_HH

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "ignition/math/Vector3.hh"

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"

namespace gazebo
{
    class SailControl : public ModelPlugin
    {
        /// \brief Constructor.
        public: SailControl();

        /// Documentation Inherited.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// Documentation Inherited
        public: virtual void Init();

        /// \brief Callback for World Update events.
        protected: virtual void OnUpdate();

        /// \brief Connection to World Update events.
        protected: event::ConnectionPtr updateConnection;

        /// \brief Pointer to the World.
        protected: physics::WorldPtr world;

        /// \brief Pointer to physics engine.
        protected: physics::PhysicsEnginePtr physics;

        /// \brief Pointer to model containing plugin.
        protected: physics::ModelPtr model;

        /// \brief Name of model containing plugin.
        protected: std::string modelName;

        /// \brief Names of allowed target links, specified in sdf parameters.
        protected: std::string linkName;

        /// \brief Pointer to link currently targeted by mud joint.
        protected: physics::LinkPtr link;

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /// \brief Force implemented by the sail eletric engine on the CG
        ignition::math::Vector3d tension;

        /// \brief Wind force on CG
        ignition::math::Vector3d windForce;

        /// \brief Boom joint
        physics::JointPtr joint;
    };
}
#endif