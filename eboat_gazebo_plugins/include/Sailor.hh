// Auhtor : Eduardo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//

#ifndef _SAILOR_PLUGIN_HH
#define _SAILOR_PLUGIN_HH

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "ignition/math/Vector3.hh"

/*
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"
*/

namespace gazebo
{
    class Sailor : public ModelPlugin
    {
        /// \brief Constructor.
        public: Sailor();

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

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /// \brief Pointer to Sail joint
        protected: physics::JointPtr sailJoint;

        /// \brief Pointer to Rudder joint
        protected: physics::JointPtr rudderJoint;

        /// \brief Pointer to Propulsor joint
        protected: physics::JointPtr propulsorJoint;

        /// \brief Pointer to Sail link
        protected: physics::LinkPtr sailLink;

        /// \brief Pointer to Rudder link
        protected: physics::LinkPtr rudderLink;

        /// \brief Pointer to Eletric Engine link
        protected: physics::LinkPtr eletricEngineLink;

        /// \brief Aparent wind vectorial velocity
        protected: ignition::math::Vector3d windVelocityVector;

        /// \brief Aparent wind velocity on sail link origin
        protected: double windVelocity;

        /// \brief Aparent wind direction in rad
        protected: double windDirection;

        /// \brief Vectorial velocity of sail link origin (aproximates the boat velocity)
        protected: ignition::math::Vector3d boatVelocityVector;

        /// \brief Sail angular position. Goes from 0 to 90 deg. The side (port or starbord) depends on the aparent wind direction.
        protected: double sailPosition;

        /// \brief Rudder angular position. Goes from -60 to 60 deg (-1.0472 to 1.0472 rad).
        protected: double rudderPosition;

        /// \brief Boom eletric engine pull/release velocity
        protected: double boomEngVel;

        /// \brief Rudder eletric engine turn velocity
        protected: double rudderEngVel;

        /// \brief Eletric propulsion turbine velocity (the current real propulsor has 11 velocities, so turbineVel can assume discrete values [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5])
        protected: double turbineVel;

        /// \brief Store boat position in world frame
        protected: physics::LinkPtr boatLink;

        /// \brief Boat angular DOFs
        protected: double roll;
        protected: double pitch;
        protected: double yaw;

        /// \brief Boat Velocity
        protected: double boatVelocity;

        /// \brief Compass (Boat direction)
        protected: double compass;

        /// \brief Agent interaction frequency (in Hz)
        protected: float freq;

        /// \brief Factor to tranform degree to rad
        private: const double d2r = M_PI / 180.0;
    };
}
#endif