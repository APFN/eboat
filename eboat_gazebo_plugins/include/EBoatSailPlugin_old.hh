// This code was adapted from the original LiftDrag plugin from Gazebo
//
// Auhtor : Eduardo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//
// The original code (2014) is under Apache 2.0 license
//
//    http://www.apache.org/licenses/LICENSE-2.0
//

#ifndef _EBOAT_GAZEBO_SAIL_PLUGIN_HH_
#define _EBOAT_GAZEBO_SAIL_PLUGIN_HH_

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
    class SailPlugin : public ModelPlugin
    {
        /// \brief Constructor.
        public: SailPlugin();

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

        /// \brief Coeficient of Lift / alpha slope.
        /// Lift = C_L * q * S
        /// where q (dynamic pressure) = 0.5 * rho * v^2
        protected: double cla;

        /// \brief Coeficient of Drag / alpha slope.
        /// Drag = C_D * q * S
        /// where q (dynamic pressure) = 0.5 * rho * v^2
        protected: double cda;

        /// \brief Coeficient of Moment / alpha slope.
        /// Moment = C_M * q * S
        /// where q (dynamic pressure) = 0.5 * rho * v^2
        protected: double cma;

        /// \brief angle of attach when airfoil stalls
        protected: double alphaStall;

        /// \brief Cl-alpha rate after stall
        protected: double claStall;

        /// \brief Cd-alpha rate after stall
        protected: double cdaStall;

        /// \brief Cm-alpha rate after stall
        protected: double cmaStall;

        /// \brief: \TODO: make a stall velocity curve
        protected: double velocityStall;

        /// \brief air density
        /// at 25 deg it's about 1.1839 kg/m^3
        /// at 20 deg and 101.325 kPa, dry air has a density of 1.2041 kg/m^3
        protected: double rho;

        /// \brief effective planeform surface area.
        protected: double area;

        /// \brief angle of sweep.
        protected: double sweep;

        /// \brief initial angel of attack
        protected: double alpha0;

        /// \brief angle of attack
        protected: double alpha;

        /// \brief center of pressure in link local coordinates
        protected: ignition::math::Vector3d cp;

        /// \brief forward sailing direction in link local coordinates.
        protected: ignition::math::Vector3d forward;

        /// \brief A vector in the lift/drag plane, anything orthogonal to it
        /// is considered sail sweep.
        protected: ignition::math::Vector3d upward;

        /// \brief Smooth velocity.
        protected: ignition::math::Vector3d velSmooth;

        /// \brief Names of allowed target links, specified in sdf parameters.
        protected: std::string linkName;

        /// \brief Pointer to link currently targeted by mud joint.
        protected: physics::LinkPtr link;

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /////////////////////////////////////////////////////////////////////////
        // New Code from Eduardo
        /////////////////////////////////////////////////////////////////////////
        protected: physics::LinkPtr hlink;
        
        /// \brief World Wind Vector (vento real)
        protected: ignition::math::Vector3d wwindVel;

        /// \brief Aparent Wind Vector (vento aparente)
        protected: ignition::math::Vector3d awindVel;
    };
}
#endif