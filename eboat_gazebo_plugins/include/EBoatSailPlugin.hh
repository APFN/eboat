/*
    This plugin computes the forces acting on a sail in 
    the presence of wind. The principle of lift and drag
    is applied to compute the forces

    AUTHOR : Eduardo Charles Vasconcellos
    CONTACT: evasconcellos@id.uff.br

    10/2022
*/

#ifndef _EBOAT_GAZEBO_SAIL_PLUGIN_HH_
#define _EBOAT_GAZEBO_SAIL_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "ignition/math/Vector3.hh"

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

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /// \brief Boom link.
        protected: physics::LinkPtr boomLink;

        /// \brief Base link (Hull link).
        protected: physics::LinkPtr baseLink;

        /// \brief Joint name
        protected: physics::JointPtr joint;

        /// \brief Forward indicates the forward motion direction
        protected: ignition::math::Vector3d forward;

        /// \brief Upward indicates the upward direction (it points to the sky)
        protected: ignition::math::Vector3d upward;

        /// \brief Aparent wind velocity vector;
        protected: ignition::math::Vector3d aparentWindVelVec;

        /// \brief center of pressure in link local coordinates
        protected: ignition::math::Vector3d cp;

        /// \brief Area of the sail
        protected: double area;

        /// \brief fluid density
        protected: double rho;

        /// \brief Lift coeficient
        protected: double cl[181];

        /// \brief Drag coeficient
        protected: double cd[181];

        /// \brief Distance from the boom joint in with the wind force is applied (this distance is mesuare only in the boom direction)
        protected: double armLength;
    };
}
#endif