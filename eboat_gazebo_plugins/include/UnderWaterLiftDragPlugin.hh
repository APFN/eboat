/*
    This plugin computes the forces acting on a underwater 
    rudder or keel. The principle of lift and drag is 
    applied to compute the forces.

    AUTHOR : Eduardo Charles Vasconcellos
    CONTACT: evasconcellos@id.uff.br

    10/2022
*/

#ifndef _EBOAT_GAZEBO_UNDERWATER_LIFTDRAG_PLUGIN_HH_
#define _EBOAT_GAZEBO_UNDERWATER_LIFTDRAG_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "ignition/math/Vector3.hh"

namespace gazebo
{
    class UnderWaterLiftDragPlugin : public ModelPlugin
    {
        /// \brief Constructor
        public: UnderWaterLiftDragPlugin();

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

        /// \brief Rudder link.
        protected: physics::LinkPtr link;

        /// \brief Forward indicates the forward motion direction
        protected: ignition::math::Vector3d forward;

        /// \brief Upward indicates the upward direction (it points to the sky)
        protected: ignition::math::Vector3d upward;

        /// \brief Fluid velocity vector;
        protected: ignition::math::Vector3d fluidVelVec;

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
    };
}

#endif