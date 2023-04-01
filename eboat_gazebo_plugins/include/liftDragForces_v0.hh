/*
    This plugin computes the forces acting on a sail in 
    the presence of wind. The principle of lift and drag
    is applied to compute the forces

    AUTHOR : Eduardo Charles Vasconcellos
    CONTACT: evasconcellos@id.uff.br

    10/2022
*/

#ifndef _EBOAT_GAZEBO_LIFTDRAGFORCES_PLUGIN_HH_
#define _EBOAT_GAZEBO_LIFTDRAGFORCES_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "ignition/math/Vector3.hh"

namespace gazebo
{
    class LiftDragForces : public ModelPlugin
    {
        /// \brief Constructor.
        public: LiftDragForces();

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

        // \brief Pointer to physics engine.
        //protected: physics::PhysicsEnginePtr physics;

        /// \brief Pointer to model containing plugin.
        protected: physics::ModelPtr model;

        // \brief Name of model containing plugin.
        //protected: std::string modelName;

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /// \brief Boom link.
        protected: physics::LinkPtr boomLink;

        /// \brief Boom link.
        protected: physics::LinkPtr rudderLink;

        /// \brief Boom link.
        protected: physics::LinkPtr keelLink;

        /// \brief Base link (Hull link).
        protected: physics::LinkPtr baseLink;

        // \brief Joint name
        //protected: physics::JointPtr joint;

        /// \brief Forward indicates the forward motion direction
        protected: ignition::math::Vector3d sailForward;

        /// \brief Forward indicates the forward motion direction
        protected: ignition::math::Vector3d sailUpward;

        /// \brief Forward indicates the forward motion direction
        protected: ignition::math::Vector3d rudderForward;

        /// \brief Forward indicates the forward motion direction
        protected: ignition::math::Vector3d rudderUpward;

        /// \brief Forward indicates the forward motion direction
        protected: ignition::math::Vector3d keelForward;

        /// \brief Forward indicates the forward motion direction
        protected: ignition::math::Vector3d keelUpward;

        /// \brief Indicates the perpendicular direction for the sail chord line
        protected: ignition::math::Vector3d transversal;

        /// \brief Aparent wind velocity vector;
        protected: ignition::math::Vector3d aparentWindVelVec;

        /// \brief Aparent wind velocity vector;
        protected: ignition::math::Vector3d waterVelVec;

        /// \brief Aparent wind velocity vector;
        protected: ignition::math::Vector3d currentFlowVelVec;

        /// \brief Sail forces center of pressure in link local coordinates
        protected: ignition::math::Vector3d sail_cp;

        /// \brief Keel forces center of pressure in link local coordinates
        protected: ignition::math::Vector3d keel_cp;

        /// \brief Area of the sail
        protected: double sailArea;

        /// \brief Area of the sail
        protected: double rudderArea;

        /// \brief Area of the sail
        protected: double keelArea;

        /// \brief air density
        protected: double airRHO;

        /// \brief water density
        protected: double waterRHO;

        /// \brief Lift coeficient
        protected: double sailCL[181];

        /// \brief Drag coeficient
        protected: double sailCD[181];

        /// \brief Lift coeficient
        protected: double rudderCL[181];

        /// \brief Drag coeficient
        protected: double rudderCD[181];

        /// \brief Distance from the boom joint in with the wind force is applied (this distance is mesuare only in the boom direction)
        protected: double armLength;

        private: const double r2d = 180.0 / M_PI;

        private: ignition::math::Vector3d chordLine;
        private: ignition::math::Vector3d upwardW;
        private: ignition::math::Vector3d dragDirection;
        private: ignition::math::Vector3d liftDirection;
    };
}
#endif