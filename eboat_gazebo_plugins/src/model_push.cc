#include <functional>
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "ignition/math/Vector3.hh"
#include "ros/ros.h"

namespace gazebo
{
    class ModelPush : public ModelPlugin
    {
        /// \brief Name of model containing plugin.
        protected: std::string modelName;
        
        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /// \brief Names of allowed target links, specified in sdf parameters.
        protected: std::string linkName;

        /// \brief Pointer to link currently targeted by mud joint.
        protected: physics::LinkPtr link;
        
        public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            GZ_ASSERT(_model, "SailPlugin _model pointer is NULL");
            GZ_ASSERT(_sdf, "SailPlugin _sdf pointer is NULL");
            this->model = _model;
            this->modelName = _model->GetName();
            this->sdf = _sdf;

            if (_sdf->HasElement("link_name"))
            {
                sdf::ElementPtr elem = _sdf->GetElement("link_name");
                this->linkName = elem->Get<std::string>();
                this->link = this->model->GetLink(this->linkName);
            }

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ModelPush::OnUpdate, this));
            ROS_WARN("Loaded ModelPush Plugin with model...%s", this->model->GetName().c_str());
            ROS_WARN("Loaded ModelPush Plugin with link...%s", this->linkName.c_str());
        }

        // Called by the world update start event
        public: void OnUpdate()
        {
            // Apply a small linear velocity to the model.
            this->link->SetLinearVel(ignition::math::Vector3d(0, 0.3, 0));
        }

        // Pointer to the model
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}