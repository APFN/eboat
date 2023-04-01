// Auhtor : Eduardo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//

#include <algorithm>
#include <string>

#include "ignition/math/Pose3.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "ros/ros.h"
#include "../include/sail_control.hh"

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SailControl)

//////////////////////////////////////////////////////////////////////
SailControl::SailControl() : linkName("boom_link")
{
}

//////////////////////////////////////////////////////////////////////
void SailControl::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "SailControl _model pointer is NULL");
    GZ_ASSERT(_sdf, "SailControl _sdf pointer is NULL");
    this->model = _model;
    this->modelName = _model->GetName();
    this->sdf = _sdf;

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "SailControl world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "SailControl physics pointer is NULL");

    GZ_ASSERT(_sdf, "SailControl _sdf pointer is NULL");

    if (_sdf->HasElement("link_name"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("link_name");
        this->linkName = elem->Get<std::string>();
    }
    this->link = this->model->GetLink(this->linkName);

    physics::Joint_V jointv = this->link->GetParentJoints();
    joint = jointv[0];
}

/////////////////////////////////////////////////
void SailControl::Init()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&SailControl::OnUpdate, this));
    
    joint->SetUpperLimit(0,0.0);
    joint->SetLowerLimit(0,0.0);
}

//////////////////////////////////////////////////////////////////////
void SailControl::OnUpdate()
{
    // get the force acting on the boom CG.
    ignition::math::Pose3d pose = this->link->WorldPose();
    std::cout<<"Link pose on World frame: "<<pose<<std::endl;
    //
    std::cout<<joint->GetName()<<std::endl;
    std::cout<<"Position       : "<<joint->Position()<<std::endl;
    std::cout<<"GetForce       : "<<joint->GetForce(0)<<std::endl;
    //std::cout<<"LinkForce      : "<<joint->LinkForce(0)<<std::endl;
    //std::cout<<"GetForce Torque: "<<joint->GetForceTorque(0).body2Force<<std::endl;

    /////////////////////////////////////////////////////////////////////
    // Emulates the action of pull the cable
    if (pose.X() > 10)
    {
        double engvel = 0.2;
        double pos    = joint->Position();
        engvel *= (-pos) / fabs(pos);
        joint->SetVelocity(0, engvel);
        std::cout<<"Velocidade do motor: "<<engvel<<std::endl;
    }
    ignition::math::Vector3d windvel = this->link->WorldWindLinearVel();
    if (windvel.Length() > 1.0)
    {
        joint->SetUpperLimit(0,0.4);
        joint->SetLowerLimit(0,-0.4);
    }
}