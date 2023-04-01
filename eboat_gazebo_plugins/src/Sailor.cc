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
#include "../include/Sailor.hh"

/*
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"
*/

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(Sailor)

//////////////////////////////////////////////////////////////////////
Sailor::Sailor() : sailPosition(0.0), rudderPosition(0.0), boomEngVel(0.0005),
                   rudderEngVel(0.4), turbineVel(0.0), freq(1.0)
{
}

//////////////////////////////////////////////////////////////////////
void Sailor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "Sailor _model pointer is NULL");
    GZ_ASSERT(_sdf, "Sailor _sdf pointer is NULL");
    this->model = _model;
    this->modelName = _model->GetName();
    this->sdf = _sdf;

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "Sailor world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "Sailor physics pointer is NULL");

    if (_sdf->HasElement("sail_joint_name"))
    {
        this->sailJoint = this->model->GetJoint(_sdf->Get<std::string>("sail_joint_name"));
    }
    else
        this->sailJoint = this->model->GetJoint("boom_joint");

    if (_sdf->HasElement("rudder_joint_name"))
    {
        this->rudderJoint = this->model->GetJoint(_sdf->Get<std::string>("rudder_joint_name"));
    }
    else
        this->rudderJoint = this->model->GetJoint("rudder_joint");

    if (_sdf->HasElement("propulsion_joint_name"))
    {
        this->rudderJoint = this->model->GetJoint(_sdf->Get<std::string>("propulsion_joint_name"));
    }
    else
        this->propulsorJoint = this->model->GetJoint("turbine_joint");

    if (_sdf->HasElement("frequency"))
        this->freq = _sdf->Get<double>("frequency");

    if (_sdf->HasElement("boom_eng_speedy"))
        this->freq = _sdf->Get<double>("boomEngVel");

    if (_sdf->HasElement("rudder_eng_speedy"))
        this->freq = _sdf->Get<double>("rudderEngVel");

    // TO LINKS
    this->sailLink          = this->sailJoint->GetChild();
    this->rudderLink        = this->rudderJoint->GetChild();
    this->eletricEngineLink = this->propulsorJoint->GetChild();
    // TO EMULATE SENSORS
    this->boatLink          = this->model->GetLink("base_link");
}

/////////////////////////////////////////////////
void Sailor::Init()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&Sailor::OnUpdate, this));
    
    this->sailJoint->SetUpperLimit(0,this->d2r);
    this->sailJoint->SetLowerLimit(0,-this->d2r);
    this->rudderPosition = 0.0;
    this->sailPosition   = this->d2r;
    this->turbineVel     = 0;
}

//////////////////////////////////////////////////////////////////////
void Sailor::OnUpdate()
{
    float simtime = this->world->SimTime().Float();

    float Q = simtime * freq; //--> it is the same that this->world->SimTime().Float() / (1.0 / freq)
    Q -= int(Q);
    if ((Q >= 0.0) & (Q <= 0.00001))
    {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //                                                                                                                    //
        // -------------------------------------------------- SENSOR ARRAY -------------------------------------------------- //
        //                                                                                                                    //
        //--> BOAT POSITION
        //std::cout<<"Boat position in the world frame : "<<this->boatLink->WorldPose().Pos()<<std::endl;
        //--> BOAT ANGULAR DOFs
        this->roll  = this->boatLink->WorldPose().Rot().Roll();
        this->pitch = -this->boatLink->WorldPose().Rot().Pitch();
        this->yaw   = -this->boatLink->WorldPose().Rot().Yaw();
        //std::cout<<"Boat angular DOFs                : "<<this->roll<<" "<<this->pitch<<" "<<this->yaw<<std::endl;
        //--> BOAT VELOCITY AND DIRECTION
        this->boatVelocity = this->boatLink->WorldLinearVel().Length();
        this->compass      = -this->boatLink->WorldPose().Rot().Yaw() * 180.0 / M_PI;//atan2(this->boatLink->WorldLinearVel().Y(), this->boatLink->WorldLinearVel().X()) * 180.0 / M_PI;
        /*std::cout<<"Boat velocity vector             : "<<this->boatLink->WorldLinearVel()<<std::endl;
        std::cout<<"Boat velocity                    : "<<this->boatVelocity<<std::endl;
        std::cout<<"Compass                          : "<<this->compass<<" deg from North"<<std::endl;
        */
        //--> APARENT WIND VELOCITY AND DIRECTION
        // The aparent wind is cased by the boat motion.
        // aparent wind = real wind + boat wind
        //         in other words
        // aparent wind = real wind - boat velocity
        this->boatVelocityVector = this->sailLink->WorldLinearVel(); //-> as the boat is a rigd body, the boat velocity is represented by the boom link velocity in the world frame
        this->windVelocityVector = this->sailLink->WorldWindLinearVel() - this->boatVelocityVector;
        this->windVelocity       = windVelocityVector.Length();
        this->windVelocityVector = this->boatLink->WorldPose().Rot().RotateVector(this->windVelocityVector);
        this->windDirection      = atan2(this->windVelocityVector.Y(), this->windVelocityVector.X())*180.0/M_PI;
        //std::cout<<"Aparent wind velocity            : "<<this->windVelocity<<std::endl;
        //std::cout<<"Aparent wind direction           : "<<this->windDirection<<std::endl;
        //                                                                                                                    //
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        

        ///////////////////////////////////////////////////////////////////////////////
        // Sailor policy
        // The input commands are: sailPosition (0.0 to 90.0 deg) desired angular position for the sail; it depends on wind force.
        //                         boomEngVel   (0.0 to 1.0  deg) emulates the influence of release/pull cable velocity on changing the sail angular position.
        //                         turbineVel   (-5, -4,..., 0, 1, 2,..., 5) emulates the eletric propulsion
        ///////////////////////////////////////////////////////////////////////////////
    }
    ///////////////////////////////////////////////////////////////////////////////
    // Controller interface for emulate the real behavior of the boom.
    // In real world the boom is moved by a cable connected to an electric engine.
    // The sailor could not direct control the position of the sail/boom.
    // In order to change the sail position, the sailor can release cable, so the
    // wind can push the sail, or he can pull the cable to move the sail against
    // the wind force.
    this->boomEngVel = 0.5 / 1000.0; //-->the time step in current simulation is 1 millisecond
    if (this->sailPosition > this->sailJoint->UpperLimit(0) + (M_PI/180.0)) //--> in this condition the cable should be released
    {
        // emulates cable release velocity
        if ((this->sailJoint->UpperLimit(0) + boomEngVel) < this->sailPosition)
        {
            this->sailJoint->SetUpperLimit(0,this->sailJoint->UpperLimit(0) + boomEngVel);
            this->sailJoint->SetLowerLimit(0,this->sailJoint->LowerLimit(0) - boomEngVel);
        }
        else
        {
            this->sailJoint->SetUpperLimit(0,this->sailPosition);
            this->sailJoint->SetLowerLimit(0,-this->sailPosition);
        }
    }
    else if (this->sailPosition < this->sailJoint->UpperLimit(0) - (M_PI/180.0)) //--> in this condition the cable should be pulled
    {
        // emulates cable pull velocity
        if ((this->sailJoint->UpperLimit(0) - boomEngVel) > this->sailPosition)
        {
            this->sailJoint->SetUpperLimit(0,this->sailJoint->UpperLimit(0) - boomEngVel);
            this->sailJoint->SetLowerLimit(0,this->sailJoint->LowerLimit(0) + boomEngVel);
        }
        else
        {
            this->sailJoint->SetUpperLimit(0,this->sailPosition);
            this->sailJoint->SetLowerLimit(0,-this->sailPosition);
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    // Control interface for the rudder
    this->rudderEngVel = 0.4;          //-->this velocity is already in millisecond scale.
    if ((this->rudderJoint->Position() < this->rudderPosition + 1.0e-4) &&
        (this->rudderJoint->Position() > this->rudderPosition - 1.0e-4))
    {
        this->rudderJoint->SetVelocity(0, 0.0);
    }
    else if (this->rudderPosition != this->rudderJoint->Position())
    {
        if (this->rudderPosition < this->rudderJoint->Position())
        {
            this->rudderJoint->SetVelocity(0, -this->rudderEngVel);
        }
        else
        {
            this->rudderJoint->SetVelocity(0, this->rudderEngVel);
        }
    }
    /*std::cout<<"rudderJoint velocity   : "<<this->rudderJoint->GetVelocity(0)<<std::endl;
    std::cout<<"Rudder Link Pose       : "<<this->rudderLink->WorldPose()<<std::endl;
    std::cout<<"Rudder target position : "<<this->rudderPosition<<std::endl;
    std::cout<<"Rudder actual position : "<<this->rudderJoint->Position()<<std::endl;
    /*std::cout<<"Sail Link Pose : "<<this->sailLink->WorldPose()<<std::endl;*/
    ///////////////////////////////////////////////////////////////////////////////
    // Eletric propulsion control interface and model
    // Force applied as function of turbine angular speed
    // Our max force are 24.49 kgf
    double turbineAngVel = this->turbineVel * (propulsorJoint->GetVelocityLimit(0)/5.0);
    //std::cout<<"TurbinVel: "<<turbineAngVel<<std::endl;
    if (turbineAngVel != this->propulsorJoint->GetVelocity(0))
    {
        this->propulsorJoint->SetVelocity(0, turbineAngVel);
        //double force = turbineVel * 24.49 * 9.80665; //--> force in newtons
        double force = this->turbineVel * 24.49;       //--> force in kgf
        ignition::math::Vector3d thrust = ignition::math::Vector3d(force,0,0);
        this->eletricEngineLink->AddLinkForce(thrust, ignition::math::Vector3d(0,0,0));
        //std::cout<<"Thurst         : "<<thrust<<std::endl;
    }
    /*std::cout<<"SimTime                    : "<<this->world->SimTime().Float()<<std::endl;
    std::cout<<"---------------------------------------------------"<<std::endl;
    */
    /*if (simtime == 6.0)
    {
        std::cout<<" ----------> "<<this->sailJoint->Position(0)<<std::endl;
        this->world->SetPaused(true);
    }*/
}