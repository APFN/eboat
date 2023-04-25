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
#include "../include/sailController.hh"

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"


#include <ros/param.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <mutex>
#include <cmath>



using namespace gazebo;


std::mutex mutex;

GZ_REGISTER_MODEL_PLUGIN(SailControllerPlugin)

//////////////////////////////////////////////////////////////////////
SailControllerPlugin::SailControllerPlugin() : sailPosition(0.0), boomEngVel(0.0005), flappyBoat(true)
{
}


void SailControllerPlugin::OnFlappyBoatMsg(const std_msgs::BoolConstPtr& _msg)
{
    //std::cout<<"msg->data chamada"<<std::endl;
    if (mutex.try_lock()) {
        //std::cout<<"msg->data dentro"<<std::endl;
        this->flappyBoat = _msg->data;
        mutex.unlock();
        //std::cout<<"msg->data fim"<<std::endl;
    } else {
        //std::cout<<"Tentou msg ocupado"<<std::endl;
    } 
}

//////////////////////////////////////////////////////////////////////
void SailControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "SailControllerPlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "SailControllerPlugin _sdf pointer is NULL");
    this->model     = _model;
    this->sdf       = _sdf;

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "SailControllerPlugin world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "SailControllerPlugin physics pointer is NULL");

    if (_sdf->HasElement("sail_joint_name"))
    {
        this->sailJoint = this->model->GetJoint(sdf->Get<std::string>("sail_joint_name"));
    }
    else
        this->sailJoint = this->model->GetJoint("boom_joint");

    if (_sdf->HasElement("boom_motor_speedy"))
        this->boomEngVel = _sdf->Get<double>("boom_motor_speedy");
    else
        this->boomEngVel = 0.5;

    if (_sdf->HasElement("flappy_boat")) {
        this->flappyBoat = _sdf->Get<bool>("flappy_boat");
    } else {
        this->flappyBoat = false;
    }
    
    //--> LINKS
    this->sailLink = this->sailJoint->GetChild();

    //--> CONSTANTS
    this->d2r = M_PI / 180.0;

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions boomAngleSub =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/eboat/control_interface/sail",
            1,
            boost::bind(&SailControllerPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(boomAngleSub);

    ros::SubscribeOptions flappyBoatSub =
        ros::SubscribeOptions::create<std_msgs::Bool>(
            "/eboat/control_interface/flappy_boat",
            1,
            boost::bind(&SailControllerPlugin::OnFlappyBoatMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->flappyBoatSub = this->rosNode->subscribe(flappyBoatSub);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&SailControllerPlugin::QueueThread, this));
}

/////////////////////////////////////////////////
void SailControllerPlugin::Init()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&SailControllerPlugin::OnUpdate, this));
    
    this->sailJoint->SetUpperLimit(0, 0.0); //(1.0*d2r));
    this->sailJoint->SetLowerLimit(0, 0.0); //-(1.0*d2r));
    this->sailPosition = this->d2r;
    SailControllerPlugin* self = this;
}

//////////////////////////////////////////////////////////////////////
void SailControllerPlugin::OnUpdate()
{
    if (this->flappyBoat) {
        if (mutex.try_lock()) {
            std::cout<<"flappyBoat: TRUE "<<std::endl;  
            this->sailJointThread = std::thread(&SailControllerPlugin::SailJointThreadFunction, this);                        
            this->sailJointThread.detach(); 
            //std::cout<<"Saiu thread "<<std::endl;            
        } else {
            //std::cout<<"tentou thread ocupado"<<std::endl;
        } 
    } else {
        if (mutex.try_lock()) {
            //std::cout<<"flappyBoat: FALSE "<<std::endl;
            ///////////////////////////////////////////////////////////////////////////////
            // Controller interface for emulate the real behavior of the boom.
            // In real world the boom is moved by a cable connected to an electric engine.
            // The sailor could not direct control the position of the sail/boom.
            // In order to change the sail position, the sailor can release cable, so the
            // wind can push the sail, or he can pull the cable to move the sail against
            // the wind force.
            double sov = this->boomEngVel * this->world->Physics()->GetMaxStepSize();
            if (this->sailPosition > 90.0)
                this->sailPosition == 90.0;
            else if (this->sailPosition < -90.0)
                this->sailPosition == -90.0;

            //std::cout<<"sailPosition: "<<this->sailPosition<<"this->sailJoint->UpperLimit(0): "<<this->sailJoint->UpperLimit(0)<<std::endl;

            if (this->sailPosition > this->sailJoint->UpperLimit(0) + this->d2r) //--> in this condition the cable should be released
            {
                //std::cout<<"flappyBoat: FALSE  - SOLTANDO CABO"<<std::endl;
                // emulates cable release velocity
                if ((this->sailJoint->UpperLimit(0) + sov) < this->sailPosition)
                {
                    this->sailJoint->SetUpperLimit(0,this->sailJoint->UpperLimit(0) + sov);
                    this->sailJoint->SetLowerLimit(0,this->sailJoint->LowerLimit(0) - sov);
                }
                else
                {
                    this->sailJoint->SetUpperLimit(0,this->sailPosition);
                    this->sailJoint->SetLowerLimit(0,-this->sailPosition);
                }
            }
            else if (this->sailPosition < this->sailJoint->UpperLimit(0) - this->d2r) //--> in this condition the cable should be pulled
            {
                //std::cout<<"flappyBoat: FALSE  - CAÇANDO CABO"<<std::endl;
                // emulates cable pull velocity
                if ((this->sailJoint->UpperLimit(0) - sov) > this->sailPosition)
                {
                    this->sailJoint->SetUpperLimit(0,this->sailJoint->UpperLimit(0) - sov);
                    this->sailJoint->SetLowerLimit(0,this->sailJoint->LowerLimit(0) + sov);
                }
                else
                {
                    this->sailJoint->SetUpperLimit(0,this->sailPosition);
                    this->sailJoint->SetLowerLimit(0,-this->sailPosition);
                }
            }
            mutex.unlock(); 
            //std::cout<<"FIm flappyBoat: false "<<std::endl;
        } else {
            //std::cout<<"flappyBoat: false - ocupado"<<std::endl;
        }
    }
}

void SailControllerPlugin:: SailJointThreadFunction(SailControllerPlugin* self)
{
    for (int angle = 0; angle <= 90; angle+=2) {
        //std::cout<<"Entrou for 1 "<<std::endl;
        float i = angle* M_PI / 180.0;
        self->sailJoint->SetUpperLimit(0,i);
        self->sailJoint->SetLowerLimit(0,i);        
        ros::Duration(0, 4000).sleep();;
        //std::cout<<"saiu for 1 "<<std::endl; 
    }
    //std::cout<<"Fim for 1 "<<std::endl; 
    for (int angle = 90; angle >= 0; angle-=15) {
        //std::cout<<"Entrou for 2 "<<std::endl;
        float i = angle* M_PI / 180.0;
        self->sailJoint->SetUpperLimit(0,i);
        self->sailJoint->SetLowerLimit(0,i);        
        ros::Duration(0, 400).sleep();
    }
    for (int angle = 0; angle >= -90; angle-=2) {
        //std::cout<<"Entrou for 2 "<<std::endl;
        float i = angle* M_PI / 180.0;
        self->sailJoint->SetUpperLimit(0,i);
        self->sailJoint->SetLowerLimit(0,i);        
        ros::Duration(0, 400).sleep();
    }
    for (int angle = -90; angle <= 0; angle+=+15) {
        //std::cout<<"Entrou for 1 "<<std::endl;
        float i = angle* M_PI / 180.0;
        self->sailJoint->SetUpperLimit(0,i);
        self->sailJoint->SetLowerLimit(0,i);        
        ros::Duration(0, 4000).sleep();;
        //std::cout<<"saiu for 1 "<<std::endl; 
    }
    std::cout<<"Fim flappyBoat "<<std::endl; 
    self->flappyBoat = false;
    mutex.unlock(); 
    return;            
}
