// Auhtor : Eduardo Charles Vasconcellos
// Contact: evasconcellos@id.uff.br
//

#include <algorithm>

#include "ignition/math/Pose3.hh"
#include "ignition/math/Vector3.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include "../include/missionControl.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MissionControlPlugin)

//////////////////////////////////////////////////////////////////////
MissionControlPlugin::MissionControlPlugin(): freq(1.0)
{
    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    /*this->distPub      = rosNode.advertise<std_msgs::Float32>("/eboat/mission_control/distance", 500);
    this->sailPub      = rosNode.advertise<std_msgs::Float32>("/eboat/mission_control/sail_angle", 500);
    this->rudderPub    = rosNode.advertise<std_msgs::Float32>("/eboat/mission_control/rudder_angle", 500);
    this->propulsorPub = rosNode.advertise<std_msgs::Float32>("/eboat/mission_control/turbine_speed", 500);
    this->windSpeedPub = rosNode.advertise<std_msgs::Float32>("/eboat/mission_control/wind_speed", 500);
    this->windAnglePub = rosNode.advertise<std_msgs::Float32>("/eboat/mission_control/wind_angle", 500);*/
    this->obsPub = rosNode.advertise<std_msgs::Float32MultiArray>("/eboat/mission_control/observations", 1000);
    this->windPub = rosNode.advertise<std_msgs::Float32MultiArray>("/eboat/atmosferic_control/wind", 1000);
}

void MissionControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "Sailor _model pointer is NULL");
    GZ_ASSERT(_sdf, "Sailor _sdf pointer is NULL");
    this->model = _model;

    this->world = _model->GetWorld();
    GZ_ASSERT(this->world, "Sailor world pointer is NULL");

    if (_sdf->HasElement("link_name"))
    {
        this->link = _model->GetLink(_sdf->Get<std::string>("link_name"));
    }
    else
        this->link = _model->GetLink("base_link");

    if (_sdf->HasElement("frequency"))
        this->freq = _sdf->Get<int>("frequency");
    
    if (_sdf->HasElement("way_point"))
        this->wayPointModelName = _sdf->Get<std::string>("way_point");
    else
        this->wayPointModelName = "wayPointMarker";

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

    // Forward direction in World frame
    if (_sdf->HasElement("ahead"))
        this->ahead = _sdf->Get<ignition::math::Vector3d>("ahead");
    else
        this->ahead = ignition::math::Vector3d(1,0,0);

    // Port direction in World frame
    if (_sdf->HasElement("port"))
        this->port = _sdf->Get<ignition::math::Vector3d>("ahead");
    else
        this->port = ignition::math::Vector3d(0,1,0);

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
}

/////////////////////////////////////////////////
void MissionControlPlugin::Init()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MissionControlPlugin::OnUpdate, this));

    this->speedFactor = 5.0 / this->propulsorJoint->GetVelocityLimit(0);
}

//////////////////////////////////////////////////////////////////////
void MissionControlPlugin::OnUpdate()
{
    float simtime = this->world->SimTime().Float();
    
    float Q = simtime / this->freq;
    Q -= int(Q);
    if ((Q >= 0.0) & (Q <= 0.00001))
    {
        float eboat_x, eboat_y, waypoint_x, waypoint_y;

        this->timeOfLastObs = simtime;

        std_msgs::Float32MultiArray obsMsg;
        std_msgs::Float32MultiArray windMsg;
        ignition::math::Vector3d apWind        = this->sailJoint->GetChild()->WorldWindLinearVel() - this->sailJoint->GetChild()->WorldLinearVel();
        ignition::math::Vector3d aheadD        = this->link->WorldPose().Rot().RotateVector(this->ahead); //--> Dynamic ahead direction, it changes as the boats move around.
        ignition::math::Vector3d portD         = this->link->WorldPose().Rot().RotateVector(this->port);  //--> Dynamic port direction, it changes as the boats move around.
        ignition::math::Vector3d trajectoryVec;

        ignition::math::Vector3d wind_vel = this->sailJoint->GetChild()->WorldWindLinearVel();
        windMsg.data.push_back(wind_vel.X());
        windMsg.data.push_back(wind_vel.Y());
        this->windPub.publish(windMsg);

        // COMPUTE THE DISTANCE TO THE WAY POINT AND TAJECTORY TO THE WAYPOINT
        float distance;
        if (this->world->ModelByName(this->wayPointModelName) == nullptr)
        {
            //std::cout << "WAY POINT DO NOT EXIST!" << std::endl;
            distance      = 0.0;
            trajectoryVec = aheadD;
        }
        else
        {
            this->wayPoint = this->world->ModelByName(this->wayPointModelName);
            distance = this->model->WorldPose().Pos().Distance(this->wayPoint->WorldPose().Pos());
            //std::cout << "mis.control distance: " << distance << std::endl;
            trajectoryVec = (this->wayPoint->WorldPose().Pos() - this->model->WorldPose().Pos()) / distance;

            waypoint_x = this->wayPoint->WorldPose().Pos().X();
            waypoint_y = this->wayPoint->WorldPose().Pos().Y();
        }
        
        // INSERT DISTANCE INTO ROS MESAGE
        obsMsg.data.push_back(distance);
        
        // COMPUTE THE ANGLE BETWEEN THE BOAT AHEAD (FORWARD DIRECTION) AND THE DIRECTION OF THE LINE CONNECTING THE BOAT AND THE WAYPOINT
        float trajectoryAng = aheadD.Dot(trajectoryVec);
        if (trajectoryAng > 1.0)
            trajectoryAng = this->r2d;
        else
            trajectoryAng = acos(trajectoryAng) * this->r2d;
        
        if (acos(trajectoryVec.Dot(portD))*this->r2d > 90.0)
            trajectoryAng *= -1;
        obsMsg.data.push_back(trajectoryAng);

        // COMPUTE THE LINEAR VELOCITY OF THE BOAT
        float linearVel = aheadD.Dot(this->model->WorldLinearVel());
        obsMsg.data.push_back(linearVel);

        // COMPUTE WIND ANGLE AND SPEED
        float windSpeed = apWind.Length();
        /* The wind Angle is measured between the forward direction (ahead) and the aparent wind direction.
         * It is positive when the aparent wind direction points to port and negative otherwise.
        */
        float windAngle = windSpeed < 0.1 ? 0.0
                                          : acos(apWind.Dot(aheadD) / windSpeed)*this->r2d;
        if (acos(apWind.Dot(portD) / windSpeed)*this->r2d > 90.0)
            windAngle *= -1;
        obsMsg.data.push_back(windSpeed);
        obsMsg.data.push_back(windAngle);

        // GET THE BOOM ANGLE (SAIL ANGLE);
        obsMsg.data.push_back(this->sailJoint->UpperLimit()*this->r2d);

        // GET RUDDER ANGLE;
        obsMsg.data.push_back(this->rudderJoint->Position(0)*this->r2d);

        //GET TURBINE SPEED;
        obsMsg.data.push_back(this->propulsorJoint->GetVelocity(0) * this->speedFactor); //--> in the real propulsor there are 5 speeds forward and 5 backward. speedFactor helps to convert the joint angular velocity to the discrete propulsor speed.

        //GET ROLL ANGLE
        obsMsg.data.push_back(this->model->WorldPose().Rot().Roll()*this->r2d);

        // PUBLISH OBSERVATIONS
        this->obsPub.publish(obsMsg);
        
        //physics::ODEPhysicsPtr ode;
        //physics::PhysicsEnginePtr physics;

        /*std::cout<<"------------------------------------------------------------------"<<std::endl;
        std::cout << this->model->GetName() << std::endl;
        std::cout << simtime << std::endl;
        std::cout << "this->rudderJoint->Position(0) = " << this->rudderJoint->Position(0) << std::endl;
        std::cout << "                                 " << this->rudderJoint->Position(0)*this->r2d << std::endl;
        /*std::cout << linearVel << std::endl;
        std::cout << distance << std::endl;
        std::cout << "trajectoryVec                   = " << trajectoryVec << "(" << aheadD << ")" << std::endl;
        std::cout << "aheadD.Dot(trajectoryVec)       = " << aheadD.Dot(trajectoryVec) << std::endl;
        std::cout << "acos(aheadD.Dot(trajectoryVec)) = " << acos(floor(aheadD.Dot(trajectoryVec))) << std::endl;
        std::cout << trajectoryAng << std::endl;
        /*std::cout << "trajectoryAng = " << trajectoryAng << std::endl;
        /*std::cout<<this->model->WorldPose().Rot().Roll()*this->r2d<<std::endl;
        
        std::cout<<simtime<<"("<<this->world->Physics()->GetMaxStepSize()<<")"<<" Observations  : "<<obsMsg.data[0]<<std::endl;
        std::cout<<this->world->Physics()->GetRealTimeUpdateRate()<<std::endl;
        std::cout<<simtime/this->world->Physics()->GetMaxStepSize()<<" / "<<this->world->Iterations()<<std::endl;
        //std::cout<<<<std::endl;
        //if (int(simtime) == 8)
        //     this->world->Physics()->SetMaxStepSize(0.01);
        */


        // eboat_x = this->model->WorldPose().Pos().X();
        // eboat_y = this->model->WorldPose().Pos().Y();


        // std::cout << "Eboot coordinates: x: " << eboat_x << " ; " << eboat_y << std::endl;
        // std::cout << "Waypoint coordinates: " << waypoint_x << " ; " << waypoint_y << std::endl;

    }
}