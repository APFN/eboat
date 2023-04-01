/*
    This plugin computes the forces acting on a underwater 
    rudder or keel. The principle of lift and drag is 
    applied to compute the forces.

    AUTHOR : Eduardo Charles Vasconcellos
    CONTACT: evasconcellos@id.uff.br

    10/2022
*/

#include <algorithm>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "ignition/math/Pose3.hh"
#include "../include/UnderWaterLiftDragPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(UnderWaterLiftDragPlugin)

//////////////////////////////////////////////////////////////////////
UnderWaterLiftDragPlugin::UnderWaterLiftDragPlugin()
{
  //->lift and drag coeficients
  double xl[] = {0.0000, 0.0649, 0.1129, 0.1473, 0.1711, 0.1877, 0.2002, 0.2118, 0.2255, 0.2434, 0.2673, 0.2980, 0.3319, 0.3642, 0.3941, 0.4363, 0.5023, 0.5736, 0.6301, 0.6738, 0.7125, 0.7521, 0.7914, 0.8274, 0.8593, 0.8948, 0.9357, 0.9527, 0.9291, 0.9293, 1.0384, 1.2652, 1.3859, 1.3855, 1.3202, 1.2461, 1.2079, 1.2040, 1.2209, 1.2456, 1.2647, 1.2681, 1.2578, 1.2393, 1.2175, 1.1979, 1.1843, 1.1754, 1.1687, 1.1615, 1.1514, 1.1363, 1.1170, 1.0945, 1.0703, 1.0456, 1.0213, 0.9974, 0.9736, 0.9495, 0.9248, 0.8992, 0.8729, 0.8459, 0.8186, 0.7910, 0.7633, 0.7355, 0.7074, 0.6788, 0.6496, 0.6197, 0.5892, 0.5582, 0.5268, 0.4951, 0.4632, 0.4311, 0.3989, 0.3664, 0.3338, 0.3010, 0.2679, 0.2347, 0.2014, 0.1680, 0.1345, 0.1009, 0.0673, 0.0336, 0.0000, -0.0336, -0.0673, -0.1009, -0.1345, -0.1680, -0.2014, -0.2347, -0.2679, -0.3010, -0.3338, -0.3664, -0.3989, -0.4311, -0.4632, -0.4951, -0.5268, -0.5582, -0.5892, -0.6197, -0.6496, -0.6788, -0.7074, -0.7355, -0.7633, -0.7910, -0.8186, -0.8459, -0.8729, -0.8992, -0.9248, -0.9495, -0.9736, -0.9974, -1.0213, -1.0456, -1.0703, -1.0945, -1.1170, -1.1363, -1.1514, -1.1615, -1.1687, -1.1754, -1.1843, -1.1979, -1.2175, -1.2393, -1.2578, -1.2681, -1.2647, -1.2456, -1.2209, -1.2040, -1.2079, -1.2461, -1.3202, -1.3855, -1.3859, -1.2652, -1.0384, -0.9293, -0.9291, -0.9527, -0.9357, -0.8948, -0.8593, -0.8274, -0.7914, -0.7521, -0.7125, -0.6738, -0.6301, -0.5736, -0.5023, -0.4363, -0.3941, -0.3642, -0.3319, -0.2980, -0.2673, -0.2434, -0.2255, -0.2118, -0.2002, -0.1877, -0.1711, -0.1473, -0.1129, -0.0649, 0.0000};
  double xd[] = {0.0043, 0.0174, 0.0280, 0.0367, 0.0445, 0.0520, 0.0600, 0.0694, 0.0806, 0.0932, 0.1064, 0.1196, 0.1323, 0.1439, 0.1549, 0.1695, 0.1907, 0.2130, 0.2307, 0.2451, 0.2594, 0.2756, 0.2917, 0.3046, 0.3144, 0.3332, 0.3631, 0.3533, 0.2751, 0.2400, 0.3943, 0.7602, 0.9763, 1.0118, 0.9496, 0.8725, 0.8466, 0.8706, 0.9264, 0.9960, 1.0612, 1.1080, 1.1388, 1.1598, 1.1773, 1.1979, 1.2262, 1.2608, 1.2987, 1.3368, 1.3722, 1.4026, 1.4285, 1.4514, 1.4725, 1.4933, 1.5148, 1.5368, 1.5589, 1.5807, 1.6018, 1.6219, 1.6410, 1.6596, 1.6780, 1.6963, 1.7148, 1.7333, 1.7513, 1.7686, 1.7847, 1.7994, 1.8129, 1.8253, 1.8368, 1.8476, 1.8579, 1.8677, 1.8768, 1.8853, 1.8931, 1.9001, 1.9064, 1.9119, 1.9166, 1.9206, 1.9239, 1.9264, 1.9282, 1.9294, 1.9298, 1.9294, 1.9282, 1.9264, 1.9239, 1.9206, 1.9166, 1.9119, 1.9064, 1.9001, 1.8931, 1.8853, 1.8768, 1.8677, 1.8579, 1.8476, 1.8368, 1.8253, 1.8129, 1.7994, 1.7847, 1.7686, 1.7513, 1.7333, 1.7148, 1.6963, 1.6780, 1.6596, 1.6410, 1.6219, 1.6018, 1.5807, 1.5589, 1.5368, 1.5148, 1.4933, 1.4725, 1.4514, 1.4285, 1.4026, 1.3722, 1.3368, 1.2987, 1.2608, 1.2262, 1.1979, 1.1773, 1.1598, 1.1388, 1.1080, 1.0612, 0.9960, 0.9264, 0.8706, 0.8466, 0.8725, 0.9496, 1.0118, 0.9763, 0.7602, 0.3943, 0.2400, 0.2751, 0.3533, 0.3631, 0.3332, 0.3144, 0.3046, 0.2917, 0.2756, 0.2594, 0.2451, 0.2307, 0.2130, 0.1907, 0.1695, 0.1549, 0.1439, 0.1323, 0.1196, 0.1064, 0.0932, 0.0806, 0.0694, 0.0600, 0.0520, 0.0445, 0.0367, 0.0280, 0.0174, 0.0043};
  for (int i = 0; i < 181; i++)
  {
    this->cl[i] = xl[i];
    this->cd[i] = xd[i];
  }
}

//////////////////////////////////////////////////////////////////////
void UnderWaterLiftDragPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "UnderWaterLiftDragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "UnderWaterLiftDragPlugin _sdf pointer is NULL");
  this->model     = _model;
  this->modelName = _model->GetName();
  this->sdf = _sdf;

  this->world     = this->model->GetWorld();
  GZ_ASSERT(this->world, "UnderWaterLiftDragPlugin world pointer is NULL");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    this->link = this->model->GetLink(elem->Get<std::string>());
  }
  else
  {
    this->link = this->model->GetLink("rudder_link");
  }
  GZ_ASSERT(this->link, "UnderWaterLiftDragPlugin boom link pointer is NULL");

  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
  else
    this->forward = ignition::math::Vector3d(1,0,0);

  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
  else
    this->upward = ignition::math::Vector3d(0,0,1);

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<ignition::math::Vector3d>("cp");
  else
    this->cp = ignition::math::Vector3d(0,0,0);

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");
  else
    this->area = 0.19895;

  if (_sdf->HasElement("fluid_density"))
    this->rho = _sdf->Get<double>("fluid_density");
  else
    this->rho = 997.0;
}

//////////////////////////////////////////////////////////////////////
void UnderWaterLiftDragPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&UnderWaterLiftDragPlugin::OnUpdate, this));
}

//////////////////////////////////////////////////////////////////////
void UnderWaterLiftDragPlugin::OnUpdate()
{
  /////////////////////////////////////////////////////////
  //--> FLUID VELOCITY
  //      The fluid velocity are compose by to components:
  //      the negative boat velocity + the ocean/river current
  ignition::math::Vector3d boatVelVec      = this->link->WorldCoGLinearVel();
  ignition::math::Vector3d fluidNaturalVel = ignition::math::Vector3d(0,0,0);
  boatVelVec.Z(0); //--> we ignore the z component of boat velocity
  this->fluidVelVec = fluidNaturalVel - boatVelVec;
  /////////////////////////////////////////////////////////

  if (this->fluidVelVec.Length() > 0)
  {
    /////////////////////////////////////////////////////////
    //-->ATTACK ANGLE
    //    The attack angle is defined as the angle between the
    //    the rudder chord line and the inflow (see https://www.meoexams.com/post/rudder-lift-and-drag-force).
    //->Chord line (forward) direction in the World frame
    ignition::math::Vector3d chordLine = this->link->WorldPose().Rot().RotateVector(-forward);
    //->Upward direction in the World frame
    ignition::math::Vector3d upwardW = this->link->WorldPose().Rot().RotateVector(upward);
    //->Normal to sail plan (the signal of this vector depends on the boom to be port side or starboard side).
    ignition::math::Vector3d ldNormal = chordLine.Cross(upwardW);
    //->Attack angle
    double atkangle = acos((chordLine.Dot(this->fluidVelVec)) / (chordLine.Length() * this->fluidVelVec.Length()))*180.0/M_PI;
    if (atkangle > 90.0)
      atkangle = 180.0 - atkangle;
      
    ignition::math::Vector3d dragDirection = this->fluidVelVec/this->fluidVelVec.Length();
    ignition::math::Vector3d liftDirection  = dragDirection.Cross(upwardW);
    if (this->link->RelativePose().Rot().Yaw() < 0)
    {
      ldNormal *= -1; //-> As boom movement limits are -60 to 60, we impose that ldNormal always points forward.
      liftDirection *= -1; //-> As boom movement limits are -60 to 60, we impose that ldNormal always points forward.
    }
    /////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //--> COMPUTE LIFT AND DRAG FORCES
    //->dynamic pressure
    double fluidVel = this->fluidVelVec.Length();
    double q        = 0.5 * this->rho * this->area * fluidVel * fluidVel;
    int coefIndex   = (atkangle - int(atkangle)) < 0.5 ? int(atkangle)
                                                      : int(atkangle) + 1;
    ignition::math::Vector3d lift  = q * cl[coefIndex] * liftDirection;  //-> the lift force is normal to the lift-drag plan
    ignition::math::Vector3d drag  = q * cd[coefIndex] * dragDirection; //-> the drag
    ignition::math::Vector3d force = lift + drag;
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //--> APPLY FORCE
    //->Pressure center: The pressure center is sets the position in which the aerodynamic force will be applied.
    //                   It's psition are relative to the base_link's center of mass (CoG).
    //                   The applied force vector is expressed in World frame and the centor of pressure are expressed in the link own frame.
    //->Apply resultant force
    this->link->AddForceAtRelativePosition(force, this->cp);
    //////////////////////////////////////////////////////////

    float simtime = this->world->SimTime().Float();
    /*if ((simtime - int(simtime)) == 0)
    {
      std::cout<<"---------------------------------------------------"<<std::endl;
      //std::cout<<"Fluid Natural velocity vector: "<<fluidNaturalVel<<std::endl; //this->boomLink->WorldWindLinearVel()<<std::endl;
      //std::cout<<"Boat velocity vector         : "<<boatVelVec<<std::endl;
      std::cout<<"inflow velocity vector       : "<<this->fluidVelVec<<std::endl;
      std::cout<<"Chord line direction         : "<<chordLine<<std::endl;
      std::cout<<"Upward direction             : "<<upwardW<<std::endl;
      std::cout<<"drag direction               : "<<dragDirection<<std::endl;
      std::cout<<"lift direction               : "<<liftDirection<<std::endl;
      std::cout<<"RelativePose().Rot().Yaw()   : "<<this->link->RelativePose().Rot().Yaw()*180.0/M_PI<<std::endl;
      std::cout<<"ldNormal direction           : "<<ldNormal<<std::endl;
      std::cout<<"Attack angle                 : "<<atkangle<<std::endl;
      std::cout<<"cl                           : "<<cl[coefIndex]<<std::endl;
      //std::cout<<"cd                           : "<<cd[coefIndex]<<std::endl;
      //std::cout<<"Dynamic pressure             : "<<q<<std::endl;
      std::cout<<"Lift force                   : "<<lift<<std::endl;
      std::cout<<"Drag force                   : "<<drag<<std::endl;
      std::cout<<"Hydrodynamic force           : "<<force<<std::endl;
      //std::cout<<"boom CoG world pose          : "<<this->boomLink->WorldCoGPose().Pos()<<std::endl;
      //std::cout<<"base CoG world pose          : "<<this->baseLink->WorldCoGPose().Pos()<<std::endl;
      //std::cout<<"Pressure center              : "<<dynamic_cp<<std::endl;
      std::cout<<"SimTime                      : "<<simtime<<std::endl;
    }*/
    if ((simtime - int(simtime)) == 0)
    {
      std::cout<<"---------------------------------------------------"<<std::endl;
      std::cout<<"Link              : "<<this->link->GetName()<<std::endl;
      std::cout<<"Hydrodynamic force: "<<force<<std::endl;
    }
  }
}