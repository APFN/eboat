/*
    This plugin computes the forces acting on a sail in 
    the presence of wind. The principle of lift and drag
    is applied to compute the forces

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

#include <iostream>
#include <fstream>

#include "../include/liftDragForces.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LiftDragForces)

//////////////////////////////////////////////////////////////////////
LiftDragForces::LiftDragForces()
{
  //->lift and drag coeficients for the sail
  double sxl[] = {0.0000, 0.0283, 0.0498, 0.0671, 0.0829, 0.1000, 0.1211, 0.1489, 0.1862, 0.2357, 0.3000, 0.3804, 0.4714, 0.5663, 0.6582, 0.7400, 0.8069, 0.8616, 0.9089, 0.9535, 1.0000, 1.0518, 1.1066, 1.1604, 1.2095, 1.2500, 1.2791, 1.2972, 1.3058, 1.3063, 1.3000, 1.2885, 1.2738, 1.2577, 1.2425, 1.2300, 1.2217, 1.2164, 1.2122, 1.2074, 1.2000, 1.1888, 1.1744, 1.1582, 1.1413, 1.1250, 1.1102, 1.0970, 1.0848, 1.0734, 1.0625, 1.0516, 1.0404, 1.0284, 1.0151, 1.0000, 0.9829, 0.9640, 0.9436, 0.9222, 0.9000, 0.8776, 0.8556, 0.8348, 0.8160, 0.8000, 0.7871, 0.7762, 0.7658, 0.7542, 0.7400, 0.7221, 0.7010, 0.6779, 0.6539, 0.6300, 0.6071, 0.5850, 0.5633, 0.5418, 0.5200, 0.4976, 0.4745, 0.4505, 0.4257, 0.4000, 0.3733, 0.3458, 0.3176, 0.2889, 0.2600, 0.2310, 0.2021, 0.1738, 0.1463, 0.1200, 0.0950, 0.0710, 0.0475, 0.0240, 0.0000, -0.0325, -0.0644, -0.0958, -0.1266, -0.1569, -0.1867, -0.2159, -0.2446, -0.2727, -0.3003, -0.3273, -0.3538, -0.3797, -0.4051, -0.4300, -0.4543, -0.4781, -0.5013, -0.5240, -0.5462, -0.5678, -0.5888, -0.6093, -0.6293, -0.6487, -0.6676, -0.6859, -0.7037, -0.7210, -0.7377, -0.7539, -0.7695, -0.7846, -0.7991, -0.8131, -0.8265, -0.8394, -0.8518, -0.8636, -0.8749, -0.8856, -0.8958, -0.9054, -0.9145, -0.9231, -0.9311, -0.9386, -0.9455, -0.9519, -0.9577, -0.9630, -0.9677, -0.9719, -0.9756, -0.9787, -0.9813, -0.9833, -0.9848, -0.9858, -0.9862, -0.9860, -0.9853, -0.9841, -0.9823, -0.9800, -0.9771, -0.9737, -0.9698, -0.9653, -0.9603, -0.9547, -0.9486, -0.9419, -0.9347, -0.9269, -0.9186, -0.9098, -0.9004, -0.8905, -0.8800};
  double sxd[] = {0.0500, 0.0488, 0.0484, 0.0489, 0.0502, 0.0523, 0.0553, 0.0591, 0.0638, 0.0693, 0.0756, 0.0828, 0.0909, 0.0997, 0.1094, 0.1200, 0.1314, 0.1436, 0.1567, 0.1706, 0.1854, 0.2010, 0.2175, 0.2348, 0.2529, 0.2719, 0.2917, 0.3123, 0.3338, 0.3562, 0.3794, 0.4028, 0.4260, 0.4488, 0.4714, 0.4937, 0.5156, 0.5373, 0.5587, 0.5798, 0.6005, 0.6210, 0.6412, 0.6611, 0.6807, 0.7000, 0.7190, 0.7377, 0.7561, 0.7742, 0.7920, 0.8095, 0.8268, 0.8437, 0.8603, 0.8766, 0.8927, 0.9084, 0.9238, 0.9390, 0.9538, 0.9684, 0.9826, 0.9966, 1.0102, 1.0236, 1.0367, 1.0494, 1.0619, 1.0741, 1.0859, 1.0975, 1.1088, 1.1198, 1.1303, 1.1404, 1.1501, 1.1593, 1.1681, 1.1764, 1.1843, 1.1918, 1.1988, 1.2054, 1.2115, 1.2172, 1.2225, 1.2273, 1.2317, 1.2357, 1.2392, 1.2422, 1.2449, 1.2470, 1.2488, 1.2501, 1.2509, 1.2514, 1.2514, 1.2509, 1.2500, 1.2487, 1.2469, 1.2447, 1.2420, 1.2389, 1.2354, 1.2314, 1.2270, 1.2221, 1.2168, 1.2111, 1.2049, 1.1983, 1.1912, 1.1837, 1.1758, 1.1674, 1.1587, 1.1498, 1.1409, 1.1319, 1.1228, 1.1137, 1.1046, 1.0953, 1.0861, 1.0767, 1.0673, 1.0579, 1.0484, 1.0388, 1.0292, 1.0195, 1.0098, 1.0000, 0.9902, 0.9802, 0.9703, 0.9603, 0.9502, 0.9400, 0.9299, 0.9196, 0.9093, 0.8989, 0.8885, 0.8780, 0.8675, 0.8569, 0.8462, 0.8355, 0.8248, 0.8139, 0.8031, 0.7921, 0.7811, 0.7701, 0.7590, 0.7478, 0.7366, 0.7253, 0.7139, 0.7025, 0.6911, 0.6796, 0.6680, 0.6564, 0.6447, 0.6329, 0.6211, 0.6093, 0.5974, 0.5854, 0.5734, 0.5613, 0.5491, 0.5369, 0.5247, 0.5124, 0.5000};
  for (int i = 0; i < 181; i++)
  {
    this->sailCL[i] = sxl[i];
    this->sailCD[i] = sxd[i];
  }
  //->lift and drag coeficients for the rudder
  double rxl[] = {0.0000, 0.0649, 0.1129, 0.1473, 0.1711, 0.1877, 0.2002, 0.2118, 0.2255, 0.2434, 0.2673, 0.2980, 0.3319, 0.3642, 0.3941, 0.4363, 0.5023, 0.5736, 0.6301, 0.6738, 0.7125, 0.7521, 0.7914, 0.8274, 0.8593, 0.8948, 0.9357, 0.9527, 0.9291, 0.9293, 1.0384, 1.2652, 1.3859, 1.3855, 1.3202, 1.2461, 1.2079, 1.2040, 1.2209, 1.2456, 1.2647, 1.2681, 1.2578, 1.2393, 1.2175, 1.1979, 1.1843, 1.1754, 1.1687, 1.1615, 1.1514, 1.1363, 1.1170, 1.0945, 1.0703, 1.0456, 1.0213, 0.9974, 0.9736, 0.9495, 0.9248, 0.8992, 0.8729, 0.8459, 0.8186, 0.7910, 0.7633, 0.7355, 0.7074, 0.6788, 0.6496, 0.6197, 0.5892, 0.5582, 0.5268, 0.4951, 0.4632, 0.4311, 0.3989, 0.3664, 0.3338, 0.3010, 0.2679, 0.2347, 0.2014, 0.1680, 0.1345, 0.1009, 0.0673, 0.0336, 0.0000, -0.0336, -0.0673, -0.1009, -0.1345, -0.1680, -0.2014, -0.2347, -0.2679, -0.3010, -0.3338, -0.3664, -0.3989, -0.4311, -0.4632, -0.4951, -0.5268, -0.5582, -0.5892, -0.6197, -0.6496, -0.6788, -0.7074, -0.7355, -0.7633, -0.7910, -0.8186, -0.8459, -0.8729, -0.8992, -0.9248, -0.9495, -0.9736, -0.9974, -1.0213, -1.0456, -1.0703, -1.0945, -1.1170, -1.1363, -1.1514, -1.1615, -1.1687, -1.1754, -1.1843, -1.1979, -1.2175, -1.2393, -1.2578, -1.2681, -1.2647, -1.2456, -1.2209, -1.2040, -1.2079, -1.2461, -1.3202, -1.3855, -1.3859, -1.2652, -1.0384, -0.9293, -0.9291, -0.9527, -0.9357, -0.8948, -0.8593, -0.8274, -0.7914, -0.7521, -0.7125, -0.6738, -0.6301, -0.5736, -0.5023, -0.4363, -0.3941, -0.3642, -0.3319, -0.2980, -0.2673, -0.2434, -0.2255, -0.2118, -0.2002, -0.1877, -0.1711, -0.1473, -0.1129, -0.0649, 0.0000};
  double rxd[] = {0.0043, 0.0174, 0.0280, 0.0367, 0.0445, 0.0520, 0.0600, 0.0694, 0.0806, 0.0932, 0.1064, 0.1196, 0.1323, 0.1439, 0.1549, 0.1695, 0.1907, 0.2130, 0.2307, 0.2451, 0.2594, 0.2756, 0.2917, 0.3046, 0.3144, 0.3332, 0.3631, 0.3533, 0.2751, 0.2400, 0.3943, 0.7602, 0.9763, 1.0118, 0.9496, 0.8725, 0.8466, 0.8706, 0.9264, 0.9960, 1.0612, 1.1080, 1.1388, 1.1598, 1.1773, 1.1979, 1.2262, 1.2608, 1.2987, 1.3368, 1.3722, 1.4026, 1.4285, 1.4514, 1.4725, 1.4933, 1.5148, 1.5368, 1.5589, 1.5807, 1.6018, 1.6219, 1.6410, 1.6596, 1.6780, 1.6963, 1.7148, 1.7333, 1.7513, 1.7686, 1.7847, 1.7994, 1.8129, 1.8253, 1.8368, 1.8476, 1.8579, 1.8677, 1.8768, 1.8853, 1.8931, 1.9001, 1.9064, 1.9119, 1.9166, 1.9206, 1.9239, 1.9264, 1.9282, 1.9294, 1.9298, 1.9294, 1.9282, 1.9264, 1.9239, 1.9206, 1.9166, 1.9119, 1.9064, 1.9001, 1.8931, 1.8853, 1.8768, 1.8677, 1.8579, 1.8476, 1.8368, 1.8253, 1.8129, 1.7994, 1.7847, 1.7686, 1.7513, 1.7333, 1.7148, 1.6963, 1.6780, 1.6596, 1.6410, 1.6219, 1.6018, 1.5807, 1.5589, 1.5368, 1.5148, 1.4933, 1.4725, 1.4514, 1.4285, 1.4026, 1.3722, 1.3368, 1.2987, 1.2608, 1.2262, 1.1979, 1.1773, 1.1598, 1.1388, 1.1080, 1.0612, 0.9960, 0.9264, 0.8706, 0.8466, 0.8725, 0.9496, 1.0118, 0.9763, 0.7602, 0.3943, 0.2400, 0.2751, 0.3533, 0.3631, 0.3332, 0.3144, 0.3046, 0.2917, 0.2756, 0.2594, 0.2451, 0.2307, 0.2130, 0.1907, 0.1695, 0.1549, 0.1439, 0.1323, 0.1196, 0.1064, 0.0932, 0.0806, 0.0694, 0.0600, 0.0520, 0.0445, 0.0367, 0.0280, 0.0174, 0.0043};
  for (int i = 0; i < 181; i++)
  {
    this->rudderCL[i] = rxl[i];
    this->rudderCD[i] = rxd[i];
  }
}

//////////////////////////////////////////////////////////////////////
void LiftDragForces::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LiftDragForces _model pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDragForces _sdf pointer is NULL");
  this->model = _model;
  this->sdf   = _sdf;
  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDragForces world pointer is NULL");
  //------------------------------------------------------------------
  if (_sdf->HasElement("boom_link_name"))
  {
  sdf::ElementPtr elem = _sdf->GetElement("boom_link_name");
  this->boomLink = this->model->GetLink(elem->Get<std::string>());
  }
  else
  {
  this->boomLink = this->model->GetLink("boom_link");
  }
  GZ_ASSERT(this->boomLink, "LiftDragForcesPlugin boom link pointer is NULL");
  //------------------------------------------------------------------
  if (_sdf->HasElement("rudder_link_name"))
  {
  sdf::ElementPtr elem = _sdf->GetElement("rudder_link_name");
  this->rudderLink = this->model->GetLink(elem->Get<std::string>());
  }
  else
  {
  this->rudderLink = this->model->GetLink("rudder_link");
  }
  GZ_ASSERT(this->rudderLink, "LiftDragForcesPlugin rudder link pointer is NULL");
  //------------------------------------------------------------------
  if (_sdf->HasElement("keel_link_name"))
  {
  sdf::ElementPtr elem = _sdf->GetElement("keel_link_name");
  this->keelLink = this->model->GetLink(elem->Get<std::string>());
  }
  else
  {
  this->keelLink = this->model->GetLink("keel_link");
  }
  GZ_ASSERT(this->keelLink, "LiftDragForcesPlugin keel link pointer is NULL");
  //------------------------------------------------------------------
  if (_sdf->HasElement("base_link_name"))
  {
  sdf::ElementPtr elem = _sdf->GetElement("base_link_name");
  this->baseLink = this->model->GetLink(elem->Get<std::string>());
  }
  else
  {
  this->baseLink = this->model->GetLink("base_link");
  }
  GZ_ASSERT(this->baseLink, "LiftDragForcesPlugin base link pointer is NULL");
  //------------------------------------------------------------------
  if (_sdf->HasElement("sail_forward"))
      this->sailForward = _sdf->Get<ignition::math::Vector3d>("sail_forward");
  else
      this->sailForward = ignition::math::Vector3d(1,0,0);
  //------------------------------------------------------------------
  if (_sdf->HasElement("rudder_forward"))
      this->rudderForward = _sdf->Get<ignition::math::Vector3d>("rudder_forward");
  else
      this->rudderForward = ignition::math::Vector3d(1,0,0);
  //------------------------------------------------------------------
  if (_sdf->HasElement("keel_forward"))
      this->keelForward = _sdf->Get<ignition::math::Vector3d>("keel_forward");
  else
      this->keelForward = ignition::math::Vector3d(1,0,0);
  //------------------------------------------------------------------
  if (_sdf->HasElement("sail_upward"))
      this->sailUpward = _sdf->Get<ignition::math::Vector3d>("sail_upward");
  else
      this->sailUpward = ignition::math::Vector3d(0,0,1);
  //------------------------------------------------------------------
  if (_sdf->HasElement("rudder_upward"))
      this->rudderUpward = _sdf->Get<ignition::math::Vector3d>("rudder_upward");
  else
      this->rudderUpward = ignition::math::Vector3d(0,0,1);
  //------------------------------------------------------------------
  if (_sdf->HasElement("keel_upward"))
      this->keelUpward = _sdf->Get<ignition::math::Vector3d>("keel_upward");
  else
      this->keelUpward = ignition::math::Vector3d(0,0,1);
  //------------------------------------------------------------------
  if (_sdf->HasElement("sail_cp"))
    this->sail_cp = _sdf->Get<ignition::math::Vector3d>("sail_cp");
  else
    this->sail_cp = ignition::math::Vector3d(0.5,0,1.5);
  //------------------------------------------------------------------
  if (_sdf->HasElement("keel_cp"))
    this->keel_cp = _sdf->Get<ignition::math::Vector3d>("keel_cp");
  else
    this->keel_cp = ignition::math::Vector3d(0,0,0.5);
  //------------------------------------------------------------------
  if (_sdf->HasElement("sail_area"))
    this->sailArea = _sdf->Get<double>("sail_area");
  else
    this->sailArea = 3.0;
  //------------------------------------------------------------------
  if (_sdf->HasElement("rudder_area"))
    this->rudderArea = _sdf->Get<double>("rudder_area");
  else
    this->rudderArea = 0.19895;
  //------------------------------------------------------------------
  if (_sdf->HasElement("keel_area"))
    this->keelArea = _sdf->Get<double>("keel_area");
  else
    this->keelArea = 0.2365;
  //------------------------------------------------------------------
  if (_sdf->HasElement("air_density"))
    this->airRHO = _sdf->Get<double>("air_density");
  else
    this->airRHO = 1.1839;
  //------------------------------------------------------------------
  if (_sdf->HasElement("water_density"))
    this->waterRHO = _sdf->Get<double>("water_density");
  else
    this->waterRHO = 997.0;
}

//////////////////////////////////////////////////////////////////////
void LiftDragForces::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LiftDragForces::OnUpdate, this));
  
  this->armLength = 0.59; //-->1/3 of boom length //((this->boomLink->WorldCoGPose().Pos() + this->sail_cp) - this->model->GetJoint("boom_joint")->WorldPose().Pos()).Dot(this->sailForward);
  this->boomLink->SetWindEnabled(true);
  //this->currentFlowVelVec = ignition::math::Vector3d(0,0,0);

  /*std::ofstream myfile;
  myfile.open ("/home/eduardo/USVSim/scripts/aerodynamics.csv");
  myfile << "TYPE;ATKANGLE;X;Y;Z;LENGTH\n";
  myfile.close();
  myfile.open ("/home/eduardo/USVSim/scripts/hydrodynamicsR.csv");
  myfile << "TYPE;ATKANGLE;X;Y;Z;LENGTH\n";
  myfile.close();
  myfile.open ("/home/eduardo/USVSim/scripts/hydrodynamicsK.csv");
  myfile << "TYPE;ATKANGLE;X;Y;Z;LENGTH\n";
  myfile.close();
  */
}

//////////////////////////////////////////////////////////////////////
void LiftDragForces::OnUpdate()
{
  /*double rotang = int(this->world->SimTime().Float()) * M_PI / 6.0;
  ignition::math::Vector3d water;
  if (this->world->SimTime().Float() <= 12)
  {
    //ignition::math::Quaternion teste = ignition::math::Quaternion(0.0,0.0,1.5708)
    ignition::math::Vector3d wind = ignition::math::Quaternion(0.0,0.0,rotang).RotateVector(ignition::math::Vector3d(-4,0,0));
    this->world->Wind().SetLinearVel(wind);
    rotang = (int(this->world->SimTime().Float())-1) * M_PI / 6.0;
    water = ignition::math::Quaternion(0.0,0.0,rotang).RotateVector(ignition::math::Vector3d(-4,0,0));
  }*/
  /////////////////////////////////////////////////////////
  //--> Aparent wind computation
  // boatVelVec        = this->boomLink->WorldCoGLinearVel();
  // trueWindVelVec    = this->boomLink->WorldWindLinearVel();
  // aparentWindVelVec = trueWindVelVec - boatVelVec;
  this->aparentWindVelVec = this->boomLink->WorldWindLinearVel() - this->boomLink->WorldCoGLinearVel();
  //////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////
  //--> Water velocity
  // boatVelVec        = this->boomLink->WorldCoGLinearVel();
  // currentFlowVelVec = this->boomLink->WorldWindLinearVel();
  // waterVelVec       = currentFlowVelVec - boatVelVec;
  this->waterVelVec = -this->boomLink->WorldCoGLinearVel();
  //this->waterVelVec = water;
  //////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////
  //--> COMPUTE LIFT AND DRAG FORCES ON SAIL
  if (this->aparentWindVelVec.Length() > 0)
  {
    //////////////////////////////////////////////////////////
    //--> ATTACK ANGLE COMPUTATION
    //   The sail frame (boom frame) origin is defined as
    //   the joint origin. The chord line is defined by the forward
    //   vector. By default the chord line is defined in the X axis.
    //   The sail plan is defined by upward and forward vectors. By
    //   default, it is defined as the XZ plan.
    //->Chord line (forward) direction in the World frame
    //ignition::math::Vector3d chordLine = this->boomLink->WorldPose().Rot().RotateVector(-this->sailForward);
    this->chordLine = this->boomLink->WorldPose().Rot().RotateVector(-this->sailForward);
    //->Upward direction in the World frame
    //ignition::math::Vector3d upwardW   = this->boomLink->WorldPose().Rot().RotateVector(this->sailUpward);
    this->upwardW = this->boomLink->WorldPose().Rot().RotateVector(this->sailUpward);
    //->Normal to the chord line
    ignition::math::Vector3d normal = this->chordLine.Cross(this->upwardW);
    //->Attack angle
    double atkangle = acos((this->chordLine.Dot(this->aparentWindVelVec)) / (this->chordLine.Length() * this->aparentWindVelVec.Length()))*this->r2d;
    //->alpha is the angle between the apparent wind direction and the normal to sail plan.
    double alpha    = acos(normal.Dot(this->aparentWindVelVec)/(normal.Length() * this->aparentWindVelVec.Length()))*this->r2d;
    //->Lift and drag directions
    this->dragDirection = this->aparentWindVelVec/this->aparentWindVelVec.Length();
    this->liftDirection = this->upwardW.Cross(this->dragDirection);
    if (alpha < 90)
      liftDirection  *= -1;
    else
      normal *= -1;
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //--> COMPUTE LIFT AND DRAG FORCES
    //->dynamic pressure
    double q       = 0.5 * this->airRHO * this->sailArea * this->aparentWindVelVec.Length() * this->aparentWindVelVec.Length();
    int coefIndex  = (atkangle - int(atkangle)) < 0.5 ? int(atkangle)
                                                      : int(atkangle) + 1;
    ignition::math::Vector3d lift;  //-> the lift force is normal to the drag force
    if (coefIndex > 100)
      lift  = q * sailCL[100] * liftDirection;
    else
      lift  = q * sailCL[coefIndex] * liftDirection;
    ignition::math::Vector3d drag  = q * sailCD[coefIndex] * dragDirection;  //-> the drag
    ignition::math::Vector3d force = lift + drag;
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //--> APPLY FORCE
    //->Pressure center: The pressure center is sets the position in which the aerodynamic force will be applied.
    //                   It's psition are relative to the base_link's center of mass (CoG).
    //                   The applied force vector is expressed in World frame and the centor of pressure are expressed in the link own frame.
    ignition::math::Vector3d dynamic_cp = (this->boomLink->WorldCoGPose().Pos() + this->boomLink->WorldCoGPose().Rot().RotateVector(this->sail_cp)) - this->baseLink->WorldCoGPose().Pos();
    //->Moving force: Due to some URDF/SDF limitations on build joints, if we apply the wind force on the sail 3D element (sail_link + boom_link) this force
    //                will be transmited to the boat by the boom_joint (a revoltute joint). Therefore, the wind force could generate a not realistic torque
    //                on the revolute axis. To avoid this and emulate a more realistic boat behavior, we will apply the force direct to the boat 3D element.
    //                The position where the force will be applied is given by the dynamic_cp variable. 
    this->baseLink->AddForceAtRelativePosition(force, dynamic_cp);
    //->Force on sail: As we will aplly the wind force direct on the boat element, the sail element will stand still and it will not change position with the
    //                 wind direction and speed, as it should do.
    //                 In a real sailing boat the crew do not set a fixe position for the boom, they increases and decrease the boom cable in order to give the
    //                 boom more or less freedom of movement. The boom's position is defined by how much cable the crew released and the wind's direction and speed.
    //                 To emulate this behavior, the sail 3D element should change position according the wind's direction and speed.
    //
    //                 torque = (force.Dot(ldNormal)*ldNormal).Length()*armLength; // modulus of projected force times the arm length.
    //
    // this->joint->SetForce(0, ((force.Dot(normal)*normal).Length()*armLength));
    double torque = (alpha < 90) ? (force.Dot(normal)*normal).Length()*this->armLength*(-1) // Negative
                                 : (force.Dot(normal)*normal).Length()*this->armLength;     // Positive
    this->boomLink->GetParentJoints()[0]->SetForce(0, torque);

    float simtime = this->world->SimTime().Float();
    if ((simtime - int(simtime)) == 0)
    {
      std::cout<<"---------------------------------------------------"<<std::endl;
      /*std::cout<<"SimTime            : "<<simtime<<std::endl;
      std::cout<<"Link               : "<<this->boomLink->GetName()<<std::endl;
      std::cout<<"Attack angle       : "<<atkangle<<std::endl;
      std::cout<<"Lift force on sail : "<<lift<<" |"<<lift.Length()<<"|"<<std::endl;
      std::cout<<"Drag force on sail : "<<drag<<" |"<<drag.Length()<<"|"<<std::endl;*/
      std::cout<<"Aerodynamic force    : "<<force<<" |"<<force.Length()<<"|"<<std::endl;
      
      /*std::ofstream myfile;
      myfile.open ("/home/eduardo/USVSim/scripts/aerodynamics.csv", std::ios::app);
      myfile << "wind;" << atkangle << ";" << aparentWindVelVec.X() << ";" << aparentWindVelVec.Y() << ";" << aparentWindVelVec.Z() << ";" << aparentWindVelVec.Length() << "\n";
      myfile << "lift;" << atkangle << ";" << lift.X() << ";" << lift.Y() << ";" << lift.Z() << ";" << lift.Length() << "\n";
      myfile << "drag;" << atkangle << ";" << drag.X() << ";" << drag.Y() << ";" << drag.Z() << ";" << drag.Length() << "\n";
      myfile << "force;" << atkangle << ";" << force.X() << ";" << force.Y() << ";" << force.Z() << ";" << force.Length() << "\n";
      myfile.close();*/
    }
    physics::ModelPtr cp_marker = this->world->ModelByName("marker");
    if (cp_marker != NULL)
    {
      ignition::math::Pose3d pose;
      pose.Set(this->baseLink->WorldCoGPose().Pos() + dynamic_cp, this->baseLink->WorldCoGPose().Rot());
      cp_marker->SetWorldPose(pose);
    }
  }
  //////////////////////////////////////////////////////////////////////////

  if (this->waterVelVec.Length() > 0)
  {
    ////////////////////////////////////////////////////////////////////
    //                             RUDDER                             //
    //////////////////////////////////////////////////////////
    //--> ATTACK ANGLE COMPUTATION
    this->chordLine = this->rudderLink->WorldPose().Rot().RotateVector(-this->rudderForward);
    //->Upward direction in the World frame
    this->upwardW   = this->rudderLink->WorldPose().Rot().RotateVector(this->rudderUpward);
    //->Normal to the chord line
    ignition::math::Vector3d normal = this->chordLine.Cross(this->upwardW);
    //->Attack angle
    double atkangle = acos((this->chordLine.Dot(this->waterVelVec)) / (this->chordLine.Length() * this->waterVelVec.Length()))*this->r2d;
    //->alpha is the angle between the apparent wind direction and the normal to sail plan.
    double alpha    = acos(normal.Dot(this->waterVelVec)/(normal.Length() * this->waterVelVec.Length()))*this->r2d;
    //->Lift and drag directions
    this->dragDirection = this->waterVelVec/this->waterVelVec.Length();
    this->liftDirection = this->upwardW.Cross(this->dragDirection);
    if (alpha < 90)
      liftDirection  *= -1;
    else
      normal *= -1;
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //--> COMPUTE LIFT AND DRAG FORCES
    //->dynamic pressure
    double q       = 0.5 * this->waterRHO * this->rudderArea * this->waterVelVec.Length() * this->waterVelVec.Length();
    int coefIndex  = (atkangle - int(atkangle)) < 0.5 ? int(atkangle)
                                                      : int(atkangle) + 1;
    ignition::math::Vector3d rlift;  //-> the lift force is normal to the drag force
    if (coefIndex == 180.0)
      rlift = 0.0 * liftDirection;
    else if (coefIndex > 90)
      rlift = q * (-rudderCL[coefIndex]) * liftDirection;
    else
      rlift = q * rudderCL[coefIndex] * liftDirection;
    ignition::math::Vector3d rdrag  = q * rudderCD[coefIndex] * dragDirection;  //-> the drag
    ignition::math::Vector3d rforce = rlift + rdrag;
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //--> APPLY FORCE
    //->Pressure center: The pressure center is sets the position in which the aerodynamic force will be applied.
    //                   It's psition are relative to the base_link's center of mass (CoG).
    //                   The applied force vector is expressed in World frame and the centor of pressure are expressed in the link own frame.
    //->Apply resultant force
    //this->rudderLink->AddForceAtRelativePosition(rforce, this->cp);
    this->rudderLink->AddForce(rforce);
    //////////////////////////////////////////////////////////

    float simtime = this->world->SimTime().Float();
    if ((simtime - int(simtime)) == 0)
    {
      //this->chordLine = this->boomLink->WorldPose().Rot().RotateVector(-this->sailForward);
      /*std::cout<<"---------------------------------------------------"<<std::endl;
      std::cout<<"SimTime              : "<<simtime<<std::endl;
      std::cout<<"Link                 : "<<this->rudderLink->GetName()<<std::endl;
      std::cout<<"Attack angle         : "<<atkangle<<std::endl;
      std::cout<<"Lift force on rudder : "<<rlift<<" |"<<rlift.Length()<<"|"<<std::endl;
      std::cout<<"Drag force on rudder : "<<rdrag<<" |"<<rdrag.Length()<<"|"<<std::endl;*/
      std::cout<<"Total force on rudder: "<<rforce<<" |"<<rforce.Length()<<"|"<<std::endl;
      
      /*std::ofstream myfile;
      myfile.open ("/home/eduardo/USVSim/scripts/hydrodynamicsR.csv", std::ios::app);
      myfile << "water;" << atkangle << ";" << waterVelVec.X() << ";" << waterVelVec.Y() << ";" << waterVelVec.Z() << ";" << waterVelVec.Length() << "\n";
      myfile << "lift;" << atkangle << ";" << rlift.X() << ";" << rlift.Y() << ";" << rlift.Z() << ";" << rlift.Length() << "\n";
      myfile << "drag;" << atkangle << ";" << rdrag.X() << ";" << rdrag.Y() << ";" << rdrag.Z() << ";" << rdrag.Length() << "\n";
      myfile << "force;" << atkangle << ";" << rforce.X() << ";" << rforce.Y() << ";" << rforce.Z() << ";" << rforce.Length() << "\n";
      myfile.close();*/
    }

    ////////////////////////////////////////////////////////////////////
    //                              KEEL                              //
    //////////////////////////////////////////////////////////
    //--> ATTACK ANGLE COMPUTATION
    this->chordLine = this->keelLink->WorldPose().Rot().RotateVector(-this->keelForward);
    //->Upward direction in the World frame
    this->upwardW   = this->keelLink->WorldPose().Rot().RotateVector(this->keelUpward);
    //->Normal to the chord line
    normal   = this->chordLine.Cross(this->upwardW);
    //->Attack angle
    atkangle = acos((this->chordLine.Dot(this->waterVelVec)) / (this->chordLine.Length() * this->waterVelVec.Length()))*this->r2d;
    //->alpha is the angle between the apparent wind direction and the normal to sail plan.
    alpha    = acos(normal.Dot(this->waterVelVec)/(normal.Length() * this->waterVelVec.Length()))*this->r2d;
    //->Lift and drag directions
    this->dragDirection = this->waterVelVec/this->waterVelVec.Length();
    this->liftDirection = this->upwardW.Cross(this->dragDirection);
    if (alpha < 90)
      this->liftDirection *= -1;
    else
      normal *= -1;
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //--> COMPUTE LIFT AND DRAG FORCES
    //->dynamic pressure
    q          = 0.5 * this->waterRHO * this->keelArea * this->waterVelVec.Length() * this->waterVelVec.Length();
    coefIndex  = (atkangle - int(atkangle)) < 0.5 ? int(atkangle)
                                            : int(atkangle) + 1;
    ignition::math::Vector3d klift;  //-> the lift force is normal to the drag force
    if (coefIndex == 180.0)
      klift = 0.0 * liftDirection;
    else if (coefIndex > 90)
      klift = q * (-rudderCL[coefIndex]) * liftDirection;
    else
      klift = q * rudderCL[coefIndex] * liftDirection;
    ignition::math::Vector3d kdrag  = q * rudderCD[coefIndex] * dragDirection;  //-> the drag
    ignition::math::Vector3d kforce = klift + kdrag;
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //--> APPLY FORCE
    //->Pressure center: The pressure center is sets the position in which the aerodynamic force will be applied.
    //                   It's psition are relative to the base_link's center of mass (CoG).
    //                   The applied force vector is expressed in World frame and the centor of pressure are expressed in the link own frame.
    ignition::math::Vector3d dynamic_cp = (this->boomLink->WorldCoGPose().Pos() + this->boomLink->WorldCoGPose().Rot().RotateVector(this->keel_cp)) - this->baseLink->WorldCoGPose().Pos();
    //->Apply resultant force
    //this->keelLink->AddForceAtRelativePosition(kforce, dynamic_cp);
    //this->keelLink->AddForce(kforce);
    //////////////////////////////////////////////////////////

    if ((simtime - int(simtime)) == 0)
    {
      /*std::cout<<"---------------------------------------------------"<<std::endl;
      std::cout<<"SimTime              : "<<simtime<<std::endl;
      std::cout<<"Link                 : "<<this->keelLink->GetName()<<std::endl;
      std::cout<<"Attack angle         : "<<atkangle<<std::endl;
      std::cout<<"Lift force on keel : "<<klift<<" |"<<klift.Length()<<"|"<<std::endl;
      std::cout<<"Drag force on keel : "<<kdrag<<" |"<<kdrag.Length()<<"|"<<std::endl;*/
      std::cout<<"Total force on keel  : "<<kforce<<" |"<<kforce.Length()<<"|"<<std::endl;
      
      //std::ofstream myfile;
      //myfile.open ("/home/eduardo/USVSim/scripts/hydrodynamicsK.csv", std::ios::app);
      //myfile << "water;" << atkangle << ";" << waterVelVec.X() << ";" << waterVelVec.Y() << ";" << waterVelVec.Z() << ";" << waterVelVec.Length() << "\n";
      //myfile << "lift;" << atkangle << ";" << klift.X() << ";" << klift.Y() << ";" << klift.Z() << ";" << klift.Length() << "\n";
      //myfile << "drag;" << atkangle << ";" << kdrag.X() << ";" << kdrag.Y() << ";" << kdrag.Z() << ";" << kdrag.Length() << "\n";
      //myfile << "force;" << atkangle << ";" << kforce.X() << ";" << kforce.Y() << ";" << kforce.Z() << ";" << kforce.Length() << "\n";
      //myfile.close();


      /*std::cout<<simtime<<std::endl;
      std::cout<<"Total force on base link  : "<<this->baseLink->WorldForce()<<std::endl;
      std::cout<<"Total force on rudder link: "<<this->rudderLink->WorldForce()<<std::endl;
      std::cout<<"Total force on keel link  : "<<this->keelLink->WorldForce()<<std::endl;
      std::cout<<"baseLink angular momentum : "<<this->baseLink->WorldAngularMomentum()<<std::endl;

      std::ofstream myfile;
      myfile.open ("/home/eduardo/USVSim/scripts/forces_on.csv", std::ios::app);
      myfile << "base_link_force;" << this->baseLink->WorldForce().X() << ";" << this->baseLink->WorldForce().Y() << ";" << this->baseLink->WorldForce().Z() << "\n";
      myfile << "rudder_link_force;" << this->rudderLink->WorldForce().X() << ";" << this->rudderLink->WorldForce().Y() << ";" << this->rudderLink->WorldForce().Z() << "\n";
      myfile << "keel_link_force;" << this->keelLink->WorldForce().X() << ";" << this->keelLink->WorldForce().Y() << ";" << this->keelLink->WorldForce().Z() << "\n";
      myfile << "base_link_angmom;" << this->baseLink->WorldAngularMomentum().X() << ";" << this->baseLink->WorldAngularMomentum().Y() << ";" << this->baseLink->WorldAngularMomentum().Z() << "\n";
      myfile.close();*/
    }

    /*if (simtime >= 12)
      this->world->SetPaused(true);
    */
  }
}