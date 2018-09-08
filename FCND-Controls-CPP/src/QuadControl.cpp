#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  // Formulas to solve for motor thrusts: F1, F2, F3, F4
  // collThrustCmd = F1 + F2 + F3 + F4
  // momentCmd.x = (F1 + F4 - F2 - F3) * l
  // momentCmd.y = (F1 + F2 - F3 - F4) * l
  // momentCmd.z = (F1 - F2 + F3 - F4) * k
    
  float l = L / sqrt(2.f);
  float c = collThrustCmd;
  float k = kappa;
  
  // printf("momentCmd: %4.2f %4.2f %4.2f", momentCmd.x, momentCmd.y, momentCmd.z);
  float m_x_l = momentCmd.x / l;
  float m_y_l = momentCmd.y / l;
  float m_z_k = -momentCmd.z / k;

  float thrust0 = (c + m_x_l + m_y_l + m_z_k) / 4.f;
  float thrust1 = (c - m_x_l + m_y_l - m_z_k) / 4.f;
  float thrust2 = (c + m_x_l - m_y_l - m_z_k) / 4.f;
  float thrust3 = (c - m_x_l - m_y_l + m_z_k) / 4.f;
  
//  // Make sure thrust is greater than minimum for the motor
//  thrust0 = (thrust0 < minMotorThrust) ? minMotorThrust : thrust0;
//  thrust1 = (thrust1 < minMotorThrust) ? minMotorThrust : thrust1;
//  thrust2 = (thrust2 < minMotorThrust) ? minMotorThrust : thrust2;
//  thrust3 = (thrust3 < minMotorThrust) ? minMotorThrust : thrust3;
//
//  // Make sure thrust is less than maximum for the motor
//  thrust0 = (thrust0 > maxMotorThrust) ? maxMotorThrust : thrust0;
//  thrust1 = (thrust1 > maxMotorThrust) ? maxMotorThrust : thrust1;
//  thrust2 = (thrust2 > maxMotorThrust) ? maxMotorThrust : thrust2;
//  thrust3 = (thrust3 > maxMotorThrust) ? maxMotorThrust : thrust3;
  
  cmd.desiredThrustsN[0] = thrust0; // front left
  cmd.desiredThrustsN[1] = thrust1; // front right
  cmd.desiredThrustsN[2] = thrust2; // rear left
  cmd.desiredThrustsN[3] = thrust3; // rear right
  
//  cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
//  cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
//  cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
//  cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  V3F moi = V3F(Ixx, Iyy, Izz);
  momentCmd = moi * kpPQR * (pqrCmd - pqr);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float p_cmd = 0.f;
  float q_cmd = 0.f;
  
  float R33 = R(2, 2);
  float R21 = R(1, 0);
  float R11 = R(0, 0);
  float R22 = R(1, 1);
  float R12 = R(0, 1);
  float R13 = R(0, 2);
  float R23 = R(1, 2);
  
  if (collThrustCmd > 0.f) {
    float commandedAccel = - collThrustCmd / mass;
    float target_R13 = accelCmd.x / commandedAccel;
    float target_R23 = accelCmd.y / commandedAccel;
    target_R13 = (target_R13 < -maxTiltAngle) ? -maxTiltAngle : target_R13;
    target_R23 = (target_R23 < -maxTiltAngle) ? -maxTiltAngle : target_R23;
    target_R13 = (target_R13 > maxTiltAngle) ? maxTiltAngle : target_R13;
    target_R23 = (target_R23 > maxTiltAngle) ? maxTiltAngle : target_R23;
    
    float b_x_dot_c = kpBank * (R13 - target_R13);
    float b_y_dot_c = kpBank * (R23 - target_R23);
    
    p_cmd = (1.f / R33) * (-R21 * b_x_dot_c + R11 * b_y_dot_c);
    q_cmd = (1.f / R33) * (-R22 * b_x_dot_c + R12 * b_y_dot_c);
  }
  pqrCmd = V3F(p_cmd, q_cmd, 0.f);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
 
  velZCmd = (velZCmd > maxAscentRate) ? maxAscentRate : velZCmd;
  velZCmd = (velZCmd < -maxDescentRate) ? -maxDescentRate : velZCmd;
  
  float z_err = posZCmd - posZ;
  float z_dot_err = velZCmd - velZ;
  
  float u1_bar = kpPosZ * z_err + kpVelZ * z_dot_err + accelZCmd;
  
  float b_z = R(2, 2);
  
  thrust = - (u1_bar * CONST_GRAVITY) / b_z;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  velCmd = (velCmd.mag() > maxSpeedXY) ? velCmd.norm() * maxSpeedXY : velCmd;

  V3F posErr = posCmd - pos;
  V3F velErr = velCmd - vel;
  V3F kpPos = V3F(kpPosXY, kpPosXY, 0);
  V3F kpVel = V3F(kpVelXY, kpVelXY, 0);

  accelCmd = kpPos * posErr + kpVel * velErr + accelCmdFF;

  accelCmd = (accelCmd.mag() > maxAccelXY) ? accelCmd.norm() * maxAccelXY : accelCmd;
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  yawRateCmd = kpYaw * (yawCmd - yaw);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
