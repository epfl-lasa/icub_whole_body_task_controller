
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include "RobotInterface.h"

using namespace yarp::dev;


// void *IMU_thread(void * input)
// {
//     while(1)
//     {   
//         RobotInterface * d = ((RobotInterface*)input);
//         pthread_mutex_lock(&(d->mutex));
//         d->imu.update(d->EULER, d->LINACC, d->ANGVEL);
//         pthread_mutex_unlock(&(d->mutex));
//         Sleep(1);
//     }
// }

RobotInterface::RobotInterface(){};

RobotInterface::~RobotInterface()
{
    // RobotInterface::CloseRobotSensors();
};

bool RobotInterface::init(std::string robotName_, std::string moduleName_, int n_actuatedDofs, int period)
{
    //
    DEG2RAD = M_PI/180.0;
    RAD2DEG = 180.0/M_PI;
    //
    RobotName       =   robotName_;
    ModuleName      =   moduleName_;
    actuatedDofs    =   n_actuatedDofs;
    run_period_sec  =   0.001*period;
    
    // 
    ind[0]  = 0;
    //      
    switch (actuatedDofs)
    {
        //        torso         head            larm            rarm            lleg            rleg
        case 32 : nbjts[0] = 3; nbjts[1] = 3;   nbjts[2] = 7;   nbjts[3] = 7;   nbjts[4] = 6;   nbjts[5] = 6; break;
        case 29 : nbjts[0] = 3; nbjts[1] = 0;   nbjts[2] = 7;   nbjts[3] = 7;   nbjts[4] = 6;   nbjts[5] = 6; break;
        case 25 : nbjts[0] = 3; nbjts[1] = 0;   nbjts[2] = 5;   nbjts[3] = 5;   nbjts[4] = 6;   nbjts[5] = 6; break;
        case 23 : nbjts[0] = 3; nbjts[1] = 0;   nbjts[2] = 4;   nbjts[3] = 4;   nbjts[4] = 6;   nbjts[5] = 6; break;
        default:  nbjts[0] = 3; nbjts[1] = 3;   nbjts[2] = 7;   nbjts[3] = 7;   nbjts[4] = 6;   nbjts[5] = 6; break;
    }

    for(int i=1; i<6; i++) ind[i] = ind[i-1] + nbjts[i-1];
    //
    // Joints limits
    min_jtsLimits.resize(actuatedDofs);             // joints position min  
    max_jtsLimits.resize(actuatedDofs);             // joints position max
    velocitySaturationLimit.resize(actuatedDofs);   // joints velocity limits
    torqueSaturationLimit.resize(actuatedDofs);     // joints torque limits

    // load the joints position limits
    robotJtsLimits.load_limits();
    // default value for external base (pelvis ) pose estimation
    isBaseExternal = false;
    //
    Base_port_name  = " ";

    //
    world_H_pelvis.setIdentity();
    //
    Marker_H_Pelvis.setZero();
    Marker_H_Pelvis(3,3) = 1.0;
    // rotation                             // translation
    Marker_H_Pelvis(0,0) =  -1.0;            Marker_H_Pelvis(0,3) =  0.14; // 0.09;
    Marker_H_Pelvis(1,1) =  -1.0;            Marker_H_Pelvis(1,3) = -0.08; //-0.09;
    Marker_H_Pelvis(2,2) =   1.0;            Marker_H_Pelvis(2,3) = 0.02; // 0.015;

    if(RobotName == "icubSim")
        Marker_H_Pelvis.block<3,1>(0,3).setZero(); 


    return true;
}

// yarp devices
bool RobotInterface::OpenRobotDevice(yarpDevice& robot_device,  std::string body_part, Joints_Measurements& part_sensor)
{
    // local and remote port
    // ----------------------
    std::string localPorts    = "/" + ModuleName +  "/" + body_part;
    std::string remotePorts   = "/" + RobotName  +  "/" + body_part;

    // Devices property
    // ----------------
    yarp::os::Property DeviceOptions;
    DeviceOptions.put("device", "remote_controlboard");
    DeviceOptions.put("local", localPorts.c_str());
    DeviceOptions.put("remote", remotePorts.c_str());

    // Opening the polydrivers
    // -----------------------
    robot_device.client.open(DeviceOptions);
    if (!robot_device.client.isValid()) {
        printf("Device: %s", body_part.c_str());
        printf(" not available.  Here are the known devices:\n");
        printf("%s", yarp::dev::Drivers::factory().toString().c_str());
        return false;
    }

    bool ok = true;

    ok = ok && robot_device.client.view(robot_device.ipos);
    ok = ok && robot_device.client.view(robot_device.iposDir);
    ok = ok && robot_device.client.view(robot_device.iimp);
    ok = ok && robot_device.client.view(robot_device.ictrl);
    ok = ok && robot_device.client.view(robot_device.iint);
    ok = ok && robot_device.client.view(robot_device.itrq);
    ok = ok && robot_device.client.view(robot_device.iencs);
    ok = ok && robot_device.client.view(robot_device.ivel);
    ok = ok && robot_device.client.view(robot_device.ilim);
    ok = ok && robot_device.client.view(robot_device.ipid);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return false;
    }

    // get the number of joints for each device
    // -----------------------------------------
    robot_device.iencs->getAxes(&part_sensor.nb_joints);

    std::cout << " number of Dofs is : " <<  part_sensor.nb_joints << std::endl;

    // resize the commands and the encoders vectors
    // ---------------------------------------------
    part_sensor.position.resize(part_sensor.nb_joints);
    part_sensor.velocity.resize(part_sensor.nb_joints);
    part_sensor.acceleration.resize(part_sensor.nb_joints);
    part_sensor.torque.resize(part_sensor.nb_joints);

    // get the initial encoders values
    // -------------------------------
    printf("waiting for encoders");

    while(!robot_device.iencs->getEncoders(part_sensor.position.data())                 &&
          !robot_device.iencs->getEncoderSpeeds(part_sensor.velocity.data())            &&
          !robot_device.iencs->getEncoderAccelerations(part_sensor.acceleration.data()) &&
          !robot_device.itrq->getTorques(part_sensor.torque.data()) )
    {
      yarp::os::Time::delay(0.05);
      printf(".");
    }  
      printf("\n;"); 

    // set control and interaction mode
    robot_device.ctrl_mode  = CONTROL_MODE_POSITION;
    robot_device.inter_mode = STIFF_MODE;
    robot_device.nb_joints  = part_sensor.nb_joints;

    return ok;
}


bool RobotInterface::OpenWholeBodyDevices()
{
    //
    bool ok = true;
    ok = ok && this->OpenRobotDevice(device_torso,      "torso", jts_sensor_torso);
    ok = ok && this->OpenRobotDevice(device_head,        "head", jts_sensor_head);
    ok = ok && this->OpenRobotDevice(device_larm,    "left_arm", jts_sensor_larm);
    ok = ok && this->OpenRobotDevice(device_rarm,   "right_arm", jts_sensor_rarm);
    ok = ok && this->OpenRobotDevice(device_lleg,    "left_leg", jts_sensor_lleg);
    ok = ok && this->OpenRobotDevice(device_rleg,   "right_leg", jts_sensor_rleg);

    // PID parameters
    jts_pid_torso.resize(jts_sensor_torso.nb_joints);
    jts_pid_head.resize(jts_sensor_head.nb_joints);
    jts_pid_larm.resize(jts_sensor_larm.nb_joints);
    jts_pid_rarm.resize(jts_sensor_rarm.nb_joints);
    jts_pid_lleg.resize(jts_sensor_lleg.nb_joints);
    jts_pid_rleg.resize(jts_sensor_rleg.nb_joints);

    if (!ok) {
        printf("Opening wholeBody devices failed \n");
        return false;
    }

    this->OpenRobotSensors(RobotName, run_period_sec);

    return ok;
}

void RobotInterface::CloseRobotDevice(yarpDevice& robot_device)
{
    robot_device.ipos->stop();
    robot_device.ivel->stop();
    robot_device.client.close();
}


void RobotInterface::CloseWholeBodyRobotDevices()
{
    this->CloseRobotDevice(device_torso);
    this->CloseRobotDevice(device_head);
    this->CloseRobotDevice(device_larm);
    this->CloseRobotDevice(device_rarm);
    this->CloseRobotDevice(device_lleg);
    this->CloseRobotDevice(device_rleg);
    this->CloseRobotSensors();
}


bool RobotInterface::getBodyJointsMeasurements(yarpDevice& robot_device, Joints_Measurements& part_sensor)
{
    //
    bool ok = true;
    ok = ok && robot_device.iencs->getEncoders(part_sensor.position.data());                
    ok = ok && robot_device.iencs->getEncoderSpeeds(part_sensor.velocity.data());               
    ok = ok && robot_device.iencs->getEncoderAccelerations(part_sensor.acceleration.data()); 
    ok = ok && robot_device.itrq->getTorques(part_sensor.torque.data()); 

    if (!ok) {
        printf("Failed to get the joints sensor measurements \n");
        return false;
    }

    return ok;
}


bool RobotInterface::getWholeBodyJointsMeasurements()
{
    //
    bool ok = true;
    ok = ok && this->getBodyJointsMeasurements(device_torso,  jts_sensor_torso);
    ok = ok && this->getBodyJointsMeasurements( device_head,  jts_sensor_head);
    ok = ok && this->getBodyJointsMeasurements( device_larm,  jts_sensor_larm);
    ok = ok && this->getBodyJointsMeasurements( device_rarm,  jts_sensor_rarm);
    ok = ok && this->getBodyJointsMeasurements( device_lleg,  jts_sensor_lleg);
    ok = ok && this->getBodyJointsMeasurements( device_rleg,  jts_sensor_rleg);

    if (!ok) {
        printf("Failed to get the Whole Body joints measurements \n");
        return false;
    }

    return ok;
}

bool RobotInterface::setJointsValuesInRad(Joints_Measurements& m_jts_sensor_wb)
{
    // 
    // std::cout << " index is : "<< ind[0] << " , " << ind[1] << " , "<< ind[2] << " , "<< ind[3] << " , "<< ind[4] << " , "<< ind[5] << std::endl;
    // std::cout << " nbjts is : "<< nbjts[0] << " , " << nbjts[1] << " , "<< nbjts[2] << " , "<< nbjts[3] << " , "<< nbjts[4] << " , "<< nbjts[5] << std::endl;

    if((actuatedDofs == 29)||(actuatedDofs == 25)||(actuatedDofs == 23))
    {
        m_jts_sensor_wb.position.segment(ind[0],  nbjts[0])     =   DEG2RAD * jts_sensor_torso.position.segment(0,  nbjts[0]);
        m_jts_sensor_wb.position.segment(ind[2],  nbjts[2])     =   DEG2RAD * jts_sensor_larm.position.segment( 0,  nbjts[2]);
        m_jts_sensor_wb.position.segment(ind[3],  nbjts[3])     =   DEG2RAD * jts_sensor_rarm.position.segment( 0,  nbjts[3]);
        m_jts_sensor_wb.position.segment(ind[4],  nbjts[4])     =   DEG2RAD * jts_sensor_lleg.position.segment( 0,  nbjts[4]);
        m_jts_sensor_wb.position.segment(ind[5],  nbjts[5])     =   DEG2RAD * jts_sensor_rleg.position.segment( 0,  nbjts[5]);

        m_jts_sensor_wb.velocity.segment(ind[0],  nbjts[0])     =   DEG2RAD * jts_sensor_torso.velocity.segment(0,  nbjts[0]);
        m_jts_sensor_wb.velocity.segment(ind[2],  nbjts[2])     =   DEG2RAD * jts_sensor_larm.velocity.segment( 0,  nbjts[2]);
        m_jts_sensor_wb.velocity.segment(ind[3],  nbjts[3])     =   DEG2RAD * jts_sensor_rarm.velocity.segment( 0,  nbjts[3]);
        m_jts_sensor_wb.velocity.segment(ind[4],  nbjts[4])     =   DEG2RAD * jts_sensor_lleg.velocity.segment( 0,  nbjts[4]);
        m_jts_sensor_wb.velocity.segment(ind[5],  nbjts[5])     =   DEG2RAD * jts_sensor_rleg.velocity.segment( 0,  nbjts[5]);

        m_jts_sensor_wb.acceleration.segment(ind[0],  nbjts[0]) =   DEG2RAD * jts_sensor_torso.acceleration.segment(0,  nbjts[0]);
        m_jts_sensor_wb.acceleration.segment(ind[2],  nbjts[2]) =   DEG2RAD * jts_sensor_larm.acceleration.segment( 0,  nbjts[2]);
        m_jts_sensor_wb.acceleration.segment(ind[3],  nbjts[3]) =   DEG2RAD * jts_sensor_rarm.acceleration.segment( 0,  nbjts[3]);
        m_jts_sensor_wb.acceleration.segment(ind[4],  nbjts[4]) =   DEG2RAD * jts_sensor_lleg.acceleration.segment( 0,  nbjts[4]);
        m_jts_sensor_wb.acceleration.segment(ind[5],  nbjts[5]) =   DEG2RAD * jts_sensor_rleg.acceleration.segment( 0,  nbjts[5]);

        m_jts_sensor_wb.torque.segment(ind[0],  nbjts[0])       =   jts_sensor_torso.torque.segment(0,  nbjts[0]);
        m_jts_sensor_wb.torque.segment(ind[2],  nbjts[2])       =   jts_sensor_larm.torque.segment( 0,  nbjts[2]);
        m_jts_sensor_wb.torque.segment(ind[3],  nbjts[3])       =   jts_sensor_rarm.torque.segment( 0,  nbjts[3]);
        m_jts_sensor_wb.torque.segment(ind[4],  nbjts[4])       =   jts_sensor_lleg.torque.segment( 0,  nbjts[4]);
        m_jts_sensor_wb.torque.segment(ind[5],  nbjts[5])       =   jts_sensor_rleg.torque.segment( 0,  nbjts[5]);
    }
    else
    {
        m_jts_sensor_wb.position.segment(ind[0],  nbjts[0])     =   DEG2RAD * jts_sensor_torso.position.segment(0,  nbjts[0]);
        m_jts_sensor_wb.position.segment(ind[1],  nbjts[1])     =   DEG2RAD * jts_sensor_head.position.segment( 0,  nbjts[1]);
        m_jts_sensor_wb.position.segment(ind[2],  nbjts[2])     =   DEG2RAD * jts_sensor_larm.position.segment( 0,  nbjts[2]);
        m_jts_sensor_wb.position.segment(ind[3],  nbjts[3])     =   DEG2RAD * jts_sensor_rarm.position.segment( 0,  nbjts[3]);
        m_jts_sensor_wb.position.segment(ind[4],  nbjts[4])     =   DEG2RAD * jts_sensor_lleg.position.segment( 0,  nbjts[4]);
        m_jts_sensor_wb.position.segment(ind[5],  nbjts[5])     =   DEG2RAD * jts_sensor_rleg.position.segment( 0,  nbjts[5]);

        m_jts_sensor_wb.velocity.segment(ind[0],  nbjts[0])     =   DEG2RAD * jts_sensor_torso.velocity.segment(0,  nbjts[0]);
        m_jts_sensor_wb.velocity.segment(ind[1],  nbjts[1])     =   DEG2RAD * jts_sensor_head.velocity.segment( 0,  nbjts[1]);
        m_jts_sensor_wb.velocity.segment(ind[2],  nbjts[2])     =   DEG2RAD * jts_sensor_larm.velocity.segment( 0,  nbjts[2]);
        m_jts_sensor_wb.velocity.segment(ind[3],  nbjts[3])     =   DEG2RAD * jts_sensor_rarm.velocity.segment( 0,  nbjts[3]);
        m_jts_sensor_wb.velocity.segment(ind[4],  nbjts[4])     =   DEG2RAD * jts_sensor_lleg.velocity.segment( 0,  nbjts[4]);
        m_jts_sensor_wb.velocity.segment(ind[5],  nbjts[5])     =   DEG2RAD * jts_sensor_rleg.velocity.segment( 0,  nbjts[5]);

        m_jts_sensor_wb.acceleration.segment(ind[0],  nbjts[0]) =   DEG2RAD * jts_sensor_torso.acceleration.segment(0,  nbjts[0]);
        m_jts_sensor_wb.acceleration.segment(ind[1],  nbjts[1]) =   DEG2RAD * jts_sensor_head.acceleration.segment( 0,  nbjts[1]);
        m_jts_sensor_wb.acceleration.segment(ind[2],  nbjts[2]) =   DEG2RAD * jts_sensor_larm.acceleration.segment( 0,  nbjts[2]);
        m_jts_sensor_wb.acceleration.segment(ind[3],  nbjts[3]) =   DEG2RAD * jts_sensor_rarm.acceleration.segment( 0,  nbjts[3]);
        m_jts_sensor_wb.acceleration.segment(ind[4],  nbjts[4]) =   DEG2RAD * jts_sensor_lleg.acceleration.segment( 0,  nbjts[4]);
        m_jts_sensor_wb.acceleration.segment(ind[5],  nbjts[5]) =   DEG2RAD * jts_sensor_rleg.acceleration.segment( 0,  nbjts[5]);

        m_jts_sensor_wb.torque.segment(ind[0],  nbjts[0])       =   jts_sensor_torso.torque.segment(0,  nbjts[0]);
        m_jts_sensor_wb.torque.segment(ind[1],  nbjts[1])       =   jts_sensor_head.torque.segment( 0,  nbjts[1]);
        m_jts_sensor_wb.torque.segment(ind[2],  nbjts[2])       =   jts_sensor_larm.torque.segment( 0,  nbjts[2]);
        m_jts_sensor_wb.torque.segment(ind[3],  nbjts[3])       =   jts_sensor_rarm.torque.segment( 0,  nbjts[3]);
        m_jts_sensor_wb.torque.segment(ind[4],  nbjts[4])       =   jts_sensor_lleg.torque.segment( 0,  nbjts[4]);
        m_jts_sensor_wb.torque.segment(ind[5],  nbjts[5])       =   jts_sensor_rleg.torque.segment( 0,  nbjts[5]);
    }

    return true;
}

bool RobotInterface::getStatesEstimate(Joints_Measurements& m_jts_sensor_wb)
{
    //
    bool ok = true;
    ok = ok && this->getWholeBodyJointsMeasurements();
    ok = ok && this->setJointsValuesInRad(m_jts_sensor_wb);

    if (!ok) {
        printf("Failed to get the Whole Body states \n");
        return false;
    }
    //
    return true;
}


bool RobotInterface::setDeviceTrajectoryParameters(yarpDevice& robot_device, Eigen::VectorXd AccelParam, Eigen::VectorXd SpeedParam, int nb_jts)
{   
    //
    int numberJoints = nb_jts;

    if(numberJoints > robot_device.nb_joints) {
        // numberJoints = robot_device.nb_joints;
        printf("The number of joints provided exceed the available number of joints\n");
        return false;
    }
    // acceleration param
    if(!(AccelParam.size() == numberJoints)){
        printf("Size of the Acceleration parameters does not match the number of joints\n");
        return false;
    } 
    else {
        robot_device.ipos->setRefAccelerations(AccelParam.data());
    }
    // speed param
    if(!(SpeedParam.size() == numberJoints)){
        printf("Size of the speed parameters does not match the number of joints\n");
        return false;
    } 
    else {
        for (int i = 0; i < numberJoints; i++) {
            robot_device.ipos->setRefSpeed(i, SpeedParam[i]);
        }
    }

    return true;
}


bool RobotInterface::setWholeBodyTrajectoryParameters(double AccelParam[], double SpeedParam[])
{
    //
    int id[6], njts[6];
    //
    id[0]  = 0;
    njts[0] = 3;    njts[1] = 3;    njts[2] = 7;    njts[3] = 7;    njts[4] = 6;    njts[5] = 6;
    for(int i=1; i<6; i++) id[i] = id[i-1] + njts[i-1];

    Eigen::VectorXd AccelParam_torso(njts[0]);      Eigen::VectorXd SpeedParam_torso(njts[0]);
    Eigen::VectorXd AccelParam_head(njts[1]);       Eigen::VectorXd SpeedParam_head(njts[1]);  
    Eigen::VectorXd AccelParam_larm(njts[2]);       Eigen::VectorXd SpeedParam_larm(njts[2]); 
    Eigen::VectorXd AccelParam_rarm(njts[3]);       Eigen::VectorXd SpeedParam_rarm(njts[3]); 
    Eigen::VectorXd AccelParam_lleg(njts[4]);       Eigen::VectorXd SpeedParam_lleg(njts[4]); 
    Eigen::VectorXd AccelParam_rleg(njts[5]);       Eigen::VectorXd SpeedParam_rleg(njts[5]); 

    AccelParam_torso.setOnes();                     SpeedParam_torso.setOnes();
    AccelParam_head.setOnes();                      SpeedParam_head.setOnes();
    AccelParam_larm.setOnes();                      SpeedParam_larm.setOnes();
    AccelParam_rarm.setOnes();                      SpeedParam_rarm.setOnes();
    AccelParam_lleg.setOnes();                      SpeedParam_lleg.setOnes();
    AccelParam_rleg.setOnes();                      SpeedParam_rleg.setOnes();

    AccelParam_torso*= AccelParam[0];               SpeedParam_torso*= SpeedParam[0];                       // Torso
    AccelParam_head *= AccelParam[1];               SpeedParam_head *= SpeedParam[1];                       // Head
    AccelParam_larm *= AccelParam[2];               SpeedParam_larm *= SpeedParam[2];                       // left Hand
    AccelParam_rarm *= AccelParam[3];               SpeedParam_rarm *= SpeedParam[3];                       // right Hand
    AccelParam_lleg *= AccelParam[4];               SpeedParam_lleg *= SpeedParam[4];                       // left foot
    AccelParam_rleg *= AccelParam[5];               SpeedParam_rleg *= SpeedParam[5];                       // right foot

    bool ok=true;

    ok = ok && setDeviceTrajectoryParameters(device_torso,  AccelParam_torso, SpeedParam_torso, njts[0]);    // Torso
    ok = ok && setDeviceTrajectoryParameters( device_head,  AccelParam_head,  SpeedParam_head, njts[1]);    // Head
    ok = ok && setDeviceTrajectoryParameters( device_larm,  AccelParam_larm,  SpeedParam_larm, njts[2]);    // left Hand
    ok = ok && setDeviceTrajectoryParameters( device_rarm,  AccelParam_rarm,  SpeedParam_rarm, njts[3]);    // right Hand
    ok = ok && setDeviceTrajectoryParameters( device_lleg,  AccelParam_lleg,  SpeedParam_lleg, njts[4]);    // left foot
    ok = ok && setDeviceTrajectoryParameters( device_rleg,  AccelParam_rleg,  SpeedParam_rleg, njts[5]);    // right foot

  return true;
}


bool RobotInterface::setWholeBodyTrajectoryParameters(Eigen::VectorXd Wb_AccelParam, Eigen::VectorXd Wb_SpeedParam)
{
    //
    int id[6], njts[6];
    //
    id[0]  = 0;
    njts[0] = 3;    njts[1] = 3;    njts[2] = 7;    njts[3] = 7;    njts[4] = 6;    njts[5] = 6;
    for(int i=1; i<6; i++) id[i] = id[i-1] + njts[i];
    //
    bool ok=true;

    ok = ok && setDeviceTrajectoryParameters(device_torso, Wb_AccelParam.segment(id[0],  njts[0]), Wb_SpeedParam.segment(id[0],  njts[0]), njts[0]);   // Torso
    ok = ok && setDeviceTrajectoryParameters( device_head, Wb_AccelParam.segment(id[1],  njts[1]), Wb_SpeedParam.segment(id[1],  njts[1]), njts[1]);   // Head
    ok = ok && setDeviceTrajectoryParameters( device_larm, Wb_AccelParam.segment(id[2],  njts[2]), Wb_SpeedParam.segment(id[2],  njts[2]), njts[2]);   // left Hand
    ok = ok && setDeviceTrajectoryParameters( device_rarm, Wb_AccelParam.segment(id[3],  njts[3]), Wb_SpeedParam.segment(id[3],  njts[3]), njts[3]);   // right Hand
    ok = ok && setDeviceTrajectoryParameters( device_lleg, Wb_AccelParam.segment(id[4],  njts[4]), Wb_SpeedParam.segment(id[4],  njts[4]), njts[4]);   // left foot
    ok = ok && setDeviceTrajectoryParameters( device_rleg, Wb_AccelParam.segment(id[5],  njts[5]), Wb_SpeedParam.segment(id[5],  njts[5]), njts[5]);   // right foot

    return ok;
}

// joints limits
bool RobotInterface::getJointsLimits()
{
    
    //
    if((actuatedDofs == 29)||(actuatedDofs == 25)||(actuatedDofs == 23))
    {
        // 
        min_jtsLimits.segment(ind[0],  nbjts[0])    =   robotJtsLimits.min_torso.segment(0,  nbjts[0]);
        min_jtsLimits.segment(ind[2],  nbjts[2])    =   robotJtsLimits.min_left_arm.segment( 0,  nbjts[2]);
        min_jtsLimits.segment(ind[3],  nbjts[3])    =   robotJtsLimits.min_right_arm.segment( 0,  nbjts[3]);
        min_jtsLimits.segment(ind[4],  nbjts[4])    =   robotJtsLimits.min_left_leg.segment( 0,  nbjts[4]);
        min_jtsLimits.segment(ind[5],  nbjts[5])    =   robotJtsLimits.min_right_leg.segment( 0,  nbjts[5]);

        max_jtsLimits.segment(ind[0],  nbjts[0])    =   robotJtsLimits.max_torso.segment(0,  nbjts[0]);
        max_jtsLimits.segment(ind[2],  nbjts[2])    =   robotJtsLimits.max_left_arm.segment( 0,  nbjts[2]);
        max_jtsLimits.segment(ind[3],  nbjts[3])    =   robotJtsLimits.max_right_arm.segment( 0,  nbjts[3]);
        max_jtsLimits.segment(ind[4],  nbjts[4])    =   robotJtsLimits.max_left_leg.segment( 0,  nbjts[4]);
        max_jtsLimits.segment(ind[5],  nbjts[5])    =   robotJtsLimits.max_right_leg.segment( 0,  nbjts[5]);

        // Velocity joints limits
        velocitySaturationLimit.setOnes();                  /* actuatedDOFs */
        //
        velocitySaturationLimit.segment(ind[0],  nbjts[0])  *=   robotJtsLimits.velo_limits[0];     // Torso
        velocitySaturationLimit.segment(ind[2],  nbjts[2])  *=   robotJtsLimits.velo_limits[2];     // hand
        velocitySaturationLimit.segment(ind[3],  nbjts[3])  *=   robotJtsLimits.velo_limits[2];     // hand
        velocitySaturationLimit.segment(ind[4],  nbjts[4])  *=   robotJtsLimits.velo_limits[3];     // leg
        velocitySaturationLimit.segment(ind[5],  nbjts[5])  *=   robotJtsLimits.velo_limits[3];     // leg

        // torque joints limits
        torqueSaturationLimit.setOnes();                    /* actuatedDOFs */
        //
        torqueSaturationLimit.segment(ind[0],  nbjts[0])  *=   robotJtsLimits.torque_limits[0];     // Torso
        torqueSaturationLimit.segment(ind[2],  nbjts[2])  *=   robotJtsLimits.torque_limits[2];     // hand
        torqueSaturationLimit.segment(ind[3],  nbjts[3])  *=   robotJtsLimits.torque_limits[2];     // hand
        torqueSaturationLimit.segment(ind[4],  nbjts[4])  *=   robotJtsLimits.torque_limits[3];     // leg
        torqueSaturationLimit.segment(ind[5],  nbjts[5])  *=   robotJtsLimits.torque_limits[3];     // leg
    }
    else
    {
        // 
        min_jtsLimits.segment(ind[0],  nbjts[0])    =   robotJtsLimits.min_torso.segment(0,  nbjts[0]);
        min_jtsLimits.segment(ind[1],  nbjts[1])    =   robotJtsLimits.min_head.segment( 0,  nbjts[1]);
        min_jtsLimits.segment(ind[2],  nbjts[2])    =   robotJtsLimits.min_left_arm.segment( 0,  nbjts[2]);
        min_jtsLimits.segment(ind[3],  nbjts[3])    =   robotJtsLimits.min_right_arm.segment( 0,  nbjts[3]);
        min_jtsLimits.segment(ind[4],  nbjts[4])    =   robotJtsLimits.min_left_leg.segment( 0,  nbjts[4]);
        min_jtsLimits.segment(ind[5],  nbjts[5])    =   robotJtsLimits.min_right_leg.segment( 0,  nbjts[5]);

        max_jtsLimits.segment(ind[0],  nbjts[0])    =   robotJtsLimits.max_torso.segment(0,  nbjts[0]);
        max_jtsLimits.segment(ind[1],  nbjts[1])    =   robotJtsLimits.max_head.segment( 0,  nbjts[1]);
        max_jtsLimits.segment(ind[2],  nbjts[2])    =   robotJtsLimits.max_left_arm.segment( 0,  nbjts[2]);
        max_jtsLimits.segment(ind[3],  nbjts[3])    =   robotJtsLimits.max_right_arm.segment( 0,  nbjts[3]);
        max_jtsLimits.segment(ind[4],  nbjts[4])    =   robotJtsLimits.max_left_leg.segment( 0,  nbjts[4]);
        max_jtsLimits.segment(ind[5],  nbjts[5])    =   robotJtsLimits.max_right_leg.segment( 0,  nbjts[5]);

        // Velocity joints limits
        velocitySaturationLimit.setOnes();                  /* actuatedDOFs */
        //
        velocitySaturationLimit.segment(ind[0],  nbjts[0])  *=   robotJtsLimits.velo_limits[0];     // Torso
        velocitySaturationLimit.segment(ind[1],  nbjts[1])  *=   robotJtsLimits.velo_limits[1];     // head
        velocitySaturationLimit.segment(ind[2],  nbjts[2])  *=   robotJtsLimits.velo_limits[2];     // hand
        velocitySaturationLimit.segment(ind[3],  nbjts[3])  *=   robotJtsLimits.velo_limits[2];     // hand
        velocitySaturationLimit.segment(ind[4],  nbjts[4])  *=   robotJtsLimits.velo_limits[3];     // leg
        velocitySaturationLimit.segment(ind[5],  nbjts[5])  *=   robotJtsLimits.velo_limits[3];     // leg

        // torque joints limits
        torqueSaturationLimit.setOnes();                    /* actuatedDOFs */
        //
        torqueSaturationLimit.segment(ind[0],  nbjts[0])  *=   robotJtsLimits.torque_limits[0];     // Torso
        torqueSaturationLimit.segment(ind[1],  nbjts[1])  *=   robotJtsLimits.torque_limits[1];     // head
        torqueSaturationLimit.segment(ind[2],  nbjts[2])  *=   robotJtsLimits.torque_limits[2];     // hand
        torqueSaturationLimit.segment(ind[3],  nbjts[3])  *=   robotJtsLimits.torque_limits[2];     // hand
        torqueSaturationLimit.segment(ind[4],  nbjts[4])  *=   robotJtsLimits.torque_limits[3];     // leg
        torqueSaturationLimit.segment(ind[5],  nbjts[5])  *=   robotJtsLimits.torque_limits[3];     // leg
    }


    return true;
}

// control of the joints
// ======================
bool RobotInterface::setBodyPartControlMode(const int ctrl_mode_, yarpDevice& robotdevice,  int nb_jts)
{   
    //
    int n_joints = nb_jts;

    if(n_joints > robotdevice.nb_joints) {
        printf("The number of joints provided exceed the available number of joints\n");
        return false;
    }
    // set the mode
    robotdevice.ctrl_mode = ctrl_mode_;
    
    switch(robotdevice.ctrl_mode)
    {
        case CONTROL_MODE_POSITION :        for(int i=0; i<n_joints; i++) robotdevice.ictrl->setControlMode(i, VOCAB_CM_POSITION); break;
        case CONTROL_MODE_POSITION_DIRECT : for(int i=0; i<n_joints; i++) robotdevice.ictrl->setControlMode(i, VOCAB_CM_POSITION_DIRECT); break;
        case CONTROL_MODE_VELOCITY :        for(int i=0; i<n_joints; i++) robotdevice.ictrl->setControlMode(i, VOCAB_CM_VELOCITY);  break;
        case CONTROL_MODE_TORQUE :          for(int i=0; i<n_joints; i++) robotdevice.ictrl->setControlMode(i, VOCAB_CM_TORQUE);    break;
        default:                            for(int i=0; i<n_joints; i++) robotdevice.ictrl->setControlMode(i, VOCAB_CM_POSITION);  break;
    } 

    return true;
}


bool RobotInterface::setWholeBodyControlMode(const int ctrl_mode_)
{
    //
    setBodyPartControlMode(ctrl_mode_, device_torso, nbjts[0]);
    setBodyPartControlMode(ctrl_mode_,  device_head, nbjts[1]);
    setBodyPartControlMode(ctrl_mode_,  device_larm, nbjts[2]);
    setBodyPartControlMode(ctrl_mode_,  device_rarm, nbjts[3]);
    setBodyPartControlMode(ctrl_mode_,  device_lleg, nbjts[4]);
    setBodyPartControlMode(ctrl_mode_,  device_rleg, nbjts[5]);

    return true;
}


bool RobotInterface::setInteractionMode(const int inter_mode_, yarpDevice &robotdevice )
{
    // ste the mode
    robotdevice.inter_mode = inter_mode_;

    int n_joints;
    robotdevice.iencs->getAxes(&n_joints);

    if((robotdevice.ctrl_mode == CONTROL_MODE_POSITION) || 
       (robotdevice.ctrl_mode == CONTROL_MODE_POSITION_DIRECT) || 
       (robotdevice.ctrl_mode == CONTROL_MODE_VELOCITY) )
    {
        if(robotdevice.inter_mode == COMPLIANT_MODE){
          for(int i=0; i<n_joints; i++) robotdevice.iint->setInteractionMode(i, VOCAB_IM_COMPLIANT);
          // robotdevice.iint->setInteractionModes(VOCAB_IM_COMPLIANT);
        }else{
          for(int i=0; i<n_joints; i++) robotdevice.iint->setInteractionMode(i, VOCAB_IM_STIFF);
          // robotdevice.iint->setInteractionModes(VOCAB_IM_STIFF);
        }
    }

  return true;
}


bool RobotInterface::setWholeBodyInteractionMode(const int inter_mode_)
{
    if((actuatedDofs == 29)||(actuatedDofs == 25)||(actuatedDofs == 23))
    {
        setInteractionMode(inter_mode_, device_torso);
        setInteractionMode(inter_mode_, device_larm);
        setInteractionMode(inter_mode_, device_rarm);
        setInteractionMode(inter_mode_, device_lleg);
        setInteractionMode(inter_mode_, device_rleg);
    }
    else
    {
        setInteractionMode(inter_mode_, device_torso);
        setInteractionMode(inter_mode_, device_head);
        setInteractionMode(inter_mode_, device_larm);
        setInteractionMode(inter_mode_, device_rarm);
        setInteractionMode(inter_mode_, device_lleg);
        setInteractionMode(inter_mode_, device_rleg);
    }

  return true;
}

bool RobotInterface::setImpedance(Eigen::VectorXd Kp_vect, Eigen::VectorXd Kv_vect, yarpDevice & robotdevice )
{

  if((robotdevice.ctrl_mode == CONTROL_MODE_POSITION) || 
    (robotdevice.ctrl_mode == CONTROL_MODE_POSITION_DIRECT) || 
    (robotdevice.ctrl_mode == CONTROL_MODE_VELOCITY) )
  {
      if(robotdevice.inter_mode == COMPLIANT_MODE){
          for(int i=0; i<Kp_vect.rows(); i++) robotdevice.iimp->setImpedance(i, Kp_vect(i), Kv_vect(i));
      }
  }

  return true;
}


bool RobotInterface::setControlReferences(Eigen::VectorXd control_commands, yarpDevice& robotdevice )
{
    //
    if(robotdevice.ctrl_mode == CONTROL_MODE_TORQUE){
        control_commands = control_commands;
    }
    else{
        control_commands *= RAD2DEG;
    }

    switch(robotdevice.ctrl_mode)
    {
        case CONTROL_MODE_POSITION :        robotdevice.ipos->positionMove(control_commands.data());    break;
        case CONTROL_MODE_POSITION_DIRECT : robotdevice.iposDir->setPositions(control_commands.data()); break;
        case CONTROL_MODE_VELOCITY :        robotdevice.ivel->velocityMove(control_commands.data());    break;
        case CONTROL_MODE_TORQUE :          robotdevice.itrq->setRefTorques(control_commands.data());   break;
        default:                            robotdevice.ipos->positionMove(control_commands.data());    break;
    } 

    return true;

}

bool RobotInterface::setWholeBodyControlReferences(Eigen::VectorXd control_commands)
{
    //
    if((actuatedDofs == 29)||(actuatedDofs == 25)||(actuatedDofs == 23))
    {
        setControlReferences(control_commands.segment(ind[0],  nbjts[0]), device_torso );       // Torso
        setControlReferences(control_commands.segment(ind[2],  nbjts[2]), device_larm );        // l hand
        setControlReferences(control_commands.segment(ind[3],  nbjts[3]), device_rarm );        // r hand
        setControlReferences(control_commands.segment(ind[4],  nbjts[4]), device_lleg );        // l leg
        setControlReferences(control_commands.segment(ind[5],  nbjts[5]), device_rleg );        // r leg
    }
    else
    {
        setControlReferences(control_commands.segment(ind[0],  nbjts[0]), device_torso );       // Torso
        setControlReferences(control_commands.segment(ind[1],  nbjts[1]), device_head );        // head
        setControlReferences(control_commands.segment(ind[2],  nbjts[2]), device_larm );        // hand
        setControlReferences(control_commands.segment(ind[3],  nbjts[3]), device_rarm );        // hand
        setControlReferences(control_commands.segment(ind[4],  nbjts[4]), device_lleg );        // leg
        setControlReferences(control_commands.segment(ind[5],  nbjts[5]), device_rleg );        // leg
    }

  return true;
}

//
bool RobotInterface::setControlReference(Eigen::VectorXd control_commands, yarpDevice & robotdevice )
{
    //
    if(robotdevice.ctrl_mode == CONTROL_MODE_TORQUE){
        control_commands = control_commands;
    }
    else{
        control_commands *= RAD2DEG;
    }

    switch(robotdevice.ctrl_mode)
    {
        case CONTROL_MODE_POSITION :        for(int i=0; i<control_commands.rows(); i++) robotdevice.ipos->positionMove(i, control_commands(i));    break;
        case CONTROL_MODE_POSITION_DIRECT : for(int i=0; i<control_commands.rows(); i++) robotdevice.iposDir->setPosition(i, control_commands(i));  break;
        case CONTROL_MODE_VELOCITY :        for(int i=0; i<control_commands.rows(); i++) robotdevice.ivel->velocityMove(i, control_commands(i));    break;
        case CONTROL_MODE_TORQUE :          for(int i=0; i<control_commands.rows(); i++) robotdevice.itrq->setRefTorque(i, control_commands(i));    break;
        default:                            for(int i=0; i<control_commands.rows(); i++) robotdevice.ipos->positionMove(i, control_commands(i));    break;
    } 

  return true;

}

bool RobotInterface::setWholeBodyControlReference(Eigen::VectorXd control_commands)
{
  //
    if((actuatedDofs == 29)||(actuatedDofs == 25)||(actuatedDofs == 23))
    {
        setControlReference(control_commands.segment(ind[0],  nbjts[0]), device_torso );       // Torso
        setControlReference(control_commands.segment(ind[2],  nbjts[2]), device_larm );        // l hand
        setControlReference(control_commands.segment(ind[3],  nbjts[3]), device_rarm );        // r hand
        setControlReference(control_commands.segment(ind[4],  nbjts[4]), device_lleg );        // l leg
        setControlReference(control_commands.segment(ind[5],  nbjts[5]), device_rleg );        // r leg
    }
    else
    {
        setControlReference(control_commands.segment(ind[0],  nbjts[0]), device_torso );       // Torso
        setControlReference(control_commands.segment(ind[1],  nbjts[1]), device_head );        // head
        setControlReference(control_commands.segment(ind[2],  nbjts[2]), device_larm );        // hand
        setControlReference(control_commands.segment(ind[3],  nbjts[3]), device_rarm );        // hand
        setControlReference(control_commands.segment(ind[4],  nbjts[4]), device_lleg );        // leg
        setControlReference(control_commands.segment(ind[5],  nbjts[5]), device_rleg );        // leg
    }

  return true;
}

// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GETTING THE BASE POSE FROM EXTERNAL MEASUREMENT THROUGH PORT 
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
bool RobotInterface::SetExternalPelvisPoseEstimation(bool isBaseExternal_)
{
    isBaseExternal = isBaseExternal_;

    return true;
}

bool RobotInterface::SetPelvisPosePortName(std::string base_port_name_)
{
    Base_port_name = base_port_name_;
    return true;
}


bool RobotInterface::InitializefBasePort()
{
    // remote and local port
    std::string Remote_RootlinkPose_portName  = "/" + RobotName + "/" + this->Base_port_name;   // remote port sending the data
    std::string Local_RootlinkPose_portName   = "/" + RobotName + "/RootlinkPose_In:i";         // local port receiving the data
    //
    RootlinkPose_port_In.open(Local_RootlinkPose_portName.c_str());

    if(!yarp::os::Network::connect(Remote_RootlinkPose_portName.c_str(), RootlinkPose_port_In.getName().c_str())){
        printf(" Unable to connect to the Rootlink Port");
        return false;
    }
    // 
    RootlinkPose_values = RootlinkPose_port_In.read(); 
    RootlinkPose_measurements.resize(RootlinkPose_values->size());    

    for (int i= 0;i < RootlinkPose_values->size(); i++) RootlinkPose_measurements(i) = RootlinkPose_values->get(i).asDouble(); 
    //
    // Initialize the filter
    filter_gain_rootlink = 5.0;  // pole of the filter
    RootLinkFilter.InitializeFilter(run_period_sec, filter_gain_rootlink, filter_gain_rootlink, RootlinkPose_measurements);

    // get the first filered values and derivative
    filtered_RootlinkPose_measurements.resize(RootlinkPose_measurements.rows());
    dot_RootlinkPose_measurements.resize(RootlinkPose_measurements.rows());
    //
    filtered_RootlinkPose_measurements  = RootLinkFilter.getRK4Integral(RootlinkPose_measurements);
    dot_RootlinkPose_measurements       = filter_gain_rootlink * (RootlinkPose_measurements - filtered_RootlinkPose_measurements);

    return true;
}

void RobotInterface::get_fBase_HomoTransformation()
{
    //
    RootlinkPose_values = RootlinkPose_port_In.read(); 
    // RootlinkPose_measurements.resize(RootlinkPose_values->size());

    for (int i= 0;i < RootlinkPose_values->size(); i++)     
        RootlinkPose_measurements(i) = RootlinkPose_values->get(i).asDouble(); 
    //
    // update the quaternion
    Eigen::VectorXd qcoeff(4);
    qcoeff << RootlinkPose_measurements(6),  // w
              RootlinkPose_measurements(3),  // x
              RootlinkPose_measurements(4),  // y
              RootlinkPose_measurements(5);  // z

    // normalizing the quaternion
    qcoeff = qcoeff/qcoeff.norm();
    Eigen::Quaterniond q_root(qcoeff(0), qcoeff(1), qcoeff(2), qcoeff(3));  // w, x, y, z
    Eigen::AngleAxisd root_orient(q_root);
    // 
    Matrix3d rot;
    rot = root_orient.toRotationMatrix();
    // rot = q_root.normalized().toRotationMatrix();

    world_H_pelvis.setIdentity(); //(3,3) = 1.0;
    world_H_pelvis.block<3,1>(0,3) = RootlinkPose_measurements.head<3>();
    world_H_pelvis.block<3,3>(0,0) = rot;

    // world_H_pelvis  = world_H_Marker * Marker_H_Pelvis;
    if(this->RobotName == "icub") {
        world_H_pelvis = world_H_pelvis * Marker_H_Pelvis;
    }

}

void RobotInterface::get_EstimateBaseVelocity_Quaternion()
{
    //  filter and get the derivative of the measurements
    filtered_RootlinkPose_measurements  = RootLinkFilter.getRK4Integral(RootlinkPose_measurements);
    dot_RootlinkPose_measurements       = filter_gain_rootlink * (RootlinkPose_measurements - filtered_RootlinkPose_measurements);
    //
    Eigen::MatrixXd Omega2dqMx(4,3), psdinvOmega2dqMx(3,4);

    Omega2dqMx(0,0) = -RootlinkPose_measurements(3);  Omega2dqMx(0,1) = -RootlinkPose_measurements(4);  Omega2dqMx(0,2) = -RootlinkPose_measurements(5);  
    Omega2dqMx(1,0) =  RootlinkPose_measurements(6);  Omega2dqMx(1,1) =  RootlinkPose_measurements(5);  Omega2dqMx(1,2) = -RootlinkPose_measurements(4);
    Omega2dqMx(2,0) = -RootlinkPose_measurements(5);  Omega2dqMx(2,1) =  RootlinkPose_measurements(6);  Omega2dqMx(2,2) =  RootlinkPose_measurements(3);
    Omega2dqMx(3,0) =  RootlinkPose_measurements(4);  Omega2dqMx(3,1) = -RootlinkPose_measurements(3);  Omega2dqMx(3,2) =  RootlinkPose_measurements(6);

    psdinvOmega2dqMx = PseudoInverser.get_pseudoInverse(Omega2dqMx); 

    world_Velo_pelvis.head<3>() = dot_RootlinkPose_measurements.head<3>();
    //
    Eigen::VectorXd quat_dot(4);

    quat_dot(0) = dot_RootlinkPose_measurements(6);
    quat_dot(1) = dot_RootlinkPose_measurements(3);
    quat_dot(2) = dot_RootlinkPose_measurements(4);
    quat_dot(3) = dot_RootlinkPose_measurements(5);

    world_Velo_pelvis.tail<3>() = 2.*psdinvOmega2dqMx * quat_dot;

}


void RobotInterface::get_EstimateBaseVelocity_Euler()
{
    //  filter and get the derivative of the measurements
    filtered_RootlinkPose_measurements  = RootLinkFilter.getRK4Integral(RootlinkPose_measurements);
    dot_RootlinkPose_measurements       = filter_gain_rootlink * (RootlinkPose_measurements - filtered_RootlinkPose_measurements);

    // angular velocity Jacobian
    Eigen::Matrix3d Jac_EulerZYX;   Jac_EulerZYX.setZero();
    Jac_EulerZYX(0,0) =  cos(filtered_RootlinkPose_measurements(9)) * cos(filtered_RootlinkPose_measurements(8));
    Jac_EulerZYX(0,1) = -sin(filtered_RootlinkPose_measurements(9));
    Jac_EulerZYX(1,0) =  sin(filtered_RootlinkPose_measurements(9)) * cos(filtered_RootlinkPose_measurements(8));
    Jac_EulerZYX(1,1) =  cos(filtered_RootlinkPose_measurements(9));
    Jac_EulerZYX(2,0) = -sin(filtered_RootlinkPose_measurements(8));
    Jac_EulerZYX(2,2) =  1.0;  
    //
    // Base velocity
    world_Velo_pelvis.head<3>() = dot_RootlinkPose_measurements.head<3>();    
    world_Velo_pelvis.tail<3>() = Jac_EulerZYX * dot_RootlinkPose_measurements.tail<3>();
}

void RobotInterface::closefBasePort()
{
    RootlinkPose_port_In.close();
}


//
bool RobotInterface::setControlReferenceTorqueArmsOnly(Eigen::VectorXd control_commands)
{
  //
    if((actuatedDofs == 29)||(actuatedDofs == 25)||(actuatedDofs == 23))
    {
        setControlReference(control_commands.segment(ind[2],  nbjts[2]), device_larm );        // l hand
        setControlReference(control_commands.segment(ind[3],  nbjts[3]), device_rarm );        // r hand
    }
    else
    {
        setControlReference(control_commands.segment(ind[2],  nbjts[2]), device_larm );        // hand
        setControlReference(control_commands.segment(ind[3],  nbjts[3]), device_rarm );        // hand
    }

  return true;
}
//
bool RobotInterface::setControlReferencePositionNoArms(Eigen::VectorXd control_commands)
{
  //
    if((actuatedDofs == 29)||(actuatedDofs == 25)||(actuatedDofs == 23))
    {
        setControlReference(control_commands.segment(ind[0],  nbjts[0]), device_torso );       // Torso
        setControlReference(control_commands.segment(ind[4],  nbjts[4]), device_lleg );        // l leg
        setControlReference(control_commands.segment(ind[5],  nbjts[5]), device_rleg );        // r leg
    }
    else
    {
        setControlReference(control_commands.segment(ind[0],  nbjts[0]), device_torso );       // Torso
        setControlReference(control_commands.segment(ind[1],  nbjts[1]), device_head );        // head
        setControlReference(control_commands.segment(ind[4],  nbjts[4]), device_lleg );        // leg
        setControlReference(control_commands.segment(ind[5],  nbjts[5]), device_rleg );        // leg
    }

  return true;
}

//
bool RobotInterface::setWholeBodyPostionArmsTorqueModes(std::string direct_)
{
    //
    int ctrl_mode_all;
    int ctrl_mode_arms = CONTROL_MODE_TORQUE;
    if(direct_ == "DIRECT")
        ctrl_mode_all = CONTROL_MODE_POSITION_DIRECT;
    else
        ctrl_mode_all = CONTROL_MODE_POSITION;
    
    setBodyPartControlMode(ctrl_mode_all,  device_torso, nbjts[0]);
    setBodyPartControlMode(ctrl_mode_all,  device_head, nbjts[1]);
    setBodyPartControlMode(ctrl_mode_arms,  device_larm, nbjts[2]);
    setBodyPartControlMode(ctrl_mode_arms,  device_rarm, nbjts[3]);
    setBodyPartControlMode(ctrl_mode_all,  device_lleg, nbjts[4]);
    setBodyPartControlMode(ctrl_mode_all,  device_rleg, nbjts[5]);

    return true;
}

bool RobotInterface::setWholeBodyControlMode(int ctrl_mode_[])
{
    //
    setBodyPartControlMode(ctrl_mode_[0], device_torso, nbjts[0]);
    setBodyPartControlMode(ctrl_mode_[1],  device_head, nbjts[1]);
    setBodyPartControlMode(ctrl_mode_[2],  device_larm, nbjts[2]);
    setBodyPartControlMode(ctrl_mode_[3],  device_rarm, nbjts[3]);
    setBodyPartControlMode(ctrl_mode_[4],  device_lleg, nbjts[4]);
    setBodyPartControlMode(ctrl_mode_[5],  device_rleg, nbjts[5]);

    return true;
}


//////////////////////////////////////////  PID ////////////////////////////////////////
bool RobotInterface::getBodyJointsPIDs(yarpDevice& robot_device, Joints_PIDs& part_pid)
{
    yarp::dev::Pid *pid = new yarp::dev::Pid;
    
    bool ok = true;
    for(int i = 0; i<part_pid.nb_joints; i++) {
        ok = ok && robot_device.ipid->getPid(yarp::dev::VOCAB_PIDTYPE_POSITION, i, pid); 
        part_pid.kp(i) = pid->kp;
        part_pid.kd(i) = pid->kd;
        part_pid.ki(i) = pid->ki;
    }
    
    if (!ok) {
        printf("Failed to get the joints position PID \n");
        return false;
    }

    if(pid) {
    //     pid = NULL;
        delete pid; 
    }

    return ok;
}

bool RobotInterface::getWholeBodyJointsPIDs(yarpDevice& robot_device, Joints_PIDs& part_pid)
{
    bool ok = true;
    ok = ok && this->getBodyJointsPIDs(device_torso,  jts_pid_torso);
    ok = ok && this->getBodyJointsPIDs(device_head,   jts_pid_head);
    ok = ok && this->getBodyJointsPIDs(device_larm,   jts_pid_larm);
    ok = ok && this->getBodyJointsPIDs(device_rarm,   jts_pid_rarm);
    ok = ok && this->getBodyJointsPIDs(device_lleg,   jts_pid_lleg);
    ok = ok && this->getBodyJointsPIDs(device_rleg,   jts_pid_rleg);

    return true;
}

bool RobotInterface::setJointsPIDs(yarpDevice& robot_device, int i, double kp, double kd, double ki)
{
    yarp::dev::Pid *pid = new yarp::dev::Pid;
    
    bool ok = true;
        ok = ok && robot_device.ipid->getPid(yarp::dev::VOCAB_PIDTYPE_POSITION,i, pid);
        pid->kp = kp;
        pid->kd = kd;
        pid->ki = ki;
        ok = ok && robot_device.ipid->setPid(yarp::dev::VOCAB_PIDTYPE_POSITION,i,*pid);
    
    if (!ok) {
        printf("Failed to get the joint position PID \n");
        return false;
    }

    if(pid) {
        // pid = NULL;
        delete pid; 
    }

    return ok;
}

bool RobotInterface::setBodyJointsPIDs(yarpDevice& robot_device, Joints_PIDs part_pid)
{
    yarp::dev::Pid *pid = new yarp::dev::Pid;
   
    bool ok = true;
    for(int i = 0; i<part_pid.kp.rows(); i++) {

        ok = ok && robot_device.ipid->getPid(yarp::dev::VOCAB_PIDTYPE_POSITION,i, pid);
        pid->kp = part_pid.kp(i);
        pid->kd = part_pid.kd(i);
        pid->ki = part_pid.ki(i);
        ok = ok && robot_device.ipid->setPid(yarp::dev::VOCAB_PIDTYPE_POSITION,i,*pid);
    }
   
    if (!ok) {
        printf("Failed to get the joints position PID \n");
        return false;
    }

    if(pid) {
    //     pid = NULL;
        delete pid; 
    }

    return ok;
}

bool RobotInterface::setWholeBodyJointsPIDs(yarpDevice& robot_device, Joints_PIDs wb_pid)
{
    bool ok = true;
    //
    Joints_PIDs part_pid[6];  // [0]: torso, [1]: head, [2]: larm, [3]: rarm, [4]: lleg, [5]: rleg
    //
    if((actuatedDofs == 29)||(actuatedDofs == 25)||(actuatedDofs == 23))
    {
        for (int i = 0; i < 6; ++i){
            if(i!=1){
                part_pid[i].kp = wb_pid.kp.segment(ind[i],  nbjts[i]); 
                part_pid[i].kd = wb_pid.kd.segment(ind[i],  nbjts[i]);
                part_pid[i].ki = wb_pid.ki.segment(ind[i],  nbjts[i]);
            }
        }
        setBodyJointsPIDs(device_torso,  part_pid[0]);       // Torso
        setBodyJointsPIDs(device_larm ,  part_pid[2]);        // l hand
        setBodyJointsPIDs(device_rarm ,  part_pid[3]);        // r hand
        setBodyJointsPIDs(device_lleg ,  part_pid[4]);        // l leg
        setBodyJointsPIDs(device_rleg ,  part_pid[5]);        // r leg
    }
    else
    {
        for (int i = 0; i < 6; ++i){
                part_pid[i].kp = wb_pid.kp.segment(ind[i],  nbjts[i]); 
                part_pid[i].kd = wb_pid.kd.segment(ind[i],  nbjts[i]);
                part_pid[i].ki = wb_pid.ki.segment(ind[i],  nbjts[i]);
        }
        setBodyJointsPIDs(device_torso,  part_pid[0]);       // Torso
        setBodyJointsPIDs(device_head ,  part_pid[1]);        // head
        setBodyJointsPIDs(device_larm ,  part_pid[2]);        // l arm
        setBodyJointsPIDs(device_rarm ,  part_pid[3]);        // r arm
        setBodyJointsPIDs(device_lleg ,  part_pid[4]);        // l leg
        setBodyJointsPIDs(device_rleg ,  part_pid[5]);        // r leg
    }

    return true;
}

bool RobotInterface::setLegsJointsPIDs(Joints_PIDs legs_pid_)
{
    bool ok = true;
    //
    Joints_PIDs part_pid[2];  // [0]: lleg, [1]: rleg
    part_pid[0].resize(6);
    part_pid[1].resize(6);
    for (int i = 0; i < 2; ++i){
        part_pid[i].kp = legs_pid_.kp.segment(6*i, 6); 
        part_pid[i].kd = legs_pid_.kd.segment(6*i, 6);
        part_pid[i].ki = legs_pid_.ki.segment(6*i, 6);
    }
    //
    ok = ok && setBodyJointsPIDs(device_lleg ,  part_pid[0]);        // l leg
    ok = ok && setBodyJointsPIDs(device_rleg ,  part_pid[1]);        // r leg

    return ok;
}


bool RobotInterface::getLegsJointsPIDs(Joints_PIDs& legs_pid_)
{
    bool ok = true;
    //
    Joints_PIDs part_pid[2];  // [0]: lleg, [1]: rleg
    part_pid[0].resize(6);
    part_pid[1].resize(6);
    ok = ok && getBodyJointsPIDs(device_lleg ,  part_pid[0]);        // l leg
    ok = ok && getBodyJointsPIDs(device_rleg ,  part_pid[1]);        // r leg
    //
    legs_pid_.resize(12);
    for (int i = 0; i < 2; ++i){
        legs_pid_.kp.segment(6*i, 6) = part_pid[i].kp; 
        legs_pid_.kd.segment(6*i, 6) = part_pid[i].kd;
        legs_pid_.ki.segment(6*i, 6) = part_pid[i].ki;
    }
    //
    return ok;
}

// =============================================================================================================
void RobotInterface::OpenRobotSensors(std::string robotName, double SamplingTime)
{
    // Opening CoM and IMU ports
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // CoM_port_In.open("/CoM_In:i");
    // Network::connect("/wholeBodyDynamicsTree/com:o",CoM_port_In.getName().c_str());
    // local port
    std::string imu_in_portName = "/"+robotName+"/IMU_In:i";
    IMU_port_In.open(imu_in_portName.c_str());
    // remote port
    std::string imu_portName="/";
                imu_portName += robotName;
                imu_portName += "/inertial";         
    yarp::os::Network::connect(imu_portName.c_str(), IMU_port_In.getName().c_str());

    // Opening force/torque sensors ports
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // left
    std::string l_foot_FT_in_portName = "/"+robotName+"/l_foot_FT_readPort:i";
    l_foot_FT_inputPort.open(l_foot_FT_in_portName.c_str());
    std::string l_foot_FT_portName ="/";
                l_foot_FT_portName += robotName;
                l_foot_FT_portName += "/left_foot/analog:o";
    yarp::os::Network::connect(l_foot_FT_portName.c_str(),l_foot_FT_inputPort.getName().c_str());
       // right
    std::string r_foot_FT_in_portName = "/"+robotName+"/r_foot_FT_readPort:i";
    r_foot_FT_inputPort.open(r_foot_FT_in_portName.c_str());
    std::string r_foot_FT_portName="/";
                r_foot_FT_portName += robotName;
                r_foot_FT_portName += "/right_foot/analog:o";
    yarp::os::Network::connect(r_foot_FT_portName.c_str(),r_foot_FT_inputPort.getName().c_str());

    // Opening force/torque sensors ports
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // left
    std::string l_arm_FT_in_portName = "/"+robotName+"/l_arm_FT_readPort:i";
    l_arm_FT_inputPort.open(l_arm_FT_in_portName.c_str());
    std::string l_arm_FT_portName="/";
                l_arm_FT_portName += robotName;
                l_arm_FT_portName += "/left_arm/analog:o";
    yarp::os::Network::connect(l_arm_FT_portName.c_str(),l_arm_FT_inputPort.getName().c_str());
        // right
    std::string r_arm_FT_in_portName = "/"+robotName+"/r_arm_FT_readPort:i";
    r_arm_FT_inputPort.open(r_arm_FT_in_portName.c_str());
    std::string r_arm_FT_portName="/";
                r_arm_FT_portName += robotName;
                r_arm_FT_portName += "/right_arm/analog:o";
    yarp::os::Network::connect(r_arm_FT_portName.c_str(),r_arm_FT_inputPort.getName().c_str());

    // //
    // // Opening force/torque sensors ports
    // // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //     // left
    // std::string l_hand_FT_in_portName = "/"+robotName+"/l_hand_FT_readPort:i";
    // l_hand_FT_inputPort.open(l_hand_FT_in_portName.c_str());
    // std::string l_hand_FT_portName="/";
    //             l_hand_FT_portName += robotName;
    //             l_hand_FT_portName += "/left_hand/analog:o";
    // yarp::os::Network::connect(l_hand_FT_portName.c_str(),l_hand_FT_inputPort.getName().c_str());
    //     // right
    // std::string r_hand_FT_in_portName = "/"+robotName+"/r_hand_FT_readPort:i";
    // r_hand_FT_inputPort.open(r_hand_FT_in_portName.c_str());
    // std::string r_hand_FT_portName="/";
    //             r_hand_FT_portName += robotName;
    //             r_hand_FT_portName += "/right_hand/analog:o";
    // yarp::os::Network::connect(r_hand_FT_portName.c_str(),r_hand_FT_inputPort.getName().c_str());

    //
    yarp::os::Time::delay(0.02);
    // Reading of all created ports
    // CoM_values     = CoM_port_In.read(); 
    this->IMU_values     = IMU_port_In.read();  
    this->l_arm_FT_data  = l_arm_FT_inputPort.read();
    this->r_arm_FT_data  = r_arm_FT_inputPort.read();   
    this->l_foot_FT_data = l_foot_FT_inputPort.read();
    this->r_foot_FT_data = r_foot_FT_inputPort.read();  
    // this->l_hand_FT_data  = l_hand_FT_inputPort.read();
    // this->r_hand_FT_data  = r_hand_FT_inputPort.read();  
    //
    // Resizing of the measurement vectors    
    // CoM_measurements.resize(CoM_values->size()); 
    this->Inertial_measurements.resize(IMU_values->size());
    this->m_acceleration.resize(3);
    this->m_orientation_rpy.resize(3, 0.0);
    this->m_gyro_xyz.resize(3);

    this->l_arm_FT_vector.resize(this->l_arm_FT_data->size());
    this->r_arm_FT_vector.resize(this->r_arm_FT_data->size());
    this->l_foot_FT_vector.resize(this->l_foot_FT_data->size());
    this->r_foot_FT_vector.resize(this->r_foot_FT_data->size());
    // this->l_hand_FT_vector.resize(this->l_hand_FT_data->size());
    // this->r_hand_FT_vector.resize(this->r_hand_FT_data->size());
    for (int i=0; i<l_arm_FT_data->size();  i++) {  l_arm_FT_vector(i)  = l_arm_FT_data->get(i).asDouble();   }
    for (int i=0; i<r_arm_FT_data->size();  i++) {  r_arm_FT_vector(i)  = r_arm_FT_data->get(i).asDouble();   }
    for (int i=0; i<l_foot_FT_data->size(); i++) {  l_foot_FT_vector(i) = l_foot_FT_data->get(i).asDouble();  }
    for (int i=0; i<r_foot_FT_data->size(); i++) {  r_foot_FT_vector(i) = r_foot_FT_data->get(i).asDouble();  }

    //
    sensor_T_Gfoot = Eigen::MatrixXd::Identity(4,4);
    sensor_T_Gfoot(0,0) =  1.0;
    sensor_T_Gfoot(1,1) = -1.0;
    sensor_T_Gfoot(2,2) = -1.0;
    sensor_T_Gfoot(2,3) =  0.075;

    MatrixXd WrenchMap_ft_s;
    Trsf.Get6DWrenchTransform(sensor_T_Gfoot.block<3,3>(0,0), sensor_T_Gfoot.block<3,1>(0,3), WrenchMap_ft_s);
    WrenchMap_sensor_foot = WrenchMap_ft_s.inverse();

    // initalize the low pass filter for the force torque measurement
    // Initialize the filter
    double filter_gain_FT_feet = 5.0;
    Filter_FT_LeftFoot.InitializeFilter(SamplingTime,  filter_gain_FT_feet, filter_gain_FT_feet, l_foot_FT_vector);
    Filter_FT_RightFoot.InitializeFilter(SamplingTime, filter_gain_FT_feet, filter_gain_FT_feet, r_foot_FT_vector);

    double filter_gain_FT_arms = 5.0;   // pole of the filter
    Filter_FT_LeftArm.InitializeFilter(SamplingTime,  filter_gain_FT_arms, filter_gain_FT_arms, l_arm_FT_vector);
    Filter_FT_RightArm.InitializeFilter(SamplingTime, filter_gain_FT_arms, filter_gain_FT_arms, r_arm_FT_vector);

    // double filter_gain_FT_hands = 5.0;
    // Filter_FT_LeftHand.InitializeFilter(SamplingTime,  filter_gain_FT_hands, filter_gain_FT_hands, l_hand_FT_vector);
    // Filter_FT_RightHand.InitializeFilter(SamplingTime, filter_gain_FT_hands, filter_gain_FT_hands, r_hand_FT_vector);

    //
    this->getLeftArmForceTorqueValues();
    this->getRightArmForceTorqueValues();
    this->getLeftLegForceTorqueValues();
    this->getRightLegForceTorqueValues();

    // this->getLeftHandForceTorqueValues();
    // this->getRightHandForceTorqueValues();

    this->getFilteredLeftArmForceTorqueValues();
    this->getFilteredRightArmForceTorqueValues();
    this->getFilteredLeftLegForceTorqueValues();
    this->getFilteredRightLegForceTorqueValues();

    // this->getFilteredLeftHandForceTorqueValues();
    // this->getFilteredRightHandForceTorqueValues();
    //
    u32 com_port, baudrate;
    com_port = 0;
    baudrate = 115200;
    //
    if(robotName == "icub")
    {
        // initialize IMU
        printf("Check which tty is associated to the IMU:\n");
        printf("ls /dev/ttyACM (now press tab)\n");
        printf("sudo chmod 666 /dev/ttyACM0\n");
        printf("sudo adduser #user dialout\n");
        // u32 com_port, baudrate;
        // com_port = 0;
        // baudrate = 115200;
        imu.init(com_port, baudrate);
        // pthread_t thread1;
        // pthread_mutex_init (&mutex , NULL);
        // int i1 = pthread_create( &thread1, NULL, IMU_thread, (void*)(this));
        //
    }

}

// close the robot sensor class
void RobotInterface::CloseRobotSensors()
{
    IMU_port_In.close();
    // CoM_port_In.close();
    l_foot_FT_inputPort.close();
    r_foot_FT_inputPort.close();

    l_arm_FT_inputPort.close();
    r_arm_FT_inputPort.close();
    // l_hand_FT_inputPort.close();
    // r_hand_FT_inputPort.close();
}

yarp::sig::Vector RobotInterface::getImuOrientationValues()
{
    // Reading of inertial measurement
    this->IMU_values  = IMU_port_In.read();

    for (int i=0; i<this->IMU_values->size(); i++) {
        Inertial_measurements(i)= this->IMU_values->get(i).asDouble();
    }
    // roll pitch yaw
    this->m_orientation_rpy[0] = this->IMU_values->get(0).asDouble();
    this->m_orientation_rpy[1] = this->IMU_values->get(1).asDouble();
    this->m_orientation_rpy[2] = this->IMU_values->get(2).asDouble();

    return m_orientation_rpy;

}

Eigen::VectorXd RobotInterface::getImuAccelerometerValues()
{
    this->IMU_values      = IMU_port_In.read();
    // Resize
    //Inertial_measurements.resize(IMU_values->size());
    for (int i=0; i<this->IMU_values->size(); i++) {
        Inertial_measurements(i)= this->IMU_values->get(i).asDouble();
    }
    // accelerations 
    this->m_acceleration(0) = this->IMU_values->get(3).asDouble();
    this->m_acceleration(1) = this->IMU_values->get(4).asDouble();
    this->m_acceleration(2) = this->IMU_values->get(5).asDouble();
    
    return m_acceleration;
}

bool RobotInterface::getImuOrientationAcceleration()
{
    //
   // Reading of inertial measurement
    this->IMU_values  = IMU_port_In.read();

    for (int i=0; i<this->IMU_values->size(); i++) {
        Inertial_measurements(i)= this->IMU_values->get(i).asDouble();
    }
    // roll pitch yaw
    this->m_orientation_rpy[0] = this->IMU_values->get(0).asDouble();
    this->m_orientation_rpy[1] = this->IMU_values->get(1).asDouble();
    this->m_orientation_rpy[2] = this->IMU_values->get(2).asDouble();
     // accelerations 
    this->m_acceleration(0) = this->IMU_values->get(3).asDouble();
    this->m_acceleration(1) = this->IMU_values->get(4).asDouble();
    this->m_acceleration(2) = this->IMU_values->get(5).asDouble();


    return true;
}

Eigen::Vector3d RobotInterface::getImuGyroValues()
{
    IMU_values    = IMU_port_In.read();
    m_gyro_xyz(0) = IMU_values->get(6).asDouble();
    m_gyro_xyz(1) = IMU_values->get(7).asDouble();
    m_gyro_xyz(2) = IMU_values->get(8).asDouble();
    
    return m_gyro_xyz;
}

Vector6d RobotInterface::getLeftArmForceTorqueValues()
{
    this->l_arm_FT_data  = l_arm_FT_inputPort.read();

    for (int i=0; i<l_arm_FT_data->size(); i++) {
        this->l_arm_FT_vector(i)= this->l_arm_FT_data->get(i).asDouble();
    }
    return this->l_arm_FT_vector;
}

Vector6d RobotInterface::getRightArmForceTorqueValues()
{
    this->r_arm_FT_data  = r_arm_FT_inputPort.read();

    for (int i=0; i<this->r_arm_FT_data->size(); i++) {
        r_arm_FT_vector(i)= this->r_arm_FT_data->get(i).asDouble();
    }
    return this->r_arm_FT_vector;
}

Vector6d RobotInterface::getLeftLegForceTorqueValues()
{
    l_foot_FT_data  = l_foot_FT_inputPort.read(); 

    for (int i=0; i<l_foot_FT_data->size(); i++) {
        l_foot_FT_vector(i)= l_foot_FT_data->get(i).asDouble();
    }
    return WrenchMap_sensor_foot * l_foot_FT_vector;
}

Vector6d RobotInterface::getRightLegForceTorqueValues()
{
    r_foot_FT_data  = r_foot_FT_inputPort.read();

    for (int i=0; i<r_foot_FT_data->size(); i++) {
        r_foot_FT_vector(i)= r_foot_FT_data->get(i).asDouble();
    }
    return WrenchMap_sensor_foot * r_foot_FT_vector;
}


// //
// Vector6d RobotInterface::getLeftHandForceTorqueValues()
// {
//     this->l_hand_FT_data  = l_hand_FT_inputPort.read();

//     for (int i=0; i<l_hand_FT_data->size(); i++) {
//         this->l_hand_FT_vector(i)= this->l_hand_FT_data->get(i).asDouble();
//     }
//     return this->l_hand_FT_vector;
// }

// Vector6d RobotInterface::getRightHandForceTorqueValues()
// {
//     this->r_hand_FT_data  = r_hand_FT_inputPort.read();

//     for (int i=0; i<this->r_hand_FT_data->size(); i++) {
//         r_hand_FT_vector(i)= this->r_hand_FT_data->get(i).asDouble();
//     }
//     return this->r_hand_FT_vector;
// }
//-----------------------------------------------------------------------

Vector6d RobotInterface::getFilteredLeftArmForceTorqueValues()
{
    this->l_arm_FT_data  = l_arm_FT_inputPort.read();

    for (int i=0; i<l_arm_FT_data->size(); i++) {
        this->l_arm_FT_vector(i)= this->l_arm_FT_data->get(i).asDouble();
    }
    // Filtering
    // this->l_arm_FT_vector = this->Filter_FT_LeftArm.getEulerIntegral(this->l_arm_FT_vector);

    std::cout << " XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX         L ARM FT MEASUREMENT : \t" << l_arm_FT_vector.transpose() << endl;

    return this->l_arm_FT_vector;
}

Vector6d RobotInterface::getFilteredRightArmForceTorqueValues()
{
    this->r_arm_FT_data  = r_arm_FT_inputPort.read();

    for (int i=0; i<this->r_arm_FT_data->size(); i++) {
        r_arm_FT_vector(i)= this->r_arm_FT_data->get(i).asDouble();
    }

    std::cout << " XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX         R ARM FT MEASUREMENT : \t" << r_arm_FT_vector.transpose() << endl;
    // Filtering
    // this->r_arm_FT_vector = this->Filter_FT_RightArm.getEulerIntegral(this->r_arm_FT_vector);

    return this->r_arm_FT_vector;
}


Vector6d RobotInterface::getFilteredLeftLegForceTorqueValues()
{
    l_foot_FT_data  = l_foot_FT_inputPort.read(); 

    for (int i=0; i<l_foot_FT_data->size(); i++) {
        l_foot_FT_vector(i)= l_foot_FT_data->get(i).asDouble();
    }
    // Filtering
    // this->l_foot_FT_vector = this->Filter_FT_LeftFoot.getEulerIntegral(this->l_foot_FT_vector);
    this->l_foot_FT_vector = WrenchMap_sensor_foot * this->l_foot_FT_vector;
    
    return this->l_foot_FT_vector;
}

Vector6d RobotInterface::getFilteredRightLegForceTorqueValues()
{
    r_foot_FT_data  = r_foot_FT_inputPort.read();

    for (int i=0; i<r_foot_FT_data->size(); i++) {
        r_foot_FT_vector(i)= r_foot_FT_data->get(i).asDouble();
    }
    // Filtering
    // this->r_foot_FT_vector = this->Filter_FT_RightFoot.getEulerIntegral(this->r_foot_FT_vector);
    this->r_foot_FT_vector = WrenchMap_sensor_foot * this->r_foot_FT_vector;

    return this->r_foot_FT_vector;
}


// Vector6d RobotInterface::getFilteredLeftHandForceTorqueValues()
// {
//     this->l_hand_FT_data  = l_hand_FT_inputPort.read();

//     for (int i=0; i<l_hand_FT_data->size(); i++) {
//         this->l_hand_FT_vector(i)= this->l_hand_FT_data->get(i).asDouble();
//     }
//     // Filtering
//     this->l_hand_FT_vector = this->Filter_FT_LeftHand.getEulerIntegral(this->l_hand_FT_vector);

//     return this->l_hand_FT_vector;
// }

// Vector6d RobotInterface::getFilteredRightHandForceTorqueValues()
// {
//     this->r_hand_FT_data  = r_hand_FT_inputPort.read();

//     for (int i=0; i<this->r_hand_FT_data->size(); i++) {
//         r_hand_FT_vector(i)= this->r_hand_FT_data->get(i).asDouble();
//     }
//     // Filtering
//     this->r_hand_FT_vector = this->Filter_FT_RightHand.getEulerIntegral(this->r_hand_FT_vector);

//     return this->r_hand_FT_vector;
// }

// ======================================================================================================

void RobotInterface::get_robot_imu_measurements()
{
    // read the IMU
    // pthread_mutex_lock(&mutex);  
    i_mutex.lock();       
        imu.update(this->EULER, this->LINACC, this->ANGVEL);
        mip_interface_update(&imu.device_interface);
    i_mutex.unlock();      
    // pthread_mutex_unlock(&mutex);
    // Sleep(1);


    Vector3d rot;
    rot[0]=EULER[0]; 
    rot[1]=EULER[1]; 
    rot[2]=EULER[2]; 

    linacc[0]=LINACC[0]; 
    linacc[1]=LINACC[1]; 
    linacc[2]=LINACC[2]; 

    angvel[0]=ANGVEL[0]; 
    angvel[1]=ANGVEL[1]; 
    angvel[2]=ANGVEL[2]; 

    // make rotation matrix
    Matrix3d X = rotmatrix(Vector3d(rot[0],0,0));
    Matrix3d Y = rotmatrix(Vector3d(0,rot[1],0));
    Matrix3d Z = rotmatrix(Vector3d(0,0,rot[2]));

    // robot gives roll/pitch/yaw
    R = Z * Y * X;

    // IMU mounting on the pelvis
    Matrix3d mount = rotmatrix(Vector3d(0,M_PI/2.0,0));

    // IMU mounting, set x axis pointing front, and z up
    R = rotmatrix(Vector3d(0,M_PI,0)) * R * rotmatrix(Vector3d(0,M_PI,0));

    // and then rotate 90 deg around y
    R = R * mount;
    // local-frame angular velocity
    angvel = mount * angvel;
    // local-frame acceleration ( gives normalized g when stationary )
    linacc = mount * linacc;
    //
    std::cout<< " IMU ROBOTATION IS: \n" << R << std::endl;
    std::cout<< " IMU ANGULAR VECLOCITY IS: \n" << angvel << std::endl;
    std::cout<< " IMU LINEAR ACCEL IS: \n" << linacc << std::endl;
}

//
Matrix3d RobotInterface::rotmatrix(Vector3d a)
{
    // a: roll/pitch/yaw or XYZ Tait-Bryan angles
    // https://en.wikipedia.org/wiki/Euler_angles
    double cos1 = cos(a[0]);
    double cos2 = cos(a[1]);
    double cos3 = cos(a[2]);
    double sin1 = sin(a[0]);
    double sin2 = sin(a[1]);
    double sin3 = sin(a[2]);

    Matrix3d dircos;
    dircos(0,0) = (cos2*cos3);
    dircos(0,1) = -(cos2*sin3);
    dircos(0,2) = sin2;
    dircos(1,0) = ((cos1*sin3)+(sin1*(cos3*sin2)));
    dircos(1,1) = ((cos1*cos3)-(sin1*(sin2*sin3)));
    dircos(1,2) = -(cos2*sin1);
    dircos(2,0) = ((sin1*sin3)-(cos1*(cos3*sin2)));
    dircos(2,1) = ((cos1*(sin2*sin3))+(cos3*sin1));
    dircos(2,2) = (cos1*cos2);
    return dircos;
}

//
bool RobotInterface::OpenExternalWrenchPort(yarp::os::RpcClient &inPort, std::string port_prefix)
{
    //
    std::string portname = "/"+ port_prefix + "/applyExternalWrench/rpc:i";
    inPort.open("/"+ port_prefix + "/applyExternalWrench:o");
    yarp::os::Network::connect(inPort.getName().c_str(), portname.c_str());

    return true;
}

bool RobotInterface::CloseExternalWrenchPorts(yarp::os::RpcClient &inPort)
{
    //
    inPort.close();
    return true;
}


bool RobotInterface::applyExternalWrench(yarp::os::RpcClient &inPort, std::string link, Vector6d Wrench, double duration)
{
    //
    // yarp::os::Bottle& bot = inPort.prepare();
    yarp::os::Bottle cmd;
    yarp::os::Bottle response;

    // cmd.clear();
    cmd.addString(link);
    cmd.addDouble(Wrench[0]);
    cmd.addDouble(Wrench[1]);
    cmd.addDouble(Wrench[2]);
    cmd.addDouble(Wrench[3]);
    cmd.addDouble(Wrench[4]);
    cmd.addDouble(Wrench[5]);
    cmd.addDouble(duration);
    inPort.write(cmd, response);
    //
    return true;
}

//
bool RobotInterface::applyExternalWrench_world(yarp::os::RpcClient &inPort, std::string link, Vector6d Wrench, double duration, Matrix3d w_R_pelvis)
{
    //
    // Wrench.head(3) = w_R_pelvis * Wrench.head(3);
    // Wrench.tail(3) = w_R_pelvis * Wrench.tail(3);
    //
    this->applyExternalWrench(inPort, link, Wrench, duration);
    //
    return true;
}
//
bool RobotInterface::OpenExternalWrenchPort_1(std::string port_prefix)
{
    //
    std::string portname = "/"+ port_prefix + "/applyExternalWrench/rpc:i";
    l_hand_ExtWrench_inputPort.open("/"+ port_prefix + "/applyExternalWrench:o");
    yarp::os::Network::connect(l_hand_ExtWrench_inputPort.getName().c_str(), portname.c_str());

    return true;
}

bool RobotInterface::CloseExternalWrenchPort_1()
{
    l_hand_ExtWrench_inputPort.close();
    return true;
}

bool RobotInterface::applyExternalWrench_1(std::string link, Vector6d Wrench, double duration)
{
    //
    // yarp::os::Bottle& bot = l_hand_ExtWrench_inputPort.prepare();

    yarp::os::Bottle cmd;
    yarp::os::Bottle response;

    // cmd.clear();
    cmd.addString(link);
    cmd.addDouble(Wrench[0]);
    cmd.addDouble(Wrench[1]);
    cmd.addDouble(Wrench[2]);
    cmd.addDouble(Wrench[3]);
    cmd.addDouble(Wrench[4]);
    cmd.addDouble(Wrench[5]);
    cmd.addDouble(duration);
    l_hand_ExtWrench_inputPort.write(cmd, response);

    //
    return true;
}


