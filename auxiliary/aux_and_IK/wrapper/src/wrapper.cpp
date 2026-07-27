#include "wrapper.h"
#include <iostream>

Eigen::Matrix3d wrapper::rotmatrix(Eigen::Vector3d a)
{
	// a: roll/pitch/yaw or XYZ Tait-Bryan angles
	// https://en.wikipedia.org/wiki/Euler_angles
    double cos1 = cos(a[0]);
    double cos2 = cos(a[1]);
    double cos3 = cos(a[2]);
    double sin1 = sin(a[0]);
    double sin2 = sin(a[1]);
    double sin3 = sin(a[2]);

	Eigen::Matrix3d dircos;
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

wrapper::wrapper()
{
}

wrapper::~wrapper()
{
}

int wrapper::checkRobot(int argc, char *argv[])
{
    params.fromCommand(argc, argv);
    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icubSim)\n");
        return 1;
    }
    robotName = params.find("robot").asString();
	return 0;
}

int wrapper::setupSensorConnector(sensorConnector &F, std::string moduleName, std::string analog)
{
	F.port.open(moduleName);
    yarp::os::Network::connect(analog.c_str(), F.port.getName().c_str());
	return 0;
}

int wrapper::setupJointConnector(jointConnector &C, std::string moduleName, std::string robotName, int size)
{
    std::string remotePorts = robotName + moduleName;

    std::string localPorts = "/test" + robotName + moduleName;

    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts);
    options.put("remote", remotePorts);

    C.robotDevice.open(options);
    if (!C.robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", yarp::dev::Drivers::factory().toString().c_str());
        return 1;
    }

    bool ok;
    ok = C.robotDevice.view(C.pos);
    ok = ok && C.robotDevice.view(C.encs);
    ok = ok && C.robotDevice.view(C.ictrl);
    ok = ok && C.robotDevice.view(C.iint);
    ok = ok && C.robotDevice.view(C.iimp);
    ok = ok && C.robotDevice.view(C.itrq);
    ok = ok && C.robotDevice.view(C.ipid);
    ok = ok && C.robotDevice.view(C.ilimit);
    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 1;
    }

	C.number = size;
    int nj = 0;
    C.pos->getAxes(&nj);
    //C.encoders.resize(nj);
    //C.torques.resize(nj);
    return 0;
}

int wrapper::initialize()
{
	/* Index reference and a natural posture of icub           */
	/* global positions                                        */
	/* pelvis pos:     pos[0] pos[1] pos[2]                    */
	/* pelvis rot:     pos[3] pos[4] pos[5] pos[38]            */
	/*                 // right      // left                   */
	/* head                   pos[11]                          */
	/* neck roll              pos[10]                          */
	/* neck pitch             pos[9]                           */
	/* shoulder pitch  pos[31]       pos[24]                   */
	/* shoulder roll   pos[32]       pos[25]                   */
	/* shoulder yaw    pos[33]       pos[26]                   */
	/* elbow           pos[34]       pos[27]                   */
	/* forearm yaw     pos[35]       pos[28]                   */
	/* forearm roll    pos[36]       pos[29]                   */
	/* forearm pitch   pos[37]       pos[30]                   */
	/* torso pitch            pos[6]                           */
	/* torso roll             pos[7]                           */
	/* torso yaw              pos[8]                           */
	/* hip pitch       pos[18]      pos[12]                    */
	/* hip roll        pos[19]       pos[13]                   */
	/* hip yaw         pos[20]       pos[14]                   */
	/* knee            pos[21]       pos[15]                   */
	/* ankle pitch     pos[22]       pos[16]                   */
	/* ankle roll      pos[23]       pos[17]                   */

	map = Eigen::MatrixXd(6,7);
	map <<  2 ,1 ,0 ,-1,-1,-1,-1,
			3 ,4 ,5 ,-1,-1,-1,-1,
			6 ,7 ,8 ,9 ,10,11,-1,
			12,13,14,15,16,17,-1,
			18,19,20,21,22,23,24,
			25,26,27,28,29,30,31;

    YAW = 0;
	// std::string robotName1 = "/" + robotName;
	std::string robotName1 = robotName;
	int status = 0;

    // controllers
	status |= setupJointConnector(JointPort[0], "/torso", robotName1, 3);
    status |= setupJointConnector(JointPort[1], "/head", robotName1, 3);
    status |= setupJointConnector(JointPort[2], "/left_leg", robotName1, 6);
    status |= setupJointConnector(JointPort[3], "/right_leg", robotName1, 6);
    status |= setupJointConnector(JointPort[4], "/left_arm", robotName1, 7);
    status |= setupJointConnector(JointPort[5], "/right_arm", robotName1, 7);

    // sensors
	status |= setupSensorConnector(SensorPort[0], robotName1 + "/IMU_In1:i", robotName1 + "/inertial");
	status |= setupSensorConnector(SensorPort[1], robotName1 + "/l_foot_FT_readPort1", robotName1 + "/left_foot/analog:o");
	status |= setupSensorConnector(SensorPort[2], robotName1 + "/r_foot_FT_readPort1", robotName1 + "/right_foot/analog:o");
	status |= setupSensorConnector(SensorPort[3], robotName1 + "/l_arm_FT_readPort1", robotName1 + "/left_arm/analog:o");
	status |= setupSensorConnector(SensorPort[4], robotName1 + "/r_arm_FT_readPort1", robotName1 + "/right_arm/analog:o");

    // global positions
	status |= setupSensorConnector(GlobalPosPort, robotName1 + "/RootlinkPose_In_w2:i", robotName1 + "/get_root_link_WorldPose:o");

	// ====== Opening the port to publish the Aligned CoM in position+quaternion ====== //
    std::string CoMPortName;
    CoMPortName += robotName1;
    CoMPortName += "/CoMPose:o";

    CoMPose_port_Out.open(CoMPortName.c_str());
    CoMPoses.resize(7, 0.0);


	// Gazebo clock
	status |= setupSensorConnector(TimePort, robotName1 + "/clock", "/clock");

	//external wrench
	std::string portname = robotName1 + "/applyExternalWrench/rpc:i";
	ExternalWrenchPort.port.open(robotName1 + "/applyExternalWrench:o");
    yarp::os::Network::connect(ExternalWrenchPort.port.getName().c_str(), portname.c_str());

	sens_pos = Eigen::VectorXd::Zero(32);
	sens_vel = Eigen::VectorXd::Zero(32);
	sens_acc = Eigen::VectorXd::Zero(32);
	sens_tau = Eigen::VectorXd::Zero(32);
	Root     = Eigen::VectorXd::Zero(7);
	for(int k=0;k<10;k++)
		ObjectPos[k] = Eigen::VectorXd::Zero(7);

	// time management
    double time = 0;
	// TimePort.values = TimePort.port.read();
	// if(TimePort.values == NULL)
	// 	std::cout << "WARNING: problem with getting simulation time! run gazebo with -s libgazebo_yarp_clock.so" << std::endl;
	// else
	// 	time  = TimePort.values->get(0).asDouble() + TimePort.values->get(1).asDouble()/pow(10,9);
 //    startTime = time;
	return status;
}

void wrapper::readObject(int k, std::string name)
{
	if(k<0 || k>=10)
	{
		std::cout << "WARNING: not supported object number" << std::endl;
		return;
	}
	if(!ObjectPorts[k].port.isClosed())
		ObjectPorts[k].port.close();
	std::string robotName1 = "/" + robotName;
	setupSensorConnector(ObjectPorts[k], robotName1 + "/ObjectlinkPose_In2:i", "/" + name + "/GraspedObject_WorldPose:o");
}

void wrapper::close()
{
	for(int i=0; i<6; i++)
        JointPort[i].robotDevice.close();
	for(int i=0; i<5; i++)
        SensorPort[i].port.close();
}

void wrapper::initializeJoint(Eigen::VectorXd mode)
{
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<JointPort[i].number;j++)
		{
			if(mode[map(i,j)] == 0)
			{
				JointPort[i].ictrl->setControlMode(j, VOCAB_CM_POSITION_DIRECT);
				JointPort[i].iint->setInteractionMode(j, yarp::dev::VOCAB_IM_STIFF);
			}
			else
			{
				JointPort[i].ictrl->setControlMode(j, VOCAB_CM_TORQUE);
				JointPort[i].iint->setInteractionMode(j, yarp::dev::VOCAB_IM_COMPLIANT);
			}
		}
    }
}

void wrapper::getJointLimits(double * minPosition, double * maxPosition)
{
    for(int i=0; i<6; i++)
        for(int j=0; j<JointPort[i].number;j++)
			JointPort[i].ilimit->getLimits(j, &minPosition[int(map(i,j))], &maxPosition[int(map(i,j))]);
}

void wrapper::setPidJoint(int k, double kp, double kd, double ki)
{
	yarp::dev::Pid * pid = new yarp::dev::Pid;
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<JointPort[i].number;j++)
		{
			if(map(i,j) == k)
			{
				JointPort[i].ictrl->setControlMode(j, VOCAB_CM_POSITION_DIRECT);
				JointPort[i].iint->setInteractionMode(j, yarp::dev::VOCAB_IM_STIFF);
				JointPort[i].ipid->getPid(yarp::dev::VOCAB_PIDTYPE_POSITION,j,pid);
				//printf("%d,%d: %2.2f, %2.2f, %2.2f,     %2.2f\n", i, j, pid->kp, pid->kd, pid->ki, pid->scale);
				pid->kp = kp;
				pid->kd = kd;
				pid->ki = ki;
				JointPort[i].ipid->setPid(yarp::dev::VOCAB_PIDTYPE_POSITION,j,*pid);
			}
		}
    }
}

void wrapper::controlJoint(Eigen::VectorXd mode, Eigen::VectorXd ref_pos, Eigen::VectorXd ref_tau)
{
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<JointPort[i].number;j++)
		{
			if(mode[map(i,j)] == 0)
			{
		        JointPort[i].pos->setPosition(j, ref_pos[map(i,j)]);
			}
			else
			{
		        JointPort[i].itrq->setRefTorque(j, ref_tau[map(i,j)]);
			}
		}
    }
}

int wrapper::applyExternalWrench(std::string link, Eigen::VectorXd Force, double duration)
{
	yarp::os::Bottle& bot = ExternalWrenchPort.port.prepare();
	bot.clear();
	bot.addString(link);
	bot.addDouble(Force[0]);
	bot.addDouble(Force[1]);
	bot.addDouble(Force[2]);
	bot.addDouble(Force[3]);
	bot.addDouble(Force[4]);
	bot.addDouble(Force[5]);
	bot.addDouble(duration);
	ExternalWrenchPort.port.write();
}

void wrapper::readSensors()
{
	// read IMU
	SensorPort[0].values = SensorPort[0].port.read();
	Eigen::Vector3d rot;	rot.setZero();
	for (int j=0; j<3; j++)
	{
        rot[j]    = SensorPort[0].values->get(0+j).asDouble() / 180.0 * M_PI;
		linacc[j] = SensorPort[0].values->get(3+j).asDouble();
		angvel[j] = SensorPort[0].values->get(6+j).asDouble();
	}

	// WARNING: the real robot uses a different convention
	// https://github.com/robotology/icub-gazebo/issues/18
	Eigen::Matrix3d X = rotmatrix(Eigen::Vector3d(rot[0],0,0));
	Eigen::Matrix3d Y = rotmatrix(Eigen::Vector3d(0,rot[1],0));
	Eigen::Matrix3d Z = rotmatrix(Eigen::Vector3d(0,0,rot[2]));
	R = Z * Y * X;

	// std::cout <<" BASE IN IMU BEFORE is : \n " << R << std::endl;

	// x and y directions point opposite in gazebo
	R = rotmatrix(Eigen::Vector3d(0,0,M_PI)) * R * rotmatrix(Eigen::Vector3d(0,0,-M_PI));

	// WARNING: rotations of acc and vel to be checked
	angvel = rotmatrix(Eigen::Vector3d(0,0,M_PI)) * angvel;
	linacc = rotmatrix(Eigen::Vector3d(0,0,M_PI)) * linacc;

	// read joints
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<JointPort[i].number;j++)
		{
			JointPort[i].encs->getEncoder(j, &sens_pos[map(i,j)]);
			JointPort[i].encs->getEncoderSpeed(j, &sens_vel[map(i,j)]);
			JointPort[i].encs->getEncoderAcceleration(j, &sens_acc[map(i,j)]);
			JointPort[i].itrq->getTorque(j, &sens_tau[map(i,j)]);
		}
    }

	// read contact sensors
    for(int i=1; i<5; i++)
    {
		SensorPort[i].values = SensorPort[i].port.read();
		for(int j=0; j<3; j++)
		{
			FTsensor[i-1][0][j] = SensorPort[i].values->get(j).asDouble();
			FTsensor[i-1][1][j] = SensorPort[i].values->get(j+3).asDouble();
		}
		FTsensor[i-1][0][0] *= -1;
		FTsensor[i-1][1][0] *= -1;
	}

	// reading robot position
	GlobalPosPort.values = GlobalPosPort.port.read();
	//if(!GlobalPosPort.values)
	//	std::cout << "WARNING: problem with GetLinkWorldPose!" << std::endl;
	if(GlobalPosPort.values)
		for(int i=0; i<7; i++){
		    Root[i] = GlobalPosPort.values->get(i).asDouble();
		    CoMPoses[i] = GlobalPosPort.values->get(i).asDouble();
		}

	

	// Write Global Robot Position to output port
	// yarp::sig::Vector &output_CoMPose = CoMPose_port_Out.prepare();
 //    output_CoMPose = CoMPoses;
    // CoMPose_port_Out.write();

	// reading object positions
	// for(int k=0;k<10;k++)
	// 	if(!ObjectPorts[k].port.isClosed())
	// 	{
	// 		ObjectPorts[k].values = ObjectPorts[k].port.read(false);
	// 		//if(!ObjectPorts[k].values)
	// 		//	std::cout << "WARNING: object not found!" << std::endl;
	// 		if(ObjectPorts[k].values)
	// 			for(int i=0; i<7; i++)
	// 				ObjectPos[k][i] = ObjectPorts[k].values->get(i).asDouble();
	// 	}

	// reading simulation time
 //    TimePort.values = TimePort.port.read();
	// //if(!TimePort.values)
	// //	std::cout << "WARNING: problem with getting simulation time! run gazebo with -s libgazebo_yarp_clock.so" << std::endl;
	// if(TimePort.values)
	// 	time = TimePort.values->get(0).asDouble() + TimePort.values->get(1).asDouble()/pow(10,9);
 //    if(time<startTime)
 //        startTime = time;
 //    time-=startTime;
}

