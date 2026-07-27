// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once


#ifndef RobotInterface_H
#define RobotInterface_H


#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <unistd.h> 

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <cstdlib>

// yarp headers for the joints
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/dev/IPidControl.h>

#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>

// #include <yarp/dev/IControlLimits.h>
#include <yarp/os/Vocab.h>
#include <yarp/dev/DeviceDriver.h>

#include "IMU.h"
#include <pthread.h>
#include <sys/time.h>
#include <mutex>

// #include "RobotSensors.h"
#include "wbhpidcUtilityFunctions.hpp"


typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;


// Yarp device
struct yarpDevice
{
    yarp::dev::PolyDriver          client;
    yarp::dev::IPositionControl    *ipos;
    yarp::dev::IPositionDirect     *iposDir;
    yarp::dev::IImpedanceControl   *iimp;
    yarp::dev::IControlMode2        *ictrl;
    yarp::dev::IInteractionMode    *iint;
    yarp::dev::ITorqueControl      *itrq;
    yarp::dev::IEncoders           *iencs;
    yarp::dev::IVelocityControl    *ivel;
    yarp::dev::IControlLimits      *ilim;
    yarp::dev::IPidControl		   *ipid;

    int ctrl_mode 	  = 0; 	// position mode
    int inter_mode    = 5;	// stiff

    int nb_joints;

};


// Joints limits
struct iCubJointsLimits
{
	
	// minimum joint limits
	Eigen::Matrix<double, 3,1> min_torso;
	Eigen::Matrix<double, 3,1> min_head;
	Eigen::Matrix<double, 7,1> min_left_arm;
	Eigen::Matrix<double, 7,1> min_right_arm;
	Eigen::Matrix<double, 6,1> min_left_leg;
	Eigen::Matrix<double, 6,1> min_right_leg;

	// max joints limits
	Eigen::Matrix<double, 3,1> max_torso;
	Eigen::Matrix<double, 3,1> max_head;
	Eigen::Matrix<double, 7,1> max_left_arm;
	Eigen::Matrix<double, 7,1> max_right_arm;
	Eigen::Matrix<double, 6,1> max_left_leg;
	Eigen::Matrix<double, 6,1> max_right_leg;

	double velo_limits[4];
	double torque_limits[4];

	void load_limits()
	{
		// min jts limits
		// min_torso		<< -0.349066, -0.523599, -0.872665;
		min_torso		<< -0.872665, -0.523599, -0.349066;
		min_head		<< -0.523599, -0.349066, -0.785398;
		min_left_arm	<< -1.65806,          0, -0.645772, 0.261799, -1.0472, -1.39626, -0.349066;
		min_right_arm	<< -1.65806,          0, -0.645772, 0.261799, -1.0472, -1.39626, -0.349066;
		// min_left_leg	<< -0.610865, -0.261799, -1.22173, -1.74533, -0.523599, -0.349066;
		// min_right_leg<< -0.610865, -0.261799, -1.22173, -1.74533, -0.523599, -0.349066;
		min_left_leg	<< -0.767945, -0.296706, -1.22173, -2.18166, -0.733038, -0.418879;
		min_right_leg	<< -0.767945, -0.296706, -1.22173, -2.18166, -0.733038, -0.418879;

		// max jts limits
		// max_torso		<< 1.22173,   0.523599, 0.872665;
		max_torso		<< 0.872665,   0.523599, 1.22173;
		max_head		<< 0.296706,  0.349066, 0.785398;
		max_left_arm	<< 0.174533,  2.79253,  1.39626,  1.85005,   1.0472, 0.436332, 0.436332;
		max_right_arm	<< 0.174533,  2.79253,  1.39626,  1.85005,   1.0472, 0.436332, 0.436332;
		// max_left_leg	<< 1.48353,   1.5708,   1.22173, 	    0, 0.523599, 0.349066; 
		// max_right_leg<< 1.48353,   1.5708,   1.22173, 	    0, 0.523599, 0.349066; 
		max_left_leg	<< 2.30383,   2.07694,   1.22173, 0.000000, 0.366519, 0.418879;   // knee 0.401426
		max_right_leg	<< 2.30383,   2.07694,   1.22173, 0.000000, 0.366519, 0.418879;

		// velocity saturation limits
		velo_limits[0] = M_PI/3.0;     	// Torso
	    velo_limits[1] = M_PI/4.0;      // Head
	    velo_limits[2] = M_PI/4.0;      // Hands
	    velo_limits[3] = 2.0*M_PI/3.0;  // Feets

	    // torque saturation limits
	    torque_limits[0] = 40.0;      	// Torso  24
	    torque_limits[1] =  5.0;      	// Head   10
	    torque_limits[2] = 30.0;      	// Hands  12
	    torque_limits[3] = 50.0;      	// Feets  40

	}

};

class RobotInterface
{
	protected:

		//
		std::string RobotName;
		std::string ModuleName;

		//
		double DEG2RAD;
		double RAD2DEG;
		//

	public:

		// running of the robot model period in second
		double 										run_period_sec;

		// number of considered joints per body part
		int nbjts[6];  // 0:torso	1:head 2:larm 3:rarm 4:lleg 5:rleg
		// indexes of the body's joints in the vector whole body joints  	
		int ind[6];	   // 0:torso	1:head 2:larm 3:rarm 4:lleg 5:rleg

		// number of degree of actuated Dofs
		int actuatedDofs;
		// declare a device e.g left arm
		yarpDevice device_torso;
		yarpDevice device_head;
		yarpDevice device_larm;
		yarpDevice device_rarm;
		yarpDevice device_lleg;
		yarpDevice device_rleg;

		static const int CONTROL_MODE_POSITION        = 0;
		static const int CONTROL_MODE_POSITION_DIRECT = 1;
		static const int CONTROL_MODE_VELOCITY        = 2;
		static const int CONTROL_MODE_TORQUE          = 3;

		static const int COMPLIANT_MODE               = 4;
		static const int STIFF_MODE                   = 5;

		// Joints measurements: position, velocity, acceleration, torque
		// --------------------------------------------------------------
		Joints_Measurements jts_sensor_torso;		// torso
		Joints_Measurements jts_sensor_head;		// head
		Joints_Measurements jts_sensor_larm;		// left arm
		Joints_Measurements jts_sensor_rarm;		// right arm
		Joints_Measurements jts_sensor_lleg;		// left leg
		Joints_Measurements jts_sensor_rleg;		// right leg
		// Joint pids
		// ------------
		Joints_PIDs jts_pid_torso;		// torso
		Joints_PIDs jts_pid_head;		// head
		Joints_PIDs jts_pid_larm;		// left arm
		Joints_PIDs jts_pid_rarm;		// right arm
		Joints_PIDs jts_pid_lleg;		// left leg
		Joints_PIDs jts_pid_rleg;		// right leg
		// Joints limits
		Eigen::VectorXd 	min_jtsLimits;				// joints position min	
		Eigen::VectorXd 	max_jtsLimits;				// joints position max
		Eigen::VectorXd 	velocitySaturationLimit;	// joints velocity limits
		Eigen::VectorXd 	torqueSaturationLimit;		// joints torque limits

		iCubJointsLimits  	robotJtsLimits;
		// forces torques and IMU sensors
		// RobotSensors BotSensors;
		Matrix4d sensor_T_foot;  		//  transformation from foot frame to force torque sensor frame.
		// filter
		// firstOrderFilter jtsPositionFilter;
		// firstOrderFilter jtsVelocityFilter;

		// External pelvis pose information
		// =================================

		// Port to be open
		bool 										isBaseExternal;
		std::string 								Base_port_name;
		
		yarp::os::BufferedPort<yarp::os::Bottle> 	RootlinkPose_port_In;
		yarp::os::Bottle 							*RootlinkPose_values;
		Eigen::VectorXd 							RootlinkPose_measurements;

		// Filter for the pelvis pose
		firstOrderFilter 							RootLinkFilter;
		double 										filter_gain_rootlink;

		Eigen::VectorXd 							filtered_RootlinkPose_measurements;
		Eigen::VectorXd 							dot_RootlinkPose_measurements;

		Matrix4d 									world_H_pelvis;
		Vector6d 									world_Velo_pelvis;
		Matrix4d 									Marker_H_Pelvis;    // when exteroception is used

		MatrixPseudoInverse2  PseudoInverser;       // Object to compute the pseudo inverse of a matrix


		// ==========================================================================================================================
		// ports of CoM and IMU
	    // Bottle *CoM_values;
	    yarp::os::Bottle *IMU_values;
	    // BufferedPort<Bottle> CoM_port_In;
	    yarp::os::BufferedPort<yarp::os::Bottle> IMU_port_In;
	    // Feet Force/torque sensors ports	    
	    yarp::os::BufferedPort<yarp::os::Bottle> l_foot_FT_inputPort;
	    yarp::os::BufferedPort<yarp::os::Bottle> r_foot_FT_inputPort;
	    yarp::os::BufferedPort<yarp::os::Bottle> l_arm_FT_inputPort;
	    yarp::os::BufferedPort<yarp::os::Bottle> r_arm_FT_inputPort;
	    // yarp::os::BufferedPort<yarp::os::Bottle> l_hand_FT_inputPort;
	    // yarp::os::BufferedPort<yarp::os::Bottle> r_hand_FT_inputPort;
	    yarp::os::Bottle *l_foot_FT_data;
	    yarp::os::Bottle *r_foot_FT_data;
	    yarp::os::Bottle *l_arm_FT_data;
	    yarp::os::Bottle *r_arm_FT_data;
	    // yarp::os::Bottle *l_hand_FT_data;
	    // yarp::os::Bottle *r_hand_FT_data;
	    //Inertial and CoM
	    Eigen::VectorXd   Inertial_measurements;
	    // Eigen::VectorXd   CoM_measurements;    		// as class member
	    Eigen::VectorXd   m_acceleration;
	    yarp::sig::Vector m_orientation_rpy;
	    Eigen::VectorXd   m_gyro_xyz;
	    // feet F/T measurements
	    Eigen::VectorXd l_foot_FT_vector;
	    Eigen::VectorXd r_foot_FT_vector;
        //
	    Eigen::VectorXd l_arm_FT_vector;
	    Eigen::VectorXd r_arm_FT_vector;
	    // Eigen::VectorXd l_hand_FT_vector;
	    // Eigen::VectorXd r_hand_FT_vector;
        //
        firstOrderFilter Filter_FT_LeftFoot;
        firstOrderFilter Filter_FT_RightFoot;
        firstOrderFilter Filter_FT_LeftArm;
        firstOrderFilter Filter_FT_RightArm;
        // firstOrderFilter Filter_FT_LeftHand;
        // firstOrderFilter Filter_FT_RightHand;

        // Kinematic transformations
        KineTransformations Trsf;

        //
        Matrix4d sensor_T_Gfoot;  			// homo transformation from general foot frame to FT sensor frame
        Matrix6d WrenchMap_sensor_foot;		// Wrench map from the sensor frame to the general foot frame
		// ==========================================================================================================================
		//
		


		
		RobotInterface();

		~RobotInterface();

		bool init(std::string robotName, std::string moduleName, int actuatedDofs, int period);

		// yarp devices
		bool OpenRobotDevice(yarpDevice& robot_device,  std::string body_part, Joints_Measurements& part_sensor);
		bool OpenWholeBodyDevices();
		void CloseRobotDevice(yarpDevice& robot_device);
		void CloseWholeBodyRobotDevices();
		bool getBodyJointsMeasurements(yarpDevice& robot_device, Joints_Measurements& part_sensor);
		bool getWholeBodyJointsMeasurements();
		bool setJointsValuesInRad(Joints_Measurements  &m_jts_sensor_wb);
		bool getStatesEstimate(Joints_Measurements&  m_jts_sensor_wb);
		bool setDeviceTrajectoryParameters(yarpDevice& robot_device, Eigen::VectorXd AccelParam, Eigen::VectorXd SpeedParam, int nb_jts);
		bool setWholeBodyTrajectoryParameters(double AccelParam[], double SpeedParam[]);
		bool setWholeBodyTrajectoryParameters(Eigen::VectorXd Wb_AccelParam, Eigen::VectorXd Wb_SpeedParam);

		// joints limits
		bool getJointsLimits();
		
		// control of the joints
		bool setBodyPartControlMode(const int ctrl_mode_, yarpDevice& robotdevice,  int nb_jts);
		bool setWholeBodyControlMode(const int ctrl_mode_);
		bool setInteractionMode(const int inter_mode_, yarpDevice &robotdevice );
		bool setWholeBodyInteractionMode(const int inter_mode_);
		bool setImpedance(Eigen::VectorXd Kp_vect, Eigen::VectorXd Kv_vect, yarpDevice & robotdevice );

		bool setControlReferences(Eigen::VectorXd control_commands, yarpDevice& robotdevice );
		bool setWholeBodyControlReferences(Eigen::VectorXd control_commands);
		bool setControlReference(Eigen::VectorXd control_commands, yarpDevice & robotdevice );
		bool setWholeBodyControlReference(Eigen::VectorXd control_commands);

		bool setControlReferenceTorqueArmsOnly(Eigen::VectorXd control_commands);
		bool setControlReferencePositionNoArms(Eigen::VectorXd control_commands);
		bool setWholeBodyPostionArmsTorqueModes(std::string direct_);
		bool setWholeBodyControlMode(int ctrl_mode_[]);

		//
		bool SetExternalPelvisPoseEstimation(bool isBaseExternal_);
		bool SetPelvisPosePortName(std::string base_port_name_);

		bool InitializefBasePort();
	   	void get_fBase_HomoTransformation();
	   	void get_EstimateBaseVelocity_Quaternion();
	   	void get_EstimateBaseVelocity_Euler();
	   	void closefBasePort();

	   	// PIDs
	   	bool getBodyJointsPIDs(yarpDevice& robot_device, Joints_PIDs& part_pid);
		bool getWholeBodyJointsPIDs(yarpDevice& robot_device, Joints_PIDs& part_pid);
		bool setJointsPIDs(yarpDevice& robot_device, int i, double kp, double kd, double ki);
		bool setBodyJointsPIDs(yarpDevice& robot_device, Joints_PIDs part_pid);
		bool setWholeBodyJointsPIDs(yarpDevice& robot_device, Joints_PIDs wb_pid);
		bool setLegsJointsPIDs(Joints_PIDs legs_pid_);
		bool getLegsJointsPIDs(Joints_PIDs& legs_pid_);

		// ==================================================================================================================
		// Eigen::VectorXd getCoMValues();
		void OpenRobotSensors(std::string robotName, double SamplingTime);
		void CloseRobotSensors();

	    yarp::sig::Vector getImuOrientationValues();
	    Eigen::VectorXd getImuAccelerometerValues();
        bool getImuOrientationAcceleration();
	    Eigen::Vector3d getImuGyroValues();
	    Vector6d getLeftArmForceTorqueValues();
	    Vector6d getRightArmForceTorqueValues();

	    Vector6d getLeftLegForceTorqueValues();
	    Vector6d getRightLegForceTorqueValues();

	    Vector6d getLeftHandForceTorqueValues();
	    Vector6d getRightHandForceTorqueValues();
	    // filtered value
	    Vector6d getFilteredLeftArmForceTorqueValues();
        Vector6d getFilteredRightArmForceTorqueValues();

        Vector6d getFilteredLeftLegForceTorqueValues();
        Vector6d getFilteredRightLegForceTorqueValues();

        // Vector6d getFilteredLeftHandForceTorqueValues();
        // Vector6d getFilteredRightHandForceTorqueValues();

        // =================================================================================================================
        // IMU
		pthread_mutex_t mutex;
		std::mutex i_mutex;
		IMU imu;
		float EULER[3];
		float LINACC[3];
		float ANGVEL[3];
		//
		// measured variables
		Vector3d linacc;
		Vector3d angvel;
		MatrixXd R;

		void get_robot_imu_measurements();
		Matrix3d rotmatrix(Vector3d a);


		// =================================================================================================================
		// External Wrench port
		yarp::os::RpcClient l_hand_ExtWrench_inputPort;
	    yarp::os::RpcClient r_hand_ExtWrench_inputPort;

	    bool OpenExternalWrenchPort(yarp::os::RpcClient &inPort, std::string port_prefix);
	    bool CloseExternalWrenchPorts(yarp::os::RpcClient &inPort);
	    bool applyExternalWrench(yarp::os::RpcClient &inPort, std::string link, Vector6d Wrench, double duration);
	    bool applyExternalWrench_world(yarp::os::RpcClient &inPort, std::string link, Vector6d Wrench, double duration, Matrix3d w_R_pelvis);

		bool OpenExternalWrenchPort_1(std::string port_prefix);
		bool CloseExternalWrenchPort_1();
		bool applyExternalWrench_1(std::string link, Vector6d Wrench, double duration);

};

#endif // RobotInterface_H
