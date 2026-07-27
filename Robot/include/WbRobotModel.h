// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once


#ifndef WbRobotModel_H
#define WbRobotModel_H


#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <cstdlib>


// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// Helpers function to convert between 
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>


// yarp headers for the joints
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>

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

#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

#include <pthread.h>
#include <mutex>

// #include <yarp/dev/IControlLimits.h>
#include <yarp/os/Vocab.h>
#include <yarp/dev/DeviceDriver.h>

#include "wbhpidcUtilityFunctions.hpp"
#include "RobotInterface.h"

#include "ConvexHullofPoints.hpp"


/**
 * Struct containing the floating robot state
 * using Eigen data structures.
 */
struct EigenRobotState
{
    void resize(int nrOfInternalDOFs)
    {
        jointPos.resize(nrOfInternalDOFs);
        jointVel.resize(nrOfInternalDOFs);
        //
        gravity[0] = 0;
        gravity[1] = 0;
        gravity[2] = -9.8;
    }

    void random()
    {
        world_H_base.setIdentity();
        jointPos.setRandom();
        baseVel.setRandom();
        jointVel.setRandom();
        // stance_foot = "left";

    }

    void update(//std::string 	stanceFoot, 
    			Eigen::VectorXd jointPos_, 
    			Eigen::VectorXd jointVel_, 
    			Eigen::Matrix4d world_H_base_,  
    			Eigen::Matrix<double,6,1> baseVel_)
    {
        jointPos        = jointPos_;
        jointVel        = jointVel_;
        world_H_base    = world_H_base_;
        baseVel         = baseVel_; 
        // stance_foot     = stanceFoot; 
    }

    Eigen::Matrix4d world_H_base;
    Eigen::Matrix<double,6,1> baseVel;
    Eigen::VectorXd jointPos;
    Eigen::VectorXd jointVel;
    Eigen::Vector3d gravity;
    // std::string  stance_foot;
};

/**
 * Struct containing the floating robot state
 * using iDynTree data structures.
 * For the semantics of this structures,
 * see KinDynComputation::setRobotState method.
 */
struct iDynTreeRobotState
{
    void resize(int nrOfInternalDOFs)
    {
        jointPos.resize(nrOfInternalDOFs);
        jointVel.resize(nrOfInternalDOFs);
    }

    iDynTree::Transform 		world_H_base;
    iDynTree::VectorDynSize 	jointPos;
    iDynTree::Twist         	baseVel;
    iDynTree::VectorDynSize 	jointVel;
    iDynTree::Vector3       	gravity;
};

struct EigenRobotAcceleration
{
    void resize(int nrOfInternalDOFs)
    {
        jointAcc.resize(nrOfInternalDOFs);
    }

    void random()
    {
        baseAcc.setRandom();
        jointAcc.setRandom();
    }

    Eigen::Matrix<double,6,1> baseAcc;
    Eigen::VectorXd jointAcc;
};

struct iDynTreeRobotAcceleration
{
    void resize(int nrOfInternalDOFs)
    {
        jointAcc.resize(nrOfInternalDOFs);
    }

    iDynTree::Vector6 baseAcc;
    iDynTree::VectorDynSize jointAcc;
};


//
class WbRobotModel
{
	protected:

		//
		std::string RobotName;
		std::string ModuleName;

		//
		double DEG2RAD;
		double RAD2DEG;
		// Helper class to load the model from an external format
    	iDynTree::ModelLoader mdlLoader;

		//
		iDynTree::KinDynComputations m_kinDynComp;

		//
		iDynTree::FreeFloatingGeneralizedTorques m_genTrqs;
		iDynTree::FreeFloatingGeneralizedTorques m_genGravityTrqs;
        iDynTree::FreeFloatingMassMatrix         idyn_m_massMatrix;
        iDynTree::FrameFreeFloatingJacobian      m_frameJacobian;
        iDynTree::MomentumFreeFloatingJacobian   m_momentumJacobian;
		iDynTree::MatrixDynSize 				 m_comJacobian;

		// number of degree of actuated Dofs
		int actuatedDofs;

		// Idyntree State
		iDynTreeRobotState idynRobotState;

	public:

		//
		// yarp::os::Mutex m_mutex;
		// pthread_mutex_t m_mutex;
		std::mutex m_mutex;
		// Joints measurements: position, velocity, acceleration, torque
		// -------------------------------------------------------------
		Joints_Measurements  m_jts_sensor_wb;

		// Joints limits
		Eigen::VectorXd     m_min_jtsLimits;
		Eigen::VectorXd     m_max_jtsLimits;
		Eigen::VectorXd     m_velocitySaturationLimit;
		Eigen::VectorXd     m_torqueSaturationLimit;

		// Eigen
		Eigen::MatrixXd 	MassMatrix;
		Eigen::MatrixXd 	invMassMatrix;
		Eigen::VectorXd 	Gen_bias_torques;
		Eigen::VectorXd 	Gen_Gravity_torques;
		Eigen::MatrixXd 	Jacobian;
		Eigen::VectorXd 	Bias_acceleration;
		Eigen::MatrixXd 	CoM_Jacobian;
		Eigen::VectorXd 	CoM_Bias_acceleration;

		// Eigen state
		EigenRobotState 	eigRobotState;

		// Eigen Acceleration
		EigenRobotAcceleration eigRobotAccel;
		
		// kinematic transformation 
		KineTransformations Transforms;
		// convex hull of the feet contact points
		ConvexHullofPoints ConvHull;		
		// filter
		// firstOrderFilter jtsPositionFilter;
		// firstOrderFilter jtsVelocityFilter;

		// Robot Interface for input/output communication with the robot
		// =============================================================
		RobotInterface& 		robot_interface;

        //
        // Estimation of the floating base state from fBase pose
        Vector7d            fBasePoseEstimate;
        Vector7d            filtered_fBasePoseEstimate;
        Vector7d            dot_fBasePoseEstimate;
        firstOrderFilter    fBaseFilter;
        double              filter_gain_fBase;

        MatrixPseudoInverse2  PseudoInverser;       // Object to compute the pseudo inverse of a matrix


		
		WbRobotModel(std::string robotName, RobotInterface& robot_interface_);

		~WbRobotModel();

		// bool init(std::string robotName, std::string moduleName, const iDynTree::Model & model_);
		bool init(std::string moduleName_, std::string modelFile_, std::vector<std::string> list_of_considered_joints, RobotInterface& robot_interface_);
		bool UpdateModelStates();
		bool getEstimate_BaseWorldPose(RobotInterface& robot_interface_, std::string stanceFoot, Matrix4d &W_H_B);
		bool getEstimate_BaseVelocity(RobotInterface& robot_interface_, std::string stanceFoot, Eigen::Matrix<double, 6,1> &VB);
		bool UpdateWholeBodyModel(RobotInterface& robot_interface_, std::string stanceFoot);
		bool getConfigurationStates(RobotInterface& robot_interface_);

		// compute the mass matrix
		bool computeMassMatrix(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, double * MassMatrix_);
		// Compute the Generalized bias torques (Coriolis and Gravity)
		bool computeGeneralizedBiasForces(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, double * Gen_bias_torques);
		// Compute the Generalized Gravity torques
		bool computeGravityTorques(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, double * Gen_Gravity_torques);
		// Compute the Free floating base Jacobian Matrix of a frame
		bool computeJacobian(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, iDynTree::FrameIndex arbitraryFrameIndex,  double * Jacobian);
		// Compute the Free floating base Jacobian Matrix of the Centre of mass
		bool computeCoMJacobian(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, double * CoM_Jacobian);
		// Compute the bias acceleration of a frame dJdq
		bool computeDJdq(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, iDynTree::FrameIndex arbitraryFrameIndex, double *Bias_acceleration);
		// Compute the bias acceleration of a frame dJdq of the Centre of mass
		bool computeCoMDJdq(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, double *CoM_Bias_acceleration);
		// compute the forward kinematics of end-effector 
		bool forwardKinematics(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, iDynTree::FrameIndex arbitraryFrameIndex, Eigen::Matrix<double, 7,1> &Frame_Pose, Eigen::Matrix4d &world_HmgTransf_Frame);
		// compute the forward kinematics of the center of mass (CoM)
		bool forwardKinematicsCoM(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::Matrix<double, 7,1> &CoM_Pose, Eigen::Matrix4d &world_HmgTransf_CoM);
		// compute the wrench (twist) transformation between the base and the coM
		bool compute_transform_CoM_Base(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::Matrix<double, 6,6> &CoM_X_Base);

		int getDoFs();

		bool getFrame_Index(std::string frame_name, iDynTree::FrameIndex &frameIdx);
		bool getFrameName(iDynTree::FrameIndex frameIdx, std::string &frame_name);

		bool getJointsLimits(RobotInterface& robot_interface_);
		bool getStatesEstimate();
		bool getEndEffectorsContactsStates(ContactsStates& Cont_States);
		bool getJointsStates(RobotInterface& robot_interface_, JointspaceStates& JtsStates_);
		bool setControlReference(RobotInterface& robot_interface_, Eigen::VectorXd tau_actuated);
		
		bool getMassMatrix(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::MatrixXd& mass_matrix);
		bool getInverseMassMatrix(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::MatrixXd&  inv_mass_matrix);					// compute the inverse mass matrix
		bool getGeneralizedBiasForces(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, Eigen::VectorXd& Gen_biasForces);
		bool getGeneralizedGravityForces(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, Eigen::VectorXd& Gen_gravityTorques);
		bool getJacobian(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, std::string frame_name, MatrixXdRowMaj& Jacobian_eef);
		bool getDJdq(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, std::string frame_name, Eigen::Matrix<double, 6,1> &DJacobianDq_eef);
		bool getLinkPose(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, std::string frame_name,  Eigen::Matrix<double, 7,1> &frame_pose);
		bool EstimateRobotStates(RobotInterface& robot_interface_, std::string stance_ft, Joints_Measurements& m_jts_sensor_wb_, Matrix4d &W_H_B_, Vector6d  &VB_);

		bool getEstimate_BaseWorldPose(RobotInterface& robot_interface_, Matrix4d w_H_lf, Matrix4d w_H_rf, std::string stanceFoot, Eigen::Matrix4d &W_H_B);
		bool getEstimate_BaseVelocity(RobotInterface& robot_interface_, Matrix4d w_H_lf, Matrix4d w_H_rf, std::string stanceFoot, Eigen::Matrix<double, 6,1> &VB);
		bool UpdateWholeBodyModel(RobotInterface& robot_interface_, Matrix4d w_H_lf, Matrix4d w_H_rf, std::string stanceFoot);
		bool EstimateRobotStates(RobotInterface& robot_interface_, Matrix4d w_H_lf, Matrix4d w_H_rf, std::string stance_ft, 
                                        Joints_Measurements& m_jts_sensor_wb_, Matrix4d &W_H_B_, Vector6d  &VB_);

        //
        bool UpdateWholeBodyModel_init(RobotInterface& robot_interface_, std::string stanceFoot);
        Vector7d get_fBasePoseEstimate(Matrix4d W_H_fB);
        Vector6d get_fBasePose2VeloEstimate_Quat(Matrix4d W_H_fB);


        //
        bool get_feet_support_points(MatrixXd PtsInFoot, Vector7d Pose_lfoot, Vector7d Pose_rfoot, MatrixXd &AllPtsInAbsFoot_, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot);

        bool getConvexHullVariables(MatrixXd PtsInFoot_, Vector7d Pose_lfoot, Vector7d Pose_rfoot, MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot);

};

#endif // WbRobotModel_H



