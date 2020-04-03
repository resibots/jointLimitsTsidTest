#ifndef TSID_CONTROLLER_HPP
#define TSID_CONTROLLER_HPP

/*!
 * \file tsid_controller.hpp
 * \brief C++ Interface for a controller object of type TSID
 * \author Eloïse Dalin
 * \version 0.1
 */

 /* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "Eigen/Core"
#include <iomanip>
#include <memory>
#include <utility>
#include "map"
#include "string"
#include "vector"


#include <tsid/contacts/contact-6d.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <tsid/trajectories/trajectory-se3.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/robots/fwd.hpp>
#include <tsid/robots/robot-wrapper.hpp>

using namespace tsid;
using namespace tsid::trajectories;
using namespace tsid::math;
using namespace tsid::contacts;
using namespace tsid::tasks;
using namespace tsid::solvers;
using namespace tsid::robots;
using namespace std;


// To convert quaternions to euler angles
// source https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
///// end source


namespace tsid
{
  namespace robots
  {
    RobotWrapper::RobotWrapper(const std::string & filename,
                             const std::vector<std::string> & ,
                             const pinocchio::JointModelVariant & rootJoint,
                             bool verbose)
    : m_verbose(verbose)
    {
      pinocchio::urdf::buildModel(filename, rootJoint, m_model, m_verbose);
      m_model_filename = filename;
      m_na = m_model.nv-6;
      init();
    }
  }
};

/*! \namespace control
 *  Namespace that regroup controllers
 */
namespace control {

/*! \class controller
 *  \brief Class interface that handle common function for TSID controller
 */
class tsid_controller {
 public:
  tsid_controller(const std::string& confFile,  double dt)
  {
    std::string urdf_path, srdf_path_tsid, tsid_sot, package_dir;
    urdf_path = "../urdf/talos_simplified.urdf";
    srdf_path_tsid = "../urdf/talos_laas.srdf";
    std::vector<std::string> package_dirs{package_dir};
    //JointModelFreeFlyer is floating base mode
    robot_ = std::make_shared<RobotWrapper>(urdf_path, package_dirs, pinocchio::JointModelFreeFlyer());
    dT_ = dt;
    t_ = 0.0;
    pinocchio::srdf::loadReferenceConfigurations(robot_->model(),srdf_path_tsid,false);//the srdf contains init positions
    
    vtsid_ = Vector::Zero(robot_->nv());//size of q in tsid - non actuable number of joint  for tsid (nq - 7)  
    tau_tsid_ = Vector::Zero(robot_->na());
    atsid_ = Vector::Zero(robot_->nv());
    lower_= Vector::Zero(robot_->na());
    upper_= Vector::Zero(robot_->na());
    dqmax_= Vector::Zero(robot_->na());
    ddqmax_= Vector::Zero(robot_->na());

    unsigned int size = robot_->nv();
    q_.resize(size);
    dq_.resize(size);
    ddq_.resize(size);
    tau_.resize(size);
    q_.setZero(q_.size());
    dq_.setZero(dq_.size());
    ddq_.setZero(ddq_.size());
    tau_.setZero(tau_.size());
  }

  ~tsid_controller(){};

  
  void setInitConfiguration( ) 
  {
    ////////////////////Gather Initial Pose //////////////////////////////////////
    // qtsid_ = robot_->model().referenceConfigurations["half_sitting"];
    qtsid_ = robot_->model().referenceConfigurations["pal_start"];

    ////////////////////Create the inverse-dynamics formulation///////////////////
    tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);

    ////////////////////Create an HQP solver /////////////////////////////////////
    solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST, "solver-eiquadprog");
    solver_->resize(tsid_->nVar(), tsid_->nEq(), tsid_->nIn());
    // To be faster use the following and comment the 2 previous lines :
    // solver_ =  SolverHQPFactory::createNewSolver<62,18,34>(SOLVER_HQP_EIQUADPROG_RT, "eiquadprog-rt");

    ////////////////////Compute Problem Data at init /////////////////////////////
    const unsigned int nv = robot_->nv();
    Vector v = Vector::Zero(nv);
    tsid_->computeProblemData(dT_, qtsid_, v);
    const pinocchio::Data & data = tsid_->data();

    ////////////////////Compute Tasks, Bounds and Contacts ///////////////////////
    // Add the contact constraints
    Matrix3x contactPoints_(3,4);
    contactPoints_ << -lxn_, -lxn_, +lxp_, +lxp_,
                    -lyn_, +lyp_, -lyn_, +lyp_,
                      lz_,  lz_,  lz_,  lz_;

    contactRF_ = std::make_shared<Contact6d>("contact_rfoot", *robot_, rf_frame_name_,
                              contactPoints_, contactNormal_,
                              mu_, fMin_, fMax_);
    contactRF_->Kp(kp_contact_*Vector::Ones(6));
    contactRF_->Kd(2.0*contactRF_->Kp().cwiseSqrt());
    H_rf_ref_ = robot_->position(data, robot_->model().getJointId(rf_frame_name_));
    contactRF_->setReference(H_rf_ref_);
    tsid_->addRigidContact(*contactRF_, w_forceRef_feet_);

    contactLF_ = std::make_shared<Contact6d>("contact_lfoot", *robot_, lf_frame_name_,
                              contactPoints_, contactNormal_,
                              mu_, fMin_, fMax_);
    contactLF_->Kp(kp_contact_*Vector::Ones(6));
    contactLF_->Kd(2.0*contactLF_->Kp().cwiseSqrt());
    H_lf_ref_ = robot_->position(data, robot_->model().getJointId(lf_frame_name_));
    contactLF_->setReference(H_lf_ref_);
    tsid_->addRigidContact(*contactLF_, w_forceRef_feet_);


    // Add the com task
    comTask_ = std::make_shared<TaskComEquality>("task-com", *robot_);
    comTask_->Kp(kp_com_*Vector::Ones(3));
    comTask_->Kd(2.0*comTask_->Kp().cwiseSqrt());
    tsid_->addMotionTask(*comTask_, w_com_, 1);

    // Add the posture task
    postureTask_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
    postureTask_->Kp(kp_posture_*Vector::Ones(nv-6));
    postureTask_->Kd(2.0*postureTask_->Kp().cwiseSqrt());
    Vector maskPost = Vector::Ones(nv-6);
    // for(int i =0; i < 11; i++){
    //   maskPost[i] = 0;
    // }
    postureTask_->setMask(maskPost);
    tsid_->addMotionTask(*postureTask_, w_posture_, 1);

    // Add the waist task
    waistTask_ = std::make_shared<TaskSE3Equality>("task-waist", *robot_, "root_joint");
    waistTask_->Kp(kp_waist_*Vector::Ones(6));
    waistTask_->Kd(2.0*waistTask_->Kp().cwiseSqrt());
    Vector maskWaist = Vector::Ones(6);
    for(int i =0; i < 3; i++){
      maskWaist[i] = 0;// DO NOT CONSTRAIN WAIST POSITION IN SOT
    }
    waistTask_->setMask(maskWaist);
    tsid_->addMotionTask(*waistTask_, w_waist_, 1);

    // Add the left hand  task
    LhTask_ = std::make_shared<TaskSE3Equality>("task-lh", *robot_, "arm_left_6_joint");
    LhTask_->Kp(kp_lh_*Vector::Ones(6));
    LhTask_->Kd(2.0*LhTask_->Kp().cwiseSqrt());
    Vector maskLh = Vector::Ones(6);
    for(int i = 3; i < 6; i++){
      maskLh[i] = 0;// DO NOT CONSTRAIN HAND ORIENTATION IN SOT
    }
    LhTask_->setMask(maskLh);
    tsid_->addMotionTask(*LhTask_, w_lh_, 1);


    // Add the right hand task
    RhTask_ = std::make_shared<TaskSE3Equality>("task-rh", *robot_, "arm_right_6_joint");
    RhTask_->Kp(kp_rh_*Vector::Ones(6));
    RhTask_->Kd(2.0*RhTask_->Kp().cwiseSqrt());
    Vector maskRh = Vector::Ones(6);
    for(int i =3; i < 6; i++){
      maskRh[i] = 0;// DO NOT CONSTRAIN HAND ORIENTATION IN SOT
    }
    RhTask_->setMask(maskRh);
    tsid_->addMotionTask(*RhTask_, w_rh_, 1);

    // Add the left foot  task
    LfTask_ = std::make_shared<TaskSE3Equality>("task-lf", *robot_, "leg_left_6_joint");
    LfTask_->Kp(kp_lf_*Vector::Ones(6));
    LfTask_->Kd(2.0*LfTask_->Kp().cwiseSqrt());
    Vector maskLf = Vector::Ones(6);
    LfTask_->setMask(maskLf);
    tsid_->addMotionTask(*LfTask_, w_lf_, 1);


    // Add the right foot  task
    RfTask_ = std::make_shared<TaskSE3Equality>("task-rf", *robot_, "leg_right_6_joint");
    RfTask_->Kp(kp_rf_*Vector::Ones(6));
    RfTask_->Kd(2.0*RfTask_->Kp().cwiseSqrt());
    Vector maskRf = Vector::Ones(6);
    RfTask_->setMask(maskRf);
    tsid_->addMotionTask(*RfTask_, w_rf_, 1);



    posVelAccBoundsTask_= std::make_shared<TaskJointPosVelAccBounds>("task-posVelAcc-bounds", *robot_, dT_);
    dqmax_ = robot_->model().velocityLimit.tail(robot_->na());
    ddqmax_ = dqmax_/dT_;
    posVelAccBoundsTask_->setVelocityBounds(dqmax_);
    posVelAccBoundsTask_->setAccelerationBounds(ddqmax_);
    lower_ = robot_->model().lowerPositionLimit.tail(robot_->na());
    upper_ = robot_->model().upperPositionLimit.tail(robot_->na());

    lower_[17]=-1.5;
    upper_[17]=-1.4;
    std::cout << "Lower position limit joint 17 (" << robot_->model().names[17+2] << ") " << lower_[17] << std::endl;
    std::cout << "Upper position limit joint 17 (" << robot_->model().names[17+2] << ") " <<  upper_[17] << std::endl;
    std::cout << "Max   velocity limit joint 17 (" << robot_->model().names[17+2] << ") " << dqmax_[17] << std::endl;
    std::cout << "Max   acceleration limit joint 17 (" << robot_->model().names[17+2] << ") " <<  ddqmax_[17] << std::endl;
    posVelAccBoundsTask_->setPositionBounds(lower_ , upper_);

    // posVelAccBoundsTask_
    // tsid_->addMotionTask(*posVelAccBoundsTask_, w_velocity_, 0);

    // Add the Velocity bounds
    // velocityBoundsTask_ = std::make_shared<TaskJointBounds>("task-velocity-bounds", *robot_, dT_);
    // velocityBoundsTask_->setVelocityBounds(-robot_->model().velocityLimit.tail(robot_->na()), robot_->model().velocityLimit.tail(robot_->na()));
    // velocityBoundsTask_->setAccelerationBounds(low,up); //possible to add this
    // tsid_->addMotionTask(*velocityBoundsTask_, w_velocity_, 0); // 0.0 put this constraint as a hard constraint

    // Add the torque bounds
    // torqueBoundsTask_ = std::make_shared<TaskActuationBounds>("task-torque-bounds", *robot_);
    // torqueBoundsTask_->setBounds(-robot_->model().effortLimit.tail(robot_->na()), robot_->model().effortLimit.tail(robot_->na()));
    // tsid_->addActuationTask(*torqueBoundsTask_, w_torque_, 0); // 0.0 put this constraint as a hard constraint
  }

  void initReferences()
  {
    sampleCom_.resize(3);
    samplePosture_.resize(robot_->na());
    sampleWaist_.resize(3);
    sampleLf_.resize(6);
    sampleRf_.resize(6);
    sampleLh_.resize(3);
    sampleRh_.resize(3);


    com_init_ = robot_->com(tsid_->data());
    posture_init_ = qtsid_.tail(robot_->na());
    waist_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("root_joint"));
    lf_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("leg_left_6_joint"));
    rf_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("leg_right_6_joint"));
    lh_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("arm_left_6_joint"));
    rh_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("arm_right_6_joint"));


    com_ref_ = com_init_;
    posture_ref_ = posture_init_;
    waist_ref_ = waist_init_;
    lf_ref_ = lf_init_;
    rf_ref_ = rf_init_;
    lh_ref_ = lh_init_;
    rh_ref_ = rh_init_;


    trajCom_ = std::make_shared<TrajectoryEuclidianConstant>("traj_com", com_ref_);
    trajPosture_ = std::make_shared<TrajectoryEuclidianConstant>("traj_posture", posture_ref_);
    trajWaist_ = std::make_shared<TrajectorySE3Constant>("traj_waist", waist_ref_);
    trajLf_ = std::make_shared<TrajectorySE3Constant>("traj_lf", lf_ref_);
    trajRf_ = std::make_shared<TrajectorySE3Constant>("traj_rf", rf_ref_);
    trajLh_ = std::make_shared<TrajectorySE3Constant>("traj_lh", lh_ref_);
    trajRh_ = std::make_shared<TrajectorySE3Constant>("traj_rh", rh_ref_);


    sampleCom_=trajCom_->computeNext();
    samplePosture_=trajPosture_->computeNext();
    sampleWaist_=trajWaist_->computeNext();
    sampleLf_=trajLf_->computeNext();
    sampleRf_=trajRf_->computeNext();
    sampleLh_=trajLh_->computeNext();
    sampleRh_=trajRh_->computeNext();

    comTask_->setReference(sampleCom_);
    postureTask_->setReference(samplePosture_);
    waistTask_->setReference(sampleWaist_);
    LfTask_->setReference(sampleLf_);
    RfTask_->setReference(sampleRf_);
    LhTask_->setReference(sampleLh_);
    RhTask_->setReference(sampleRh_);
  }

  bool solve() 
  {
    //Compute the current data from the current position and solve to find next position
    const HQPData & HQPData = tsid_->computeProblemData(t_, qtsid_, vtsid_);
    const HQPOutput & sol = solver_->solve(HQPData);
    if(sol.status==HQP_STATUS_OPTIMAL)
    {
      const Vector & tau = tsid_->getActuatorForces(sol);
      const Vector & dv = tsid_->getAccelerations(sol);
      tau_tsid_ = tau;
      atsid_ = dv;
      vtsid_ += dT_*dv;
      qtsid_ = pinocchio::integrate(robot_->model(), qtsid_, dT_*vtsid_);
      t_ += dT_;

      //Convert quaternion to euler to form q_ for dart
      Quaternion quat = {.w =qtsid_(6), .x = qtsid_(3), .y = qtsid_(4), .z = qtsid_(5) };
      EulerAngles eulerA = ToEulerAngles(quat);
      q_ << qtsid_.head(3), eulerA.roll , eulerA.pitch , eulerA.yaw ,qtsid_.tail(robot_->nq()-7) ;//nq size 37 (pos+quat+nactuated)
      dq_ = vtsid_;//the speed of the free flyerjoint is dim 6 even if its pos id dim 7
      tau_<< 0, 0 ,0, 0 , 0, 0, tau_tsid_;//the size of tau is actually 30 (nactuated)
      ddq_ = atsid_;

      return true;

    }
    else{
      std::cout << "Controller failed, can't solve problem " << std::endl;
      std::cout << "Status "+toString(sol.status) << std::endl;
      return false;
    }
  }

 public:
  // TALOS CONFIG
  double lxp_ = 0.1;    //foot length in positive x direction
  double lxn_ = 0.11;   //foot length in negative x direction
  double lyp_ = 0.069;  //foot length in positive y direction
  double lyn_ = 0.069;  //foot length in negative y direction
  double lz_ = 0.107;   //foot sole height with respect to ankle joint
  double mu_ = 0.3;     //friction coefficient
  double fMin_ = 5.0; //minimum normal force
  double fMax_ = 1500.0;//maximum normal force
  std::string rf_frame_name_= "leg_right_6_joint"; // right foot joint name
  std::string lf_frame_name_ = "leg_left_6_joint"; // left foot joint name
  Vector3 contactNormal_ = Vector3::UnitZ(); // direction of the normal to the contact surface
  double w_com_ = 1.0; //  weight of center of mass task
  double w_posture_ = 0.75;//  weight of joint posture task
  double w_forceRef_feet_ = 1e-3;//# weight of force regularization task
  double w_forceRef_hands_ = 1e-3;//# weight of force regularization task
  double w_waist_ = 20.0;//# weight of waist task
  double w_velocity_ = 1.0;//# weight of velocity bounds
  double w_torque_ = 1.0;//# weight of velocity bounds
  double w_rh_ = 10.0;//# weight of waist task
  double w_lh_ = 10.0;//# weight of waist task
  double w_rf_ = 1.0;//# weight of waist task
  double w_lf_ = 1.0;//# weight of waist task
  double kp_contact_ = 30.0;//# proportional gain of contact constraint
  double kp_com_ = 3000.0;//# proportional gain of center of mass task
  double kp_posture_ = 30.0;//# proportional gain of joint posture task
  double kp_waist_ = 3000.0;//# proportional gain of waist task
  double kp_rh_ = 300.0;//# proportional gain of right hand task
  double kp_lh_ = 300.0;//# proportional gain of left hand task
  double kp_rf_ = 30.0;//# proportional gain of right foot task
  double kp_lf_ = 30.0;//# proportional gain of left foot task


  double dT_ = 0.001;


  Matrix3x contactPoints_;

  std::shared_ptr<RobotWrapper>  robot_;
  

  std::shared_ptr<InverseDynamicsFormulationAccForce>  tsid_;
  SolverHQPBase * solver_;
  pinocchio::SE3  H_rf_ref_, H_lf_ref_;

  std::vector<std::string> jNames_;
  Vector qvjoints_;
  Vector qzeros_;

  double t_;

  Vector lower_;
  Vector upper_;
  Vector dqmax_;
  Vector ddqmax_;

  Vector qtsid_;
  Vector vtsid_;
  Vector atsid_;
  Vector tau_tsid_;
    
  // Positions resized
  Eigen::VectorXd q_;
  // Velocities resized
  Eigen::VectorXd dq_;
  // Torques resized
  Eigen::VectorXd tau_;
  // Accelerations resized
  Eigen::VectorXd ddq_;

  std::shared_ptr<TaskComEquality> comTask_;
  std::shared_ptr<TaskJointPosture> postureTask_;
  std::shared_ptr<TaskSE3Equality> waistTask_;

  std::shared_ptr<TaskSE3Equality> LfTask_;
  std::shared_ptr<TaskSE3Equality> RfTask_;
  std::shared_ptr<TaskSE3Equality> LhTask_;
  std::shared_ptr<TaskSE3Equality> RhTask_;


  std::shared_ptr<TaskJointBounds> velocityBoundsTask_;
  std::shared_ptr<TaskJointPosVelAccBounds> posVelAccBoundsTask_;
  std::shared_ptr<TaskActuationBounds> torqueBoundsTask_;

  std::shared_ptr<Contact6d> contactRF_;
  std::shared_ptr<Contact6d> contactLF_;
  std::shared_ptr<Contact6d> contactRH_;
  std::shared_ptr<Contact6d> contactLH_;

  Vector3 com_init_, com_ref_;
  std::shared_ptr<TrajectoryEuclidianConstant> trajCom_;
  TrajectorySample sampleCom_;

  Vector posture_init_, posture_ref_;
  std::shared_ptr<TrajectoryBase> trajPosture_;
  TrajectorySample samplePosture_;

  pinocchio::SE3  waist_init_, waist_ref_;
  std::shared_ptr<TrajectorySE3Constant> trajWaist_;
  TrajectorySample sampleWaist_;

  // Left Foot
  pinocchio::SE3  lf_init_, lf_ref_;
  std::shared_ptr<TrajectorySE3Constant> trajLf_;
  TrajectorySample sampleLf_;

  // Right Foot
  pinocchio::SE3  rf_init_, rf_ref_;
  std::shared_ptr<TrajectorySE3Constant> trajRf_;
  TrajectorySample sampleRf_;


  // Left Hand
  pinocchio::SE3  lh_init_, lh_ref_;
  std::shared_ptr<TrajectorySE3Constant> trajLh_;
  TrajectorySample sampleLh_;

  // Right Hand
  pinocchio::SE3  rh_init_, rh_ref_;
  std::shared_ptr<TrajectorySE3Constant> trajRh_;
  TrajectorySample sampleRh_;
};

}  // namespace control
#endif  
