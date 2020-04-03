#include "tsid-controller.hpp"


int main(int argc, char *argv[]) {
  std::cout << "TEST JOINT LIMIT BEGIN " << std::endl;
 
  float dt = 0.001;
  auto controller = control::tsid_controller("test-controller",dt);
  controller.setInitConfiguration();
  controller.initReferences();
  float speed = 1;//left hand speed cm up per sec
  double time = 0;

  int nv = controller.robot_->nv();
  Eigen::MatrixXd qp_q_table;
  Eigen::MatrixXd qp_dq_table;
  Eigen::MatrixXd qp_ddq_table;
  Eigen::MatrixXd qp_tau_table;
  Eigen::MatrixXd qp_data_table;
  Eigen::VectorXd time_table;
  qp_q_table.resize(nv,1);
  qp_dq_table.resize(nv,1);
  qp_ddq_table.resize(nv,1);
  qp_tau_table.resize(nv,1);
  time_table.resize(1);
  qp_q_table= Eigen::MatrixXd::Zero(nv,1);
  qp_dq_table= Eigen::MatrixXd::Zero(nv,1);
  qp_ddq_table= Eigen::MatrixXd::Zero(nv,1);
  qp_tau_table= Eigen::MatrixXd::Zero(nv,1);
  time_table= Eigen::VectorXd::Zero(1);


  std::cout << "\nPress enter to ask the left hand to go up" << std::endl;
  std::cout << "\nType end followed by enter to end the test and save csv" << std::endl;
  std::string line;
  

  ///////// TO TEST JOINT vel LIMIT 
  // controller.dqmax_[17]=1;
  // controller.posVelAccBoundsTask_->setVelocityBounds(controller.dqmax_);

  ///////// TO TEST JOINT acc LIMIT 
  // controller.ddqmax_[17]=300;
  // controller.posVelAccBoundsTask_->setAccelerationBounds(controller.ddqmax_);
  controller.tsid_->addMotionTask(*controller.posVelAccBoundsTask_, controller.w_velocity_, 0);

  double maxSpeed = 0.0;

  while (std::getline(std::cin, line) &&  line.length() == 0)
  {
    std::stringstream linestr(line);
    controller.lh_ref_.translation()(0) +=  0.0;
    controller.lh_ref_.translation()(1) +=  0.0;
    controller.lh_ref_.translation()(2) +=  speed*dt;
    controller.trajLh_->setReference(controller.lh_ref_);
    controller.sampleLh_ = controller.trajLh_->computeNext();
    controller.LhTask_->setReference(controller.sampleLh_);
    controller.solve();


    qp_q_table.conservativeResize(Eigen::NoChange, qp_q_table.cols()+1);
    qp_dq_table.conservativeResize(Eigen::NoChange, qp_dq_table.cols()+1);
    qp_ddq_table.conservativeResize(Eigen::NoChange, qp_ddq_table.cols()+1);
    qp_tau_table.conservativeResize(Eigen::NoChange, qp_tau_table.cols()+1);
    time_table.conservativeResize(time_table.rows()+1);



    qp_q_table.col(qp_q_table.cols()-1) = controller.q_;
    qp_dq_table.col(qp_dq_table.cols()-1) = controller.dq_;
    qp_ddq_table.col(qp_ddq_table.cols()-1) = controller.ddq_;
    qp_tau_table.col(qp_tau_table.cols()-1) = controller.tau_;
    time_table.row(time_table.rows()-1)[0] = time;

    std::cout << "Joint 17 pos : " << controller.q_[17+6] << " vel " <<controller.dq_[17+6]<< " acc "<<controller.ddq_[17+6] << " time : "<< time <<std::endl;
    time+=dt;
  }

  time = 0;


  std::ofstream log_file;
  log_file.open ("log.csv");
  log_file << "time";
  auto names = controller.robot_->model().names;
  for(unsigned int i = 0; i < 6; i++){
    log_file << ",virtual_" + std::to_string(i)  +"_pos"  +",virtual_" + std::to_string(i)  +"_dq" + ",virtual_" + std::to_string(i)  +"_acc" + ",virtual_" + std::to_string(i)  +"_tau";
  }
  for(unsigned int i = 0; i < controller.robot_->na(); i++){
    log_file << ","<< names[i+2]+"_pos" <<","<< names[i+2]+"_vel" <<","<< names[i+2]+"_acc" <<","<< names[i+2]+"_tau";
  }
  log_file <<"\n";
  log_file.precision(16);
    for(unsigned int i=1 ; i < time_table.size() ; i++){
      log_file<<time_table(i);
      for(unsigned int j=0; j < qp_q_table.rows(); j++ ){
        log_file <<","<<qp_q_table(j,i) <<","<<qp_dq_table(j,i)<<","<<qp_ddq_table(j,i)<<","<<qp_tau_table(j,i);
    }
    log_file <<"\n";
  }
  log_file.close();

  std::cout << "Results Saved" << std::endl;
  std::cout << "TEST JOINT LIMIT END " << std::endl;
  
  return 0;
}
