#include "MyFirstController.h"
#include <RBDyn/MultiBodyConfig.h>
#include <SpaceVecAlg/PTransform.h>
#include <iostream>
#include <iterator>
#include <map>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PositionTask.h>
#include <mc_tasks/PostureTask.h>
#include <memory>
#include <mc_control/GlobalPluginMacros.h>
#include <ostream>
#include <string>


MyFirstController::MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt, config, Backend::TVM)
{
  postureTarget = {
    {"RCY", {-8.3654e-06}}, {"RCR", {0.000719231}}, {"RCP", {-0.468564}}, {"RKP", {0.874865}}, {"RAP", {-0.40261}}, {"RAR", {-7.29572e-05}},
    {"LCY", {8.47109e-06}}, {"LCR", {-0.000730919}}, {"LCP", {-0.46857}}, {"LKP", {0.874877}}, {"LAP", {-0.402613}}, {"LAR", {4.78505e-05}},
    {"WP", {-0.00530605}}, {"WR", {0.000145301}}, {"WY", {-5.42326e-07}}, {"HY", {-1.15688e-06}}, {"HP", {0.000168412}},
    {"RSC", {8.69819e-05}}, {"RSP", {1.04475}}, {"RSR", {0.34792}}, {"RSY", {0.0834733}}, {"REP", {-1.83161}}, {"RWRY", {0.000507872}}, {"RWRR", {0.697132}}, {"RWRP", {0.00133644}}, {"RHDY", {6.05043e-05}},
    {"LSC", {-8.61476e-05}}, {"LSP", {1.04475}}, {"LSR", {-0.347921}}, {"LSY", {-0.0834775}}, {"LEP", {-1.83161}}, {"LWRY", {-0.000507875}}, {"LWRR", {-0.697133}}, {"LWRP", {0.00133666}}, {"LHDY", {-6.04975e-05}},
  };


  // jointNames = {
  //   {"RCY", 0}, {"RCR", 1}, {"RCP", 2}, {"RKP", 3},{"RAP", 4},{"RAR", 5}, {"LCY", 6}, {"LCR", 7},{"LCP", 8}, {"LKP", 9},{"LAP", 10},{"LAR", 11},{"WP", 12}, {"WR", 13}, {"WY", 14},
  //   {"HY", 15},{"HP", 16}, {"RSC", 17}, {"RSP", 18}, {"RSR", 19}, {"RSY", 20}, {"REP", 21}, {"RWRY", 22}, {"RWRR", 23}, {"RWRP", 24}, {"RHDY", 25}, {"RTMP", 26}, {"RTPIP", 27},
  //   {"RTDIP", 28}, {"RIMP", 29}, {"RIPIP", 30}, {"RIDIP", 31}, {"RMMP", 32}, {"RMPIP", 33}, {"RMDIP", 34},
  //   {"LSC", 35}, {"LSP", 36}, {"LSR", 37}, {"LSY", 38}, {"LEP", 39}, {"LWRY", 40}, {"LWRR", 41}, {"LWRP", 42}, {"LHDY", 43}, {"LTMP", 44}, {"LTPIP", 45},
  //   {"LTDIP", 46}, {"LIMP", 47}, {"LIPIP", 48}, {"LIDIP", 49}, {"LMMP", 50}, {"LMPIP", 51}, {"LMDIP", 52}  
  // };
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(new mc_solver::DynamicsConstraint(robots(), robot().robotIndex(), solver().dt(), {0.1, 0.01, 0.5}, 1.0, false, true));
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});
  postureTask = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(), 10, 1000);
  solver().addTask(postureTask);
  datastore().make<std::string>("ControlMode", "Position"); // entree dans le datastore
  datastore().make<std::string>("Coriolis", "Yes"); 
  // datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });

  gui()->addElement(this, {"Control Mode"},
                    mc_rtc::gui::Label("Current Control :", [this]() { return this->datastore().get<std::string>("ControlMode"); }),
                    mc_rtc::gui::Button("Position", [this]() { datastore().assign<std::string>("ControlMode", "Position"); }),
                    mc_rtc::gui::Button("Torque", [this]() { datastore().assign<std::string>("ControlMode", "Torque"); }));
  
  logger().addLogEntry("ControlMode",
                       [this]()
                       {
                         auto mode = datastore().get<std::string>("ControlMode");
                         if(mode.compare("") == 0) return 0;
                         if(mode.compare("Position") == 0) return 1;
                         if(mode.compare("Velocity") == 0) return 2;
                         if(mode.compare("Torque") == 0) return 3;
                         return 0;
                       });

  for(const auto &jn : robot().frames()){
    std::cout << jn << std::endl;
  }

  mc_rtc::log::success("MyFirstController init done");
}

bool MyFirstController::run()
{ 
  ctlTime_ += timeStep;
  std::cout<<ctlTime_<<std::endl;

  // qTarget = realRobot().q();

  // int i = 0;
  // auto it = postureTarget.begin();

  // for(const auto &q : qTarget){
  //   if(!q.empty() && q.size() == 1){
  //     if(it != postureTarget.end()){
  //         it->second = qTarget[i];
  //         std::cout << "qTarget[" << i << "] = " << qTarget[i][0] << std::endl;
  //     }
  //     std::advance(it, 1);
  //   }
  //   i++; 
  // }

  // std::cout << "size : " << i << std::endl;
  // std::cout << "postureTarget size : " << postureTarget.size() << std::endl;

  // for(auto qT = postureTarget.begin(); qT != postureTarget.end(); qT++){
  //   qT
  // }

  // for (const auto& [name, index] : jointNames) {postureTarget[name] = qTarget[index];}
  // for (const auto& pair : postureTarget) {std::cout << pair.first << ": ";
  //       for (double num : pair.second) {std::cout << num << " ";}
  //       std::cout << std::endl;}

  if (ctlTime_ > 3) {
    datastore().assign<std::string>("ControlMode", "Torque"); 
    postureTask->target(postureTarget); 

  }

  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if(ctrl_mode.compare("Position") == 0)
  {
    return mc_control::MCController::run(mc_solver::FeedbackType::OpenLoop);
  }
  else {
    return mc_control::MCController::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
  return false;
}

void MyFirstController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)