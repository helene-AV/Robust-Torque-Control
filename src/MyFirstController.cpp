#include "MyFirstController.h"
#include <SpaceVecAlg/PTransform.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>

MyFirstController::MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt, config, Backend::TVM)
{
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(new mc_solver::DynamicsConstraint(robots(), 0, solver().dt(), {0.1, 0.01, 0.5}, 0.9, false, true));
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  postureTask = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(), 1, 1);
  solver().addTask(postureTask);

  // Eigen::VectorXd dimWeight(300);
  // dimWeight <<  0., 0., 10.;
  // endEffectorTask = std::make_shared<mc_tasks::EndEffectorTask>(robot().frame("tool_frame"), 10.0, 100);
  // endEffectorTask->reset();
  // endEffectorTask->dimWeight(dimWeight);
  // solver().addTask(endEffectorTask); 
  // addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  // addContact({robot().name(), "ground", "RightFoot", "AllGround"});
  datastore().make<std::string>("ControlMode", "Torque"); // entree dans le datastore
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });

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

  mc_rtc::log::success("MyFirstController init done");
}

bool MyFirstController::run()
{ 
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  
  if(ctrl_mode.compare("Position") == 0)
  {
    return mc_control::MCController::run(mc_solver::FeedbackType::OpenLoop);
  }
  else
  {
    return mc_control::MCController::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
  return false;
}

void MyFirstController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)