#pragma once

#include <mc_control/mc_controller.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/CoMTask.h>
#include <memory>
#include <string>


#include "api.h"

struct RobustTorqueControl_DLLAPI RobustTorqueControl : public mc_control::MCController
{
  RobustTorqueControl(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::shared_ptr<mc_tasks::PostureTask> postureTask ;
  std::shared_ptr<mc_tasks::CoMTask> comTask ;

  std::unique_ptr<mc_solver::ContactConstraint> contactConstraintTest;
  std::unique_ptr<mc_solver::DynamicsConstraint> dynamicConstraintTest;

  std::map<std::string, std::vector<double>> postureTarget;
  std::map<std::string, std::vector<double>> postureTargetJVRC1;
  std::map<std::string, std::vector<double>> postureTargetFixJVRC1;
  std::map<std::string, std::vector<double>> postureTargetFixReal;  
  std::vector<std::vector<double>> qTarget; 
  std::map<std::string, std::vector<double>> stance;

private:
  mc_rtc::Configuration config_;
  double ctlTime_;
  double leftFootRatio_;

  std::vector<double> jointTorqueVec;
};
