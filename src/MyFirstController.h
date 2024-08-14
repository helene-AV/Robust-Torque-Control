#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/PositionTask.h>
#include <string>


#include "api.h"

struct MyFirstController_DLLAPI MyFirstController : public mc_control::MCController
{
  MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::shared_ptr<mc_tasks::PostureTask> postureTask ;
  std::map<std::string, std::vector<double>> postureTarget;
  std::vector<std::vector<double>> qTarget; 
  std::map<std::string, size_t> jointNames;

private:
  mc_rtc::Configuration config_;
  double ctlTime_;
};
