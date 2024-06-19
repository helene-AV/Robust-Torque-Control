#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <memory>

#include "api.h"

struct MyFirstController_DLLAPI MyFirstController : public mc_control::MCController
{
  MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void switch_target();

  std::shared_ptr<mc_tasks::PostureTask> postureTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> endEffectorTask;

private:
  mc_rtc::Configuration config_;
};
