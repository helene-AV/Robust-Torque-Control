#include "RobustTorqueControl.h"

#include <Eigen/src/Core/Matrix.h>
#include <RBDyn/MultiBodyConfig.h>
#include <SpaceVecAlg/PTransform.h>

#include <array>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <iterator>
#include <map>

#include <mc_control/MCController.h>
#include <mc_control/mc_global_controller.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/gui/types.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/unique_ptr.h>
#include <mc_solver/ContactConstraint.h>
#include <mc_solver/TVMQPSolver.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/CoMTask.h>
#include <memory>
#include <mc_control/GlobalPluginMacros.h>
#include <ostream>
#include <string>
#include <tvm/ControlProblem.h>
#include <tvm/LinearizedControlProblem.h>

#include <mc_solver/DynamicsConstraint.h>


#include <iostream>
#include <fstream>

#include <iomanip>  // For std::setw
#include <tvm/scheme/internal/ProblemComputationData.h>


RobustTorqueControl::RobustTorqueControl(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt, config, Backend::Tasks), ctlTime_(0)
{
  // postureTarget = {
  //     {"RCY", {-7.20383e-09}}, {"RCR", {9.95698e-09}}, {"RCP", {-0.468969}}, {"RKP", {0.872667}}, {"RAP", {-0.403692}}, {"RAR", {-8.24594e-09}},
  //     {"LCY", {1.14468e-08}}, {"LCR", {9.16934e-10}}, {"LCP", {-0.468969}}, {"LKP", {0.872667}}, {"LAP", {-0.403692}}, {"LAR", {7.9422e-10}},
  //     {"WP", {-1.12324e-06}}, {"WR", {1.5136e-08}}, {"WY", {-4.38556e-09}}, {"HY", {-1.64622e-09}}, {"HP", {-5.59799e-08}},
  //     {"RSC", {1.53608e-07}}, {"RSP", {1.0472}}, {"RSR", {0.349066}}, {"RSY", {0.0872664}}, {"REP", {-1.8326}}, {"RWRY", {4.40205e-09}}, {"RWRR", {0.698132}}, {"RWRP", {1.13281e-08}}, {"RHDY", {1.94179e-10}},
  //     {"LSC", {-1.52486e-07}}, {"LSP", {1.0472}}, {"LSR", {-0.349066}}, {"LSY", {-0.0872664}}, {"LEP", {-1.8326}}, {"LWRY", {-4.43891e-09}}, {"LWRR", {-0.698132}}, {"LWRP", {1.15108e-08}}, {"LHDY", {-1.96356e-10}},
  //     // {"RTMP", {0.906}}, {"RTPIP", {0.000}}, {"RTDIP", {0.000}},  {"RIMP", {-0.906}},  {"RIPIP", {0.000}}, {"RIDIP", {0.000}}, {"RMMP", {-0.906}}, {"RMPIP", {0.000}}, {"RMDIP", {0.000}},
  //     // {"LTMP", {0.906}}, {"LTPIP", {0.000}}, {"LTDIP", {0.000}},  {"LIMP", {0.906}},  {"LIPIP", {0.000}}, {"LIDIP", {0.000}}, {"LMMP", {0.906}}, {"LMPIP", {0.000}}, {"LMDIP", {0.000}},
  //     {"RTMP", {}}, {"RTPIP", {}}, {"RTDIP", {}},  {"RIMP", {}},  {"RIPIP", {}}, {"RIDIP", {}}, {"RMMP", {}}, {"RMPIP", {}}, {"RMDIP", {}},
  //     {"LTMP", {}}, {"LTPIP", {}}, {"LTDIP", {}},  {"LIMP", {}},  {"LIPIP", {}}, {"LIDIP", {}}, {"LMMP", {}}, {"LMPIP", {}}, {"LMDIP", {}},

  //   };
  // postureTargetJVRC1 = {
  //       {"R_HIP_P", {-0.382943}}, {"R_HIP_R", {-0.0074722}}, {"R_HIP_Y", {-0.00107667}}, {"R_KNEE", {0.7244}}, {"R_ANKLE_P", {-0.330407}}, {"R_ANKLE_R", {-0.00521519}},
  //       {"L_HIP_P", {-0.382943}}, {"L_HIP_R", {0.0074722}}, {"L_HIP_Y", {-0.00107667}}, {"L_KNEE", {0.7244}}, {"L_ANKLE_P", {-0.330407}}, {"L_ANKLE_R", {-0.0221519}},
  //       {"WAIST_Y", {5.01705e-07}}, {"WAIST_R", {-7.03367e-07}}, {"WAIST_P", {0.133806}}, {"NECK_Y", {-5.74927e-08}}, {"NECK_R", {3.46569e-07}}, {"NECK_P", {0.00045688}},
  //       {"R_SHOULDER_P", {-0.0493383}}, {"R_SHOULDER_R", {-0.166255}}, {"R_SHOULDER_Y", {0.000705349}}, {"R_ELBOW_P", {-0.516636}}, {"R_ELBOW_Y", {1.90064e-05}},
  //       {"R_WRIST_R", {0.000472426}}, {"R_WRIST_Y", {4.49294e-05}}, {"L_SHOULDER_P", {-0.0493383}}, {"L_SHOULDER_R", {0.166255}}, {"L_SHOULDER_Y", {-0.000705349}},
  //       {"L_ELBOW_P", {-0.516733}}, {"L_ELBOW_Y", {-1.90151e-05}}, {"L_WRIST_R", {0.000472426}}, {"L_WRIST_Y", {4.49294e-05}}
  //   };

  // postureTargetFixJVRC1 = {
  //       {"R_HIP_P", {-0.380557}}, {"R_HIP_R", {-0.0095768}}, {"R_HIP_Y", {-0.000174627}}, {"R_KNEE", {0.719142}}, {"R_ANKLE_P", {-0.332425}}, {"R_ANKLE_R", {-0.00926944}},
  //       {"L_HIP_P", {-0.378981}}, {"L_HIP_R", {0.0197925}}, {"L_HIP_Y", {6.24436e-05}}, {"L_KNEE", {0.719544}}, {"L_ANKLE_P", {-0.330436}}, {"L_ANKLE_R", {-0.0201683}},
  //       {"WAIST_Y", {0.00327755}}, {"WAIST_R", {-5.35138e-07}}, {"WAIST_P", {0.133909}}, {"NECK_Y", {6.20988e-08}}, {"NECK_R", {-9.66116e-08}}, {"NECK_P", {0.000483604}},
  //       {"R_SHOULDER_P", {-0.0496289}}, {"R_SHOULDER_R", {-0.166253}}, {"R_SHOULDER_Y", {0.000705393}}, {"R_ELBOW_P", {-0.516735}}, {"R_ELBOW_Y", {1.85059e-05}},
  //       {"R_WRIST_R", {0.000472446}}, {"R_WRIST_Y", {4.374e-05}}, {"L_SHOULDER_P", {-0.0496226}}, {"L_SHOULDER_R", {0.166255}}, {"L_SHOULDER_Y", {-0.000704645}},
  //       {"L_ELBOW_P", {-0.516733}}, {"L_ELBOW_Y", {-0.000471656}}, {"L_WRIST_R", {7.70294e-05}}, {"L_WRIST_Y", {-0.000277014}}
  //   };

  // postureTargetFixReal = {
  //       {"RCY", {-2.06892e-09}}, {"RCR", {-0.000105259}}, {"RCP", {-0.468851}}, {"RKP", {0.87317}}, {"RAP", {-0.40422}}, {"RAR", {0.000351273}},
  //       {"LCY", {6.05859e-09}}, {"LCR", {0.000105269}}, {"LCP", {-0.468851}}, {"LKP", {0.87317}}, {"LAP", {-0.40422}}, {"LAR", {-0.000351285}},
  //       {"WP", {-0.00524202}}, {"WR", {0.000145019}}, {"WY", {-5.94072e-09}}, {"HY", {-6.70093e-07}}, {"HP", {0.000171501}},
  //       {"RSC", {4.60242e-05}}, {"RSP", {1.04471}}, {"RSR", {0.347929}}, {"RSY", {0.0834878}}, {"REP", {-1.83162}}, {"RWRY", {0.000505649}}, {"RWRR", {0.697133}}, {"RWRP", {0.00133058}}, {"RHDY", {6.02102e-05}},
  //       {"LSC", {-4.52962e-05}}, {"LSP", {1.04471}}, {"LSR", {-0.34793}}, {"LSY", {-0.0834915}}, {"LEP", {-1.83162}}, {"LWRY", {-0.000505652}}, {"LWRR", {-0.697133}}, {"LWRP", {0.00133077}}, {"LHDY", {-6.02042e-05}}
  //   };

  dynamicConstraintTest = std::make_unique<mc_solver::DynamicsConstraint>(robots(), robot().robotIndex(), solver().dt(), std::array<double, 3>({0.1, 0.01, 0.5}), 1.0, false, true);
  // config_.load(config);
  contactConstraintTest = std::make_unique<mc_solver::ContactConstraint>(timeStep, mc_solver::ContactConstraint::ContactType::Acceleration);
  solver().addConstraintSet(dynamicConstraintTest);
  solver().addConstraintSet(contactConstraintTest);
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});
  postureTask = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(), 10, 1000);

  // comTask = std::make_shared<mc_tasks::CoMTask>(robots(), robot().robotIndex(), 10.0, 1000);
  // solver().addTask(comTask);
  solver().addTask(postureTask);
  datastore().make<std::string>("ControlMode", "Position"); // entree dans le datastore
  datastore().make<std::string>("Coriolis", "Yes"); 
  // postureTask->stiffness(1);
  // datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });
  // // comTask->move_com({0,0,1});
  // comZero = Eigen::Vector3d{0, 0, 0};
  jointIndexL = robot().jointIndexByName("LKP");
  jointIndexR = robot().jointIndexByName("RKP");

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

  logger().addLogEntry("leftFootRatio",
  [this]()
  {
    return leftFootRatio_;
  });

  // Update observers
  datastore().make_call("KinematicAnchorFrame::" + robot().name(),
                        [this](const mc_rbdyn::Robot & robot)
                        {
                          leftFootRatio_ = robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                           / (robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                              + robot.indirectSurfaceForceSensor("RightFootCenter").force().z());
                          return sva::interpolate(robot.surfacePose("RightFootCenter"),
                                                  robot.surfacePose("LeftFootCenter"), leftFootRatio_);
                        });
  
  logger_->addLogEntry("my torque", [this](){return tau_;});

  mc_rtc::log::success("RobustTorqueControl init done");

}


bool RobustTorqueControl::run()
{ 
  if(done)
  {
    exit(0);
  }
  //Torque control over 10s
  if (ctlTime_ >= 10.000) { 
    datastore().assign<std::string>("ControlMode", "Torque");
    // done = true;
  }

  if(ctlTime_>= 20.0)
  {
    // comTask->com(comZero - Eigen::Vector3d{0, 0, 0.5});
    postureTask->target({{"LKP",{1.0}}});
    postureTask->target({{"RKP",{1.0}}});
    done = true;
  }
  
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  mc_rtc::log::info(ctrl_mode);

  if(verbose and start)
  {
    if(this->qpsolver->backend()== Backend::TVM)
    {
      tau_ = id_tvm("/home/helene/work/inverse_dynamics/id_tvm/1movement/data/");

    }else if (this->qpsolver->backend()== Backend::Tasks) {
      tau_ = id_tasks("/home/helene/work/inverse_dynamics/id_task_forces/1movement/data/");
    }
  }

  start = true; 

  ctlTime_ += timeStep;
  // done = true;
  mc_rtc::log::info("Current time {}", ctlTime_);
  if(ctrl_mode.compare("Position") == 0)
  {
    mc_rtc::log::info("OpenLoop");
    return mc_control::MCController::run(mc_solver::FeedbackType::OpenLoop);

  }else{
      mc_rtc::log::info("CloseLoop");
      return mc_control::MCController::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
}

void RobustTorqueControl::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
  // comTask->reset();
  // comZero = comTask->com();
}

void RobustTorqueControl::switch_com_target()
{
  if(comDown) { comTask->com(comZero - Eigen::Vector3d{0, 0, 0.5}); }
  else { comTask->com(comZero); }
  comDown = !comDown;
}

Eigen::MatrixXd RobustTorqueControl::loadMatrixFromCSV(const std::string& filename) {
    std::ifstream file(filename);  // Open the file
    std::string line;
    std::vector<std::string> rows;  // Store the rows to determine the size of the matrix

    // Read the file line by line
    while (std::getline(file, line)) {
        rows.push_back(line);
    }

    // Close the file after reading
    file.close();

    // Determine the number of rows and columns
    size_t numRows = rows.size();
    size_t numCols = 0;

    // Find the maximum number of columns by inspecting the first row
    std::stringstream ss(rows[0]);
    std::string value;
    while (ss >> value) {
        numCols++;
    }

    // Create the Eigen matrix with the correct size
    Eigen::MatrixXd mat(numRows, numCols);

    // Populate the matrix directly
    for (size_t i = 0; i < numRows; ++i) {
        std::stringstream ss(rows[i]);
        for (size_t j = 0; j < numCols; ++j) {
            ss >> mat(i, j);  // Extract each value and assign it to the matrix
        }
    }

    return mat;
}


// Function to load a vector from a CSV file
Eigen::VectorXd RobustTorqueControl::loadVectorFromCSV(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    std::vector<double> vector_data;
    
    while (std::getline(file, line)) {
        vector_data.push_back(std::stod(line));
    }

    Eigen::VectorXd vec(vector_data.size());
    for (int i = 0; i < vector_data.size(); i++) {
        vec(i) = vector_data[i];
    }

    return vec;
}

Eigen::MatrixXd RobustTorqueControl::loadJacobianFromCSV(const std::string& filename, int leg, int force) {
    std::ifstream file(filename);  // Open the file
    std::string line;
    std::vector<std::string> rows;  // Store the rows to determine the size of the matrix

    // Read the file line by line
    while (std::getline(file, line)) {
        rows.push_back(line);
    }

    // Close the file after reading
    file.close();

    // Determine the number of rows and columns
    size_t numRows = rows.size();
    size_t numCols = 0;

    // Find the maximum number of columns by inspecting the first row
    std::stringstream ss(rows[0]);
    std::string value;
    while (ss >> value) {
        numCols++;
    }

    // Create the Eigen matrix with the correct size
    Eigen::MatrixXd mat(numRows, 41);

    // Populate the matrix directly
    for (size_t i = 0; i < numRows; ++i) {

        std::stringstream ss(rows[i]);

        for (size_t j = 0; j < 41; ++j) {
            if(((j < 12 and j > 5) or j >17) and leg == 0)
            {
                mat(i, j) = 0;
            }else if(j>11 and leg == 1) {
                mat(i, j) = 0;
            }else{
                ss >> mat(i, j);  // Extract each value and assign it to the matrix
            }
        }

    }

    return mat;
}

Eigen::VectorXd RobustTorqueControl::id_tvm(std::string filename)
{
    // Load matrices and vectors from CSV
    Eigen::MatrixXd H = loadMatrixFromCSV(filename + "matrix-tasks-H.csv");
    Eigen::MatrixXd C = loadMatrixFromCSV(filename + "matrix-tasks-C.csv");
    Eigen::MatrixXd qdotdot = loadMatrixFromCSV(filename + "qdotdot.csv");

    Eigen::VectorXd tau = H*qdotdot + C;

    Eigen::MatrixXd force_jac;
    Eigen::MatrixXd contact;

    for(int leg = 0; leg<2; leg++)
    {
        for(int force = 0; force<4; force++)
        {

            force_jac = loadMatrixFromCSV(filename + "jacobian_force" + std::to_string(force) + std::to_string(leg) + ".csv");
            contact = loadMatrixFromCSV(filename + "force" + std::to_string(force) + std::to_string(leg) + ".csv");

            tau += force_jac*contact; 
        }
    }

    return tau;
}

Eigen::VectorXd RobustTorqueControl::id_tasks(std::string filename){

  // Load matrices and vectors from CSV
  Eigen::MatrixXd H = loadMatrixFromCSV(filename + "matrix-tasks-H.csv");
  Eigen::MatrixXd C = loadMatrixFromCSV(filename + "matrix-tasks-C.csv");
  Eigen::MatrixXd qdotdot = loadVectorFromCSV(filename + "alphaDsegment.csv");

  Eigen::VectorXd tau = H*qdotdot + C;

  Eigen::MatrixXd force_jac;
  Eigen::MatrixXd contact;


  for(int leg = 0; leg<2; leg++)
  {
      for(int force = 0; force<4; force++)
      {
  
          force_jac = loadJacobianFromCSV(filename + "jac_" + std::to_string(force) + std::to_string(leg) + ".csv", leg, force);
          contact = loadMatrixFromCSV(filename + "force" + std::to_string(force) + std::to_string(leg) + ".csv");

          tau += force_jac.transpose()*contact; 
      }
  }

  return tau;

}

CONTROLLER_CONSTRUCTOR("RobustTorqueControl", RobustTorqueControl)