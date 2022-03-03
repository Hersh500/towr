/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <cmath>
#include <iostream>
#include <fstream>
#include <gflags/gflags.h>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/models/endeffector_mappings.h>
#include <towr/initialization/gait_generator.h>

using namespace towr;

// Command line arguments
DEFINE_string(height_csv, "test.csv", "Path to the heightmap csv");
DEFINE_uint32(num_steps, 8, "Number of footsteps to take");
DEFINE_bool(walk, true, "true if walking gait, false if running gait");
DEFINE_double(initial_x_vel, 0.0, "initial x velocity");
DEFINE_double(initial_y_vel, 0.0, "initial y velocity");
DEFINE_double(T, 10.0, "total time of the motion");
DEFINE_double(goal_x, 2.0, "goal x location in meters");

void printProblemOutput(SplineHolder& solution) {
  using namespace std;
  double t = 0.0;
  while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;

    cout << "Foot L position x,y,z:          \t";
    cout << solution.ee_motion_.at(L)->GetPoint(t).p().transpose() << "\t[m]" << endl;
    cout << "Foot R position x,y,z:          \t";
    cout << solution.ee_motion_.at(R)->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "L Contact force x,y,z:          \t";
    cout << solution.ee_force_.at(L)->GetPoint(t).p().transpose() << "\t[N]" << endl;
    cout << "R Contact force x,y,z:          \t";
    cout << solution.ee_force_.at(R)->GetPoint(t).p().transpose() << "\t[N]" << endl;

    bool contact_L = solution.phase_durations_.at(L)->IsContactPhase(t);
    bool contact_R = solution.phase_durations_.at(R)->IsContactPhase(t);
    std::string foot_in_contact_l = contact_L? "yes" : "no";
    std::string foot_in_contact_r = contact_R? "yes" : "no";
    cout << "L Foot in contact:              \t" + foot_in_contact_l << endl;
    cout << "R Foot in contact:              \t" + foot_in_contact_r << endl;

    cout << endl;

    t += 0.2;
  }
}

void getContactSequence(SplineHolder& solution, std::vector<Eigen::Vector3d>& contacts) {
  double t = 0;
  bool left_foot_in_contact = true;
  bool right_foot_in_contact = true;
  while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
    bool cur_lfoot = solution.phase_durations_.at(L)->IsContactPhase(t);
    bool cur_rfoot = solution.phase_durations_.at(R)->IsContactPhase(t);
    // On a new contact, add it to the sequence of footsteps.
    if (!left_foot_in_contact && cur_lfoot) {
      contacts.push_back(solution.ee_motion_.at(L)->GetPoint(t).p().transpose());
    }
    if (!right_foot_in_contact && cur_rfoot) {
      contacts.push_back(solution.ee_motion_.at(R)->GetPoint(t).p().transpose());
    }
    left_foot_in_contact = cur_lfoot;
    right_foot_in_contact = cur_rfoot;
    t += 0.05;
  }
}


int main(int argc, char* argv[])
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  NlpFormulation formulation;

  // Create the heightmap from the input csv, else use flat ground.
  std::shared_ptr<CSVHeightMap> heightmap = std::make_shared<CSVHeightMap>(FLAGS_height_csv);
  if (heightmap->isEmpty()) {
    std::cout << "Couldn't read heightmap csv!" << std::endl;
    formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  } else {
    formulation.terrain_ = heightmap;
  }
  double z_ground = formulation.terrain_->GetHeight(0, 0);

  // Kinematic limits and dynamic parameters of the hopper
  formulation.model_ = RobotModel(RobotModel::Cassie);
  int n_ee = formulation.model_.kinematic_model_->GetNumberOfEndeffectors();

  // set the initial position
  formulation.initial_ee_W_ = formulation.model_.kinematic_model_->GetNominalStanceInBase();
  std::for_each(formulation.initial_ee_W_.begin(), formulation.initial_ee_W_.end(),
                [&](Eigen::Vector3d& p){p.z() = z_ground;});

  formulation.initial_base_.lin.at(kPos).z() = -formulation.model_.kinematic_model_->GetNominalStanceInBase().front().z() + z_ground;

  // define the desired goal state
  formulation.final_base_.lin.at(towr::kPos) << FLAGS_goal_x, 0.0, 0.7 + formulation.terrain_->GetHeight(FLAGS_goal_x, 0.0) + 0.7;

  // Parameters that define the motion. See c'tor for default values or
  // other values that can be modified.
  // First we define the initial phase durations, that can however be changed
  // by the optimizer. The number of swing and stance phases however is fixed.
  // alternating stance and swing:     ____-----_____-----_____-----_____
  auto gait_gen = GaitGenerator::MakeGaitGenerator(n_ee);
  std::vector<GaitGenerator::Gaits> gait_vector = {GaitGenerator::Stand};
  // divide by 2 since each Walk1 is 2 steps--left and right.
  for (int i = 0; i < (int)(FLAGS_num_steps/2); i++) {
    gait_vector.push_back(GaitGenerator::Walk1);
  }
  gait_vector.push_back(GaitGenerator::Stand);
  gait_gen->SetGaits(gait_vector);
  for (int ee=0; ee < n_ee; ++ee) {
    formulation.params_.ee_phase_durations_.push_back(gait_gen->GetPhaseDurations(FLAGS_T, ee));
    formulation.params_.ee_in_contact_at_start_.push_back(gait_gen->IsInContactAtStart(ee));
  }

  formulation.params_.OptimizePhaseDurations();

  // Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  ifopt::Problem nlp;
  SplineHolder solution;
  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  // You can add your own elements to the nlp as well, simply by calling:
  // nlp.AddVariablesSet(your_custom_variables);
  // nlp.AddConstraintSet(your_custom_constraints);
  // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
  auto solver = std::make_shared<ifopt::IpoptSolver>();
//  solver->SetOption("derivative_test", "first-order");
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 60.0);
  solver->SetOption("sb", "yes");
  solver->SetOption("print_level", 0);
  solver->Solve(nlp);
  // Can directly view the optimization variables through:
  // Eigen::VectorXd x = nlp.GetVariableValues()
  // However, it's more convenient to access the splines constructed from these
  // variables and query their values at specific times:
  using namespace std;
  cout.precision(2);
//  nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
//  cout << fixed;
//  cout << "\n====================\nCassie trajectory:\n====================\n";
  vector<Eigen::Vector3d> contacts;
  getContactSequence(solution, contacts);

  std::ofstream ofile;
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                               ", ", "\n", "", "", "", "");
  ofile.open(FLAGS_height_csv + "output");

  for (Eigen::Vector3d contact : contacts) {
    ofile << contact.transpose().format(CommaInitFmt) << endl;
  }

//  printProblemOutput(solution);
  return solver->GetReturnStatus();
}
