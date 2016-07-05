/**
 @file    snopt_adapter_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 4, 2016
 @brief   Brief description
 */

#include <gtest/gtest.h>
#include <xpp/zmp/snopt_adapter.h>

namespace xpp {
namespace zmp {

TEST(SnoptAdapterTest, Sample){

  SnoptAdapter::NLPPtr nlp;
  auto snopt_problem = SnoptAdapter::GetInstance();
  snopt_problem->SetNLP(nlp);
  snopt_problem->Init();

  int Cold = 0, Basis = 1, Warm = 2;
  snopt_problem->solve(Cold);

  Eigen::VectorXd x = snopt_problem->GetVariables();
  std::cout << "x: " << x.transpose() << std::endl;
}

} /* namespace zmp */
} /* namespace xpp */