#include <mc_tvm/utils.h>

#include <tvm/Variable.h>
#include <tvm/VariableVector.h>

namespace mc_tvm
{

void splitAddJacobian(const tvm::MatrixConstRef & J,
                      const tvm::VariableVector & vars,
                      tvm::utils::internal::map<tvm::Variable const *, tvm::internal::MatrixWithProperties> & jacobians)
{
  Eigen::DenseIndex s = 0;
  for(const auto & v : vars.variables())
  {
    auto n = static_cast<Eigen::DenseIndex>(v->space().tSize());
    jacobians[v.get()] += J.middleCols(s, n);
    s += n;
  }
}

} // namespace mc_tvm
