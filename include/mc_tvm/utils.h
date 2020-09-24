/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <tvm/defs.h>
#include <tvm/internal/FirstOrderProvider.h>

namespace mc_tvm
{

namespace internal
{

struct JacobianMapExposer : public tvm::internal::FirstOrderProvider
{
  using JacobianMap = decltype(jacobian_);
};

using JacobianMap = JacobianMapExposer::JacobianMap;

}

void MC_TVM_DLLAPI
    splitAddJacobian(const tvm::MatrixConstRef & J,
                     const tvm::VariableVector & vars,
                     internal::JacobianMap & jacobians);

} // namespace mc_tvm
