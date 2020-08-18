/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <tvm/defs.h>
#include <tvm/internal/FirstOrderProvider.h>

namespace mc_tvm
{

void MC_TVM_DLLAPI
    splitAddJacobian(const tvm::MatrixConstRef & J,
                     const tvm::VariableVector & vars,
                     tvm::utils::internal::map<tvm::Variable const *, tvm::internal::MatrixWithProperties> & jacobians);

} // namespace mc_tvm
