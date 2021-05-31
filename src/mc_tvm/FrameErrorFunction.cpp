/* Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL  */

#include <mc_tvm/FrameErrorFunction.h>

#include <mc_rbdyn/Robot.h>

namespace
{
int countOnes(const Eigen::Vector6d & dof)
{
  assert((dof.cwiseEqual(1.) || dof.cwiseEqual(0.)).all());
  return static_cast<int>(dof.sum());
}

template<bool add>
void assign_(Eigen::Ref<Eigen::MatrixXd> out, const Eigen::Ref<const Eigen::MatrixXd> & in);

template<>
void assign_<false>(Eigen::Ref<Eigen::MatrixXd> out, const Eigen::Ref<const Eigen::MatrixXd> & in)
{
  assert(out.rows() == in.rows() && out.cols() == in.cols());
  out = in;
}

template<>
void assign_<true>(Eigen::Ref<Eigen::MatrixXd> out, const Eigen::Ref<const Eigen::MatrixXd> & in)
{
  assert(out.rows() == in.rows() && out.cols() == in.cols());
  out += in;
}
} // namespace

namespace mc_tvm
{

FrameErrorFunction::FrameErrorFunction(mc_rbdyn::FramePtr f1, mc_rbdyn::FramePtr f2, const Eigen::Vector6d & dof)
: tvm::function::abstract::Function(countOnes(dof)), dof_(dof)
{
  // clang-format off
    registerUpdates(Update::Value, &FrameErrorFunction::updateValue,
                    Update::Jacobian, &FrameErrorFunction::updateJacobian,
                    Update::Velocity, &FrameErrorFunction::updateVelocity,
                    Update::NormalAcceleration, &FrameErrorFunction::updateNormalAcceleration);
  // clang-format on

  addOutputDependency<FrameErrorFunction>(Output::Value, Update::Value);
  addOutputDependency<FrameErrorFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<FrameErrorFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<FrameErrorFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  addInternalDependency<FrameErrorFunction>(Update::Jacobian, Update::Value);
  addInternalDependency<FrameErrorFunction>(Update::Velocity, Update::Value);
  addInternalDependency<FrameErrorFunction>(Update::NormalAcceleration, Update::Velocity);

  auto addRobot = [this](mc_rbdyn::FramePtr & fIn, mc_rbdyn::FramePtr & fOut, bool & useF, rbd::Jacobian & jac) {
    fOut = fIn;
    if(fIn->rbdJacobian().dof() > 0)
    {
      const auto & r = fIn->robot();
      useF = true;
      jac = fIn->rbdJacobian();
      addInputDependency<FrameErrorFunction>(Update::Value, fOut, mc_rbdyn::Frame::Output::Position);
      addInputDependency<FrameErrorFunction>(Update::Jacobian, fOut, mc_rbdyn::Frame::Output::Jacobian);
      addInputDependency<FrameErrorFunction>(Update::Velocity, fOut, mc_rbdyn::Frame::Output::Velocity);
      addInputDependency<FrameErrorFunction>(Update::NormalAcceleration, fOut,
                                             mc_rbdyn::Frame::Output::NormalAcceleration);
      addVariable(r.q(), false);
    }
    else
    {
      useF = false;
      addInputDependency<FrameErrorFunction>(Update::Value, fOut, mc_rbdyn::Frame::Output::Position);
    }
  };
  addRobot(f1, f1_, use_f1_, f1Jacobian_);
  addRobot(f2, f2_, use_f2_, f2Jacobian_);
  assert(((*f1->robot().q() == *f2->robot().q()) || !f1->robot().q()->intersects(*f2->robot().q()))
         && "Current implementation does not handle robots sharing sub-parts of their variables. updateJacobian should "
            "be changed if you want to handle that.");
  sameVariable_ = use_f1_ && use_f2_ && (*f1->robot().q() == *f2->robot().q());
  tmpMat_.resize(3, std::max(use_f1_ ? f1->robot().q()->size() : 0, use_f2_ ? f2->robot().q()->size() : 0));

  auto toDof = [](const auto & v) {
    return std::make_pair(static_cast<Dof>(v[0] * Dof::X + v[1] * Dof::Y + v[2] * Dof::Z), v.sum());
  };
  std::tie(rDof_, nr_) = toDof(dof.head<3>());
  std::tie(tDof_, nt_) = toDof(dof.tail<3>());
}

void FrameErrorFunction::updateValue()
{
  if(rDof_) rotErr_ = sva::rotationError(f1_->position().rotation(), f2_->position().rotation());
  assign<false>::run(value_.head(nr_), rotErr_, rDof_, tmpMat_);
  assign<false>::run(value_.tail(nt_), f2_->position().translation() - f1_->position().translation(), tDof_, tmpMat_);
  dLog_ = sva::SO3RightJacInv(rotErr_);
}

void FrameErrorFunction::updateJacobian()
{
  if(use_f1_)
  {
    auto & J1 = jacobian_[f1_->robot().q().get()];
    assign<false>::run(J1.topRows(nr_), -dLog_ * f1_->jacobian().topRows<3>(), rDof_, tmpMat_);
    assign<false>::run(J1.bottomRows(nt_), -f1_->jacobian().bottomRows<3>(), tDof_, tmpMat_);
  }
  if(use_f2_)
  {
    auto & J2 = jacobian_[f2_->robot().q().get()];
    if(sameVariable_)
    {
      assign<true>::run(J2.topRows(nr_), dLog_.transpose() * f2_->jacobian().topRows<3>(), rDof_, tmpMat_);
      assign<true>::run(J2.bottomRows(nt_), f2_->jacobian().bottomRows<3>(), tDof_, tmpMat_);
    }
    else
    {
      assign<false>::run(J2.topRows(nr_), dLog_.transpose() * f2_->jacobian().topRows<3>(), rDof_, tmpMat_);
      assign<false>::run(J2.bottomRows(nt_), f2_->jacobian().bottomRows<3>(), tDof_, tmpMat_);
    }
  }
}

void FrameErrorFunction::updateVelocity()
{
  if(use_f1_)
  {
    if(rDof_) rotVel_ = -dLog_ * f1_->velocity().angular();
    assign<false>::run(velocity_.tail(nt_), -f1_->velocity().linear(), tDof_, tmpMat_);
  }
  else
  {
    rotVel_.setZero();
    velocity_.tail(nt_).setZero();
  }
  if(use_f2_)
  {
    if(rDof_) rotVel_ += dLog_.transpose() * f2_->velocity().angular();
    assign<true>::run(velocity_.tail(nt_), f2_->velocity().linear(), tDof_, tmpMat_);
  }
  assign<false>::run(velocity_.head(nr_), rotVel_, rDof_, tmpMat_);
}

void FrameErrorFunction::updateNormalAcceleration()
{
  if(rDof_) d2Log_ = sva::SO3RightJacInvDot(rotErr_, rotVel_);
  if(use_f1_)
  {
    assign<false>::run(normalAcceleration_.head(nr_),
                       -d2Log_ * f1_->velocity().angular() - dLog_ * f1_->normalAcceleration().angular(), rDof_,
                       tmpMat_);
    assign<false>::run(normalAcceleration_.tail(nt_), -f1_->normalAcceleration().linear(), tDof_, tmpMat_);
  }
  else
  {
    normalAcceleration_.setZero();
  }
  if(use_f2_)
  {
    assign<true>::run(normalAcceleration_.head(nr_),
                      d2Log_.transpose() * f2_->velocity().angular()
                          + dLog_.transpose() * f2_->normalAcceleration().angular(),
                      rDof_, tmpMat_);
    assign<true>::run(normalAcceleration_.tail(nt_), f2_->normalAcceleration().linear(), tDof_, tmpMat_);
  }
}

template<bool add>
template<typename Derived>
void FrameErrorFunction::assign<add>::run(Eigen::Ref<Eigen::MatrixXd> out,
                                          const Eigen::MatrixBase<Derived> & in,
                                          mc_tvm::FrameErrorFunction::Dof dof,
                                          Eigen::MatrixXd & buffer)
{
  assert(in.rows() == 3);
  assert(out.rows() == ((dof & Dof::X) / Dof::X + (dof & Dof::Y) / Dof::Y + (dof & Dof::Z) / Dof::Z));

  // `in` can be an Eigen expression. We need to force computing its value before assigning (part of) it.
  auto b = buffer.leftCols(in.cols());
  if(dof) b.noalias() = in;

  switch(dof)
  {
    case Dof::___:
      break;
    case Dof::X__:
      assign_<add>(out, b.row(0));
      break;
    case Dof::_Y_:
      assign_<add>(out, b.row(1));
      break;
    case Dof::__Z:
      assign_<add>(out, b.row(2));
      break;
    case Dof::XY_:
      assign_<add>(out, b.topRows<2>());
      break;
    case Dof::X_Z:
      // assign_ should be templated on the out type to accept the following lines.
      // To avoid this complication (which would require an additional helper structure) for a single case, we use if
      // constexpr instead.
      if constexpr(add)
      {
        out.row(0) += b.row(0);
        out.row(1) += b.row(2);
      }
      else
      {
        out.row(0) = b.row(0);
        out.row(1) = b.row(2);
      }
      break;
    case Dof::_YZ:
      assign_<add>(out, b.bottomRows<2>());
      break;
    case Dof::XYZ:
      assign_<add>(out, b);
      break;
    default:
      assert(false);
  }
}

} // namespace mc_tvm
