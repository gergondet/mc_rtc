#include <mc_tvm/PostureFunction.h>

#include <mc_rbdyn/Frame.h>
#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

PostureFunction::PostureFunction(mc_rbdyn::Robot & robot)
: tvm::function::abstract::Function(robot.qJoints().totalSize()), robot_(robot),
  j0_(robot_->mb().joint(0).type() == rbd::Joint::Free ? 1 : 0)
{
  // clang-format off
  registerUpdates(Update::Value, &PostureFunction::updateValue,
                  Update::Velocity, &PostureFunction::updateVelocity);
  // clang-format on
  addOutputDependency<PostureFunction>(Output::Value, Update::Value);
  addOutputDependency<PostureFunction>(Output::Velocity, Update::Velocity);
  addInputDependency<PostureFunction>(Update::Value, robot_, mc_rbdyn::Robot::Output::FK);
  addInputDependency<PostureFunction>(Update::Velocity, robot_, mc_rbdyn::Robot::Output::FK);
  addVariable(robot_->qJoints(), false);
  int startIdx = 0;
  for(const auto & v : robot.qJoints().variables())
  {
    auto & jac = jacobian_[v.get()];
    jac.block(startIdx, 0, v->space().tSize(), v->space().tSize()).setIdentity();
    JDot_[v.get()].setZero();
    startIdx += v->space().tSize();
  }
  normalAcceleration_.setZero();
  value_.setZero();
  velocity_.setZero();
  reset();
}

void PostureFunction::reset()
{
  posture_ = robot_->mbc().q;
}

void PostureFunction::posture(const std::string & j, const std::vector<double> & q)
{
  if(!robot_->hasJoint(j))
  {
    mc_rtc::log::error("[PostureFunction] No joint named {} in {}", j, robot_->name());
    return;
  }
  auto jIndex = static_cast<size_t>(robot_->mb().jointIndexByName(j));
  if(posture_[jIndex].size() != q.size())
  {
    mc_rtc::log::error("[PostureFunction] Wrong size for input target on joint {}, excepted {} got {}", j,
                       posture_[jIndex].size(), q.size());
    return;
  }
  posture_[static_cast<size_t>(jIndex)] = q;
}

namespace
{
bool isValidPosture(const std::vector<std::vector<double>> & ref, const std::vector<std::vector<double>> & in)
{
  if(ref.size() != in.size())
  {
    return false;
  }
  for(size_t i = 0; i < ref.size(); ++i)
  {
    if(ref[i].size() != in[i].size())
    {
      return false;
    }
  }
  return true;
}
} // namespace

void PostureFunction::posture(const std::vector<std::vector<double>> & p)
{
  if(!isValidPosture(posture_, p))
  {
    mc_rtc::log::error("[PostureFunction] Invalid posture provided for {}", robot_->name());
    return;
  }
  posture_ = p;
}

void PostureFunction::updateValue()
{
  int pos = 0;
  for(int jI = j0_; jI < robot_->mb().nrJoints(); ++jI)
  {
    auto jIdx = static_cast<size_t>(jI);
    const auto & j = robot_->mb().joint(jI);
    if(j.dof() == 1) // prismatic or revolute
    {
      value_(pos) = robot_->mbc().q[jIdx][0] - posture_[jIdx][0];
      pos++;
    }
    else if(j.dof() == 4) // spherical
    {
      Eigen::Matrix3d ori(
          Eigen::Quaterniond(posture_[jIdx][0], posture_[jIdx][1], posture_[jIdx][2], posture_[jIdx][3]).matrix());
      auto error = sva::rotationError(ori, robot_->mbc().jointConfig[jIdx].rotation());
      value_.segment(pos, 3) = error;
      pos += 3;
    }
  }
}

void PostureFunction::updateVelocity()
{
  int pos = 0;
  for(int jI = j0_; jI < robot_->mb().nrJoints(); ++jI)
  {
    for(auto & qI : robot_->mbc().alpha[static_cast<size_t>(jI)])
    {
      velocity_(pos) = qI;
      pos++;
    }
  }
}

} // namespace mc_tvm
