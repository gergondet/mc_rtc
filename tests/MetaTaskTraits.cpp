/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/details/details.h>

/** This executable only test the traits used to determine if a task has valid
 * refVel and/or refAccel member functions, compilation failure would indicate
 * an issue with said traits. */

// clang-format off

struct Valid
{
  const Eigen::VectorXd & refVel() { return vel_; }
  void refVel(const Eigen::VectorXd & vel) { vel_ = vel; }
  const Eigen::VectorXd & refAccel() { return vel_; }
  void refAccel(const Eigen::VectorXd & vel) { vel_ = vel; }
private:
  Eigen::VectorXd vel_;
};

struct MissingGetter
{
  void refVel(const Eigen::VectorXd & vel) { vel_ = vel; }
  void refAccel(const Eigen::VectorXd & vel) { vel_ = vel; }
private:
  Eigen::VectorXd vel_;
};

struct MissingSetter
{
  const Eigen::VectorXd & refVel() { return vel_; }
  const Eigen::VectorXd & refAccel() { return vel_; }
private:
  Eigen::VectorXd vel_;
};

struct MissingBoth
{
};

int main()
{
  static_assert(mc_tasks::details::has_refVel_v<Valid>);
  static_assert(mc_tasks::details::has_refAccel_v<Valid>);
  static_assert(
    !mc_tasks::details::has_refVel_v<MissingGetter> &&
    mc_tasks::details::has_refVel_setter_v<MissingGetter> &&
    !mc_tasks::details::has_refVel_getter_v<MissingGetter>
  );
  static_assert(
    !mc_tasks::details::has_refAccel_v<MissingGetter> &&
    mc_tasks::details::has_refAccel_setter_v<MissingGetter> &&
    !mc_tasks::details::has_refAccel_getter_v<MissingGetter>
  );
  static_assert(
    !mc_tasks::details::has_refVel_v<MissingSetter> &&
    !mc_tasks::details::has_refVel_setter_v<MissingSetter> &&
    mc_tasks::details::has_refVel_getter_v<MissingSetter>
  );
  static_assert(
    !mc_tasks::details::has_refAccel_v<MissingSetter> &&
    !mc_tasks::details::has_refAccel_setter_v<MissingSetter> &&
    mc_tasks::details::has_refAccel_getter_v<MissingSetter>
  );
  static_assert(
    !mc_tasks::details::has_refVel_v<MissingBoth> &&
    !mc_tasks::details::has_refVel_setter_v<MissingBoth> &&
    !mc_tasks::details::has_refVel_getter_v<MissingBoth>
  );
  static_assert(
    !mc_tasks::details::has_refAccel_v<MissingBoth> &&
    !mc_tasks::details::has_refAccel_setter_v<MissingBoth> &&
    !mc_tasks::details::has_refAccel_getter_v<MissingBoth>
  );
  return 0;
}
