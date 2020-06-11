/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robot.h>

#include <mc/rtc/deprecated.hh>

#include <optional>
#include <string_view>

namespace mc_rbdyn
{

struct MC_RBDYN_DLLAPI Robots
{
  friend struct Robot;

public:
  /** @name Iterators
   *
   * These functions provide an iterator interface to Robots
   *
   * @{
   */
  using iterator = std::vector<RobotPtr>::iterator;
  using const_iterator = std::vector<ConstRobotPtr>::const_iterator;
  using reverse_iterator = std::vector<RobotPtr>::reverse_iterator;
  using const_reverse_iterator = std::vector<ConstRobotPtr>::const_reverse_iterator;
  using size_type = std::vector<RobotPtr>::size_type;
  using value_type = std::vector<RobotPtr>::value_type;
  /** @} */

  Robots();
  Robots(const Robots & rhs);
  Robots & operator=(const Robots & rhs);
  Robots(Robots &&) = default;
  Robots & operator=(Robots &&) = default;

  /** Give access to the underlying list of Robot objects */
  std::vector<RobotPtr> & robots();
  /** Give access to the underlying list of Robot objects (const) */
  const std::vector<ConstRobotPtr> & robots() const;

  /** True if the given robot is part of this intance */
  bool hasRobot(std::string_view name) const;

  /** @name Robot(s) loading/unloading functions
   *
   * These functions allow to load or unload robot(s) from the Robots class
   *
   * @{
   */

  /** Load a single robot from a RobotModule with an optional base
   *
   * \param name Name of the new robot. Must be unique.
   *
   * \param module The RobotModule to fetch data from for this robot
   *
   * \param name Name that will identify this robot
   *
   * \param base If non-null, used as the initial transformation between the base and the world
   *
   * \param bName If empty, use the "normal" base, otherwise use body bName as base
   *
   * \throws If a robot with the same name already exists
   *
   * \returns the robot that was just loaded
   */
  RobotPtr load(const RobotModule & module,
                std::string_view name,
                const std::optional<sva::PTransformd> & base = std::nullopt,
                const std::optional<std::string_view> & bName = std::nullopt);

  /** Load a Robot from URDF content
   *
   * \param name Name of the robot
   *
   * \param urdf URDF content
   *
   * \param withVirtualLinks If true, include virtual bodies in the resulting robot
   *
   * \param filteredLinks Exclude the bodies in this list from the resulting robot
   *
   * \param fixed If true the robot has fixed base
   *
   * \param base If provided used as the initial transformation between the base and the world
   *
   * \param baseName If provided overwrite the default base choice
   */
  RobotPtr loadFromUrdf(std::string_view name,
                        const std::string & urdf,
                        bool withVirtualLinks = true,
                        const std::vector<std::string> & filteredLinks = {},
                        bool fixed = false,
                        const std::optional<sva::PTransformd> & base = std::nullopt,
                        const std::optional<std::string_view> & bName = std::nullopt);

  /** Duplicate a Robot, it doesn't have to belong to this instance of Robots
   *
   * \param robot The robot to duplicate
   *
   * \param name Name of the robot
   *
   * \throws If the robot's name is already used in this instance
   */
  RobotPtr robotCopy(const Robot & robot, std::string_view name);

  /** Remove a Robot, does nothing if the robot is not loaded
   *
   * \param name Name of the robot to remove
   */
  void removeRobot(std::string_view name);

  /** @} */
  /* End of Robot(s) loading/unloading functions group */

  /** Access a robot by name
   *
   * \param name Name of the robot
   *
   * \throws If no such robot exists within the Robots
   */
  RobotPtr robot(std::string_view name);

  /** Access a robot by name (const) */
  ConstRobotPtr robot(std::string_view name) const;

  /** @name Iterators
   *
   * These functions provide an iterator interface to Robots
   *
   * @{
   */
  iterator begin() noexcept;
  const_iterator begin() const noexcept;
  const_iterator cbegin() const noexcept;

  iterator end() noexcept;
  const_iterator end() const noexcept;
  const_iterator cend() const noexcept;

  reverse_iterator rbegin() noexcept;
  const_reverse_iterator rbegin() const noexcept;
  const_reverse_iterator crbegin() const noexcept;

  reverse_iterator rend() noexcept;
  const_reverse_iterator rend() const noexcept;
  const_reverse_iterator crend() const noexcept;
  /** @} */

  /** Number of robots
   *
   * \return The number of robots
   *
   */
  size_type size() const noexcept;

  /** Reserves space for a total number of new_cap Robots
   *
   * \param new_cap Reserve size
   *
   * Has no effect if size > new_cap
   */
  void reserve(size_type new_cap);

protected:
  std::vector<mc_rbdyn::RobotPtr> robots_;
  std::vector<mc_rbdyn::ConstRobotPtr> const_robots_;
  const_iterator getRobot(std::string_view name) const;
  RobotPtr addRobot(RobotPtr robot);
};

} // namespace mc_rbdyn
