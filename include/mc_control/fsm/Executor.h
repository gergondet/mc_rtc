#pragma once

#include <mc_control/api.h>
#include <mc_control/fsm/State.h>
#include <mc_control/fsm/StateFactory.h>
#include <mc_control/fsm/TransitionMap.h>

#include <memory>

namespace mc_control
{

namespace fsm
{

struct Controller;

/** \class Executor takes care of executing an FSM
 *
 * The executor can works in two ways:
 *
 * - managed: handles transitions through an external trigger
 *
 * - self-managed: handles transitions thanks to a TransitionMap
 *
 */
struct MC_CONTROL_DLLAPI Executor
{
  /** Initialize the executor
   *
   * \param ctl Controller using this executor
   *
   * \param config Configuration of thi executor
   *
   */
  void init(Controller & ctl, const mc_rtc::Configuration & config);

  /** Run the current state
   *
   * \param ctl Controller using this executor
   *
   * \param keep_state If true, keep the state when it finishes its run
   *
   * \returns True if the execution is complete
   *
   */
  bool run(Controller & ctl, bool keep_state);

  /** Trigger an interruption */
  inline void interrupt() { interrupt_triggered_ = true; }

  /** Returns true if the state is active */
  inline bool running() const { return state_ != nullptr; }

  /** Returns true if the executor is ready for next transition */
  inline bool ready() const { return ready_; }

  /** Resume execution to a given state
   *
   * Interrupt current state execution if needed
   */
  bool resume(const std::string & state);

  /** Trigger next state
   *
   * \returns False if the FSM is not ready for next state
   */
  bool next();

  /** Returns the current state's name */
  const std::string & state() const { return curr_state_; }

  /** Returns the latest state's output */
  const std::string & output() const { return state_output_; }

  /** Pass message to current state (read-only) */
  bool read_msg(std::string & msg);

  /** Pass message to current state (read-write) */
  bool read_write_msg(std::string & msg, std::string & out);
private:
  /** Configuration passed at construction, can hold specific states' configuration */
  mc_rtc::Configuration config_;
  /** True if managed */
  bool managed_ = false;
  /** If true and not managed, waits for trigger before transitions */
  bool step_by_step_ = true;

  /** Transition map, empty if managed */
  TransitionMap transition_map_;

  /** Current state */
  StatePtr state_ = nullptr;
  /** Curent state (name) */
  std::string curr_state_ = "";
  /** State output */
  std::string state_output_ = "";

  /** If true, the state has been interrupted */
  bool interrupt_triggered_ = false;
  /** If true, executor is ready for next transition */
  bool ready_ = true;
  /** If true, transition has been triggered */
  bool transition_triggered_ = false;
  /** Name of the next state */
  std::string next_state_ = "";
private:
  /** Complete execution */
  bool complete(Controller & ctl, bool keep_state);

  /** Setup next state */
  void next(Controller & ctl);
};

} // namespace fsm

} // namespace mc_control
