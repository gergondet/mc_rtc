/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/SimulationContactPair.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/AddRemoveContact.h>

#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/configuration_io.h>

namespace mc_control
{

namespace fsm
{

namespace
{

template<typename T>
struct always_false : public std::false_type
{
};

} // namespace

enum class ContactOp
{
  Remove = 0,
  Add = 1,
  AddCompliant = 2
};

template<ContactOp>
struct ContactTraits
{
  using task_t = mc_tasks::AddRemoveContactTask;
};

template<>
struct ContactTraits<ContactOp::AddCompliant>
{
  using task_t = mc_tasks::force::ComplianceTask;
};

template<ContactOp op>
struct AddRemoveContactStateImplHelper
{
  using task_t = typename ContactTraits<op>::task_t;
  static void make_run(AddRemoveContactStateImpl & impl, Controller & ctl, mc_rbdyn::Contact & contact);

  static void make_run_impl(AddRemoveContactStateImpl &, Controller &, mc_rbdyn::Contact &);
};

template<>
void AddRemoveContactStateImplHelper<ContactOp::Remove>::make_run_impl(AddRemoveContactStateImpl & impl,
                                                                       Controller & ctl,
                                                                       mc_rbdyn::Contact & contact);

template<>
void AddRemoveContactStateImplHelper<ContactOp::Add>::make_run_impl(AddRemoveContactStateImpl & impl,
                                                                    Controller & ctl,
                                                                    mc_rbdyn::Contact & contact);

template<>
void AddRemoveContactStateImplHelper<ContactOp::AddCompliant>::make_run_impl(AddRemoveContactStateImpl & impl,
                                                                             Controller & ctl,
                                                                             mc_rbdyn::Contact & contact);

struct AddRemoveContactStateImpl
{
  mc_rtc::Configuration config_;
  std::shared_ptr<mc_tasks::MetaTask> task_ = nullptr;
  std::shared_ptr<mc_tasks::CoMTask> com_task_ = nullptr;
  bool useCoM_ = true;
  std::function<bool(AddRemoveContactStateImpl &, Controller &)> run_ = [](AddRemoveContactStateImpl &, Controller &) {
    return true;
  };
  void start(Controller & ctl)
  {
    mc_rbdyn::Contact contact = config_("contact");
    config_("useCoM", useCoM_);
    if(useCoM_)
    {
      com_task_ = std::make_shared<mc_tasks::CoMTask>(ctl.robot(contact.r1));
      if(config_.has("com"))
      {
        com_task_->load(ctl.solver(), config_("com"));
      }
    }
    std::string type = config_("type");
    bool removeContact = (type == "removeContact");
    bool isCompliant = (type == "compliance");
    if(isCompliant)
    {
      const auto & r1 = ctl.robot(contact.r1);
      if(r1.frame(contact.r1Surface).hasForceSensor())
      {
        config_.add("robot", contact.r1);
        config_.add("frame", contact.r1Surface);
      }
      else
      {
        mc_rtc::log::error(
            "AddRemoveContactState configured with compliant task but surface {} does not have a force sensor.",
            contact.r1Surface);
        mc_rtc::log::warning("Defaulting to simulated contact sensor");
        config_.add("type", "addContact");
        isCompliant = false;
        if(!config_.has("stiffness"))
        {
          config_.add("stiffness", 2.0);
        }
        if(!config_.has("weight"))
        {
          config_.add("weight", 1000.0);
        }
        if(!config_.has("speed"))
        {
          config_.add("speed", 0.01);
        }
      }
    }
    bool hasContact = ctl.hasContact(contact);
    if((hasContact && removeContact) || (!hasContact && !removeContact))
    {
      if(removeContact)
      {
        AddRemoveContactStateImplHelper<ContactOp::Remove>::make_run(*this, ctl, contact);
      }
      else
      {
        if(isCompliant)
        {
          AddRemoveContactStateImplHelper<ContactOp::AddCompliant>::make_run(*this, ctl, contact);
        }
        else
        {
          AddRemoveContactStateImplHelper<ContactOp::Add>::make_run(*this, ctl, contact);
        }
      }
      if(useCoM_)
      {
        com_task_->name(fmt::format("{}_{}_{}_com", removeContact ? "RemoveContact" : "AddContact", contact.r1Surface,
                                    contact.r2Surface));
        ctl.solver().addTask(com_task_);
      }
    }
    else
    {
      mc_rtc::log::info("AddRemoveContactState has nothing to do here");
    }
  }
};

template<ContactOp op>
void AddRemoveContactStateImplHelper<op>::make_run(AddRemoveContactStateImpl & impl,
                                                   Controller & ctl,
                                                   mc_rbdyn::Contact & contact)
{
  auto t = mc_tasks::MetaTaskLoader::load<task_t>(ctl.solver(), impl.config_);
  impl.task_ = t;
  ctl.solver().addTask(impl.task_);
  make_run_impl(impl, ctl, contact);
}

template<>
void AddRemoveContactStateImplHelper<ContactOp::Remove>::make_run_impl(AddRemoveContactStateImpl & impl,
                                                                       Controller & ctl,
                                                                       mc_rbdyn::Contact & contact)
{
  ctl.removeContact(contact);
  double distance_ = impl.config_("distance", 0.075);
  auto & r1_ = ctl.robot(contact.r1);
  auto r1Surface_ = contact.r1Surface;
  auto init_pos_ = r1_.frame(contact.r1Surface).position().translation();
  bool reached_ = false;
  impl.run_ = [distance_, &r1_, r1Surface_, init_pos_, reached_](AddRemoveContactStateImpl & impl,
                                                                 Controller & ctl) mutable {
    const auto & pos = r1_.frame(r1Surface_).position().translation();
    auto d = (pos - init_pos_).norm();
    if(d >= distance_)
    {
      if(!reached_)
      {
        reached_ = true;
        ctl.solver().removeTask(impl.task_);
        auto t = std::static_pointer_cast<mc_tasks::AddRemoveContactTask>(impl.task_);
        auto w = t->weight();
        auto s = t->stiffness();
        impl.task_ = std::make_shared<mc_tasks::TransformTask>(r1_.frame(r1Surface_), s, w);
        ctl.solver().addTask(impl.task_);
      }
    }
    return d >= distance_;
  };
}

template<>
void AddRemoveContactStateImplHelper<ContactOp::Add>::make_run_impl(AddRemoveContactStateImpl & impl,
                                                                    Controller & ctl,
                                                                    mc_rbdyn::Contact & contact)
{
  auto & r1_ = ctl.robot(contact.r1);
  auto & r2_ = ctl.robot(contact.r2);
  auto sensor_ = SimulationContactPair(r1_.surface(contact.r1Surface), r2_.surface(contact.r2Surface));
  auto forceThreshold_ = impl.config_("forceThreshold", std::numeric_limits<double>::infinity());
  bool forceOnly_ = impl.config_("forceOnly", false);
  size_t forceThresholdIter_ = static_cast<size_t>(impl.config_("forceThresholdIter", 3));
  bool hasForceSensor_ = r1_.frame(contact.r1Surface).hasForceSensor();
  auto forceSensorName_ = hasForceSensor_ ? r1_.frame(contact.r1Surface).forceSensor().name() : "";
  size_t forceIter_ = 0;
  auto contact_ = contact;
  bool done_ = false;
  impl.run_ = [contact_, sensor_, hasForceSensor_, forceThreshold_, forceSensorName_, forceOnly_, forceThresholdIter_,
               forceIter_, done_](AddRemoveContactStateImpl & impl, Controller & ctl) mutable {
    if(done_)
    {
      return true;
    }
    auto d = sensor_.update();
    if(hasForceSensor_ && ctl.robot().forceSensor(forceSensorName_).force().z() > forceThreshold_)
    {
      forceIter_++;
    }
    else
    {
      forceIter_ = 0;
    }
    if((!forceOnly_ && d <= 0) || forceIter_ > forceThresholdIter_)
    {
      if(!done_)
      {
        if(d <= 0)
        {
          mc_rtc::log::info("Geometric contact detected");
        }
        else
        {
          mc_rtc::log::info("Force contact detected");
        }
        ctl.addContact(contact_);
        auto t = std::static_pointer_cast<mc_tasks::AddRemoveContactTask>(impl.task_);
        t->speed(0.0);
        done_ = true;
      }
      return true;
    }
    return false;
  };
}

template<>
void AddRemoveContactStateImplHelper<ContactOp::AddCompliant>::make_run_impl(AddRemoveContactStateImpl & impl,
                                                                             Controller &,
                                                                             mc_rbdyn::Contact & contact)
{
  auto contact_ = contact;
  bool done_ = false;
  double vel_thresh_ = impl.config_("velocity", 1e-4);
  impl.run_ = [contact_, done_, vel_thresh_](AddRemoveContactStateImpl & impl, Controller & ctl) mutable {
    auto t = std::static_pointer_cast<mc_tasks::force::ComplianceTask>(impl.task_);
    if(t->speed().norm() < vel_thresh_ && t->eval().norm() < t->getTargetWrench().vector().norm() / 2 && !done_)
    {
      ctl.addContact(contact_);
      done_ = true;
      return true;
    }
    return false;
  };
}

AddRemoveContactState::AddRemoveContactState() : impl_(new AddRemoveContactStateImpl()) {}

AddRemoveContactState::~AddRemoveContactState() {}

void AddRemoveContactState::configure(const mc_rtc::Configuration & config)
{
  impl_->config_.load(config);
}

void AddRemoveContactState::start(Controller & ctl)
{
  impl_->start(ctl);
}

bool AddRemoveContactState::run(Controller & ctl)
{
  if(impl_->run_(*impl_, ctl))
  {
    output("OK");
    return true;
  }
  return false;
}

void AddRemoveContactState::teardown(Controller & ctl)
{
  if(impl_->task_)
  {
    ctl.solver().removeTask(impl_->task_);
    if(impl_->useCoM_)
    {
      ctl.solver().removeTask(impl_->com_task_);
    }
  }
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("AddRemoveContact", mc_control::fsm::AddRemoveContactState)
