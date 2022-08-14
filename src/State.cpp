#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/State.h>

using namespace BWC;

void State::start(mc_control::fsm::Controller & _ctl)
{
  ctlPtr_ = &static_cast<BaselineWalkingController &>(_ctl);

  // Setup PostureTask
  // Note that the target of PostureTask is updated to the current position automatically when the state switches
  // https://github.com/jrl-umi3218/mc_rtc/blob/b2fe81b4f418c8251d85d3ceee974c0ba7e0610a/src/mc_control/fsm/Executor.cpp#L203
  if(ctl().config().has("PostureTask"))
  {
    auto postureTask = ctl().getPostureTask(ctl().robot().name());
    postureTask->load(ctl().solver(), ctl().config()("PostureTask"));
  }
}
