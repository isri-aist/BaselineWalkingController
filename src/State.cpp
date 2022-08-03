#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/State.h>

using namespace BWC;

void State::start(mc_control::fsm::Controller & _ctl)
{
  ctlPtr_ = &static_cast<BaselineWalkingController &>(_ctl);
}
