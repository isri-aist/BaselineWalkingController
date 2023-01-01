#include <mc_rtc/logging.h>

#include <BaselineWalkingController/FootTypes.h>

using namespace BWC;

Foot BWC::strToFoot(const std::string & footStr)
{
  if(footStr == "Left")
  {
    return Foot::Left;
  }
  else if(footStr == "Right")
  {
    return Foot::Right;
  }
  else
  {
    mc_rtc::log::error_and_throw("[strToFoot] Unsupported Foot name: {}", footStr);
  }
}

Foot BWC::opposite(const Foot & foot)
{
  if(foot == Foot::Left)
  {
    return Foot::Right;
  }
  else // if(footStr == "Right")
  {
    return Foot::Left;
  }
}

int BWC::sign(const Foot & foot)
{
  if(foot == Foot::Left)
  {
    return 1;
  }
  else // if(footStr == "Right")
  {
    return -1;
  }
}

std::string std::to_string(const Foot & foot)
{
  if(foot == Foot::Left)
  {
    return std::string("Left");
  }
  else if(foot == Foot::Right)
  {
    return std::string("Right");
  }
  else
  {
    mc_rtc::log::error_and_throw("[to_string] Unsupported foot: {}", std::to_string(static_cast<int>(foot)));
  }
}

std::string std::to_string(const SupportPhase & phase)
{
  if(phase == SupportPhase::DoubleSupport)
  {
    return std::string("DoubleSupport");
  }
  else if(phase == SupportPhase::LeftSupport)
  {
    return std::string("LeftSupport");
  }
  else if(phase == SupportPhase::RightSupport)
  {
    return std::string("RightSupport");
  }
  else
  {
    mc_rtc::log::error_and_throw("[to_string] Unsupported support phase: {}", std::to_string(static_cast<int>(phase)));
  }
}
