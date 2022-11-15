#pragma once

#include <mc_control/Configuration.h>

namespace BWC
{
/** \brief Overwrite the controller configuration.
    \param config original configuration
    \param robotName robot name

    Extract the configurations specific to robotName and OverwriteConfigKeys from OverwriteConfigList and overwrite the
   original configuration.
 */
inline mc_rtc::Configuration overwriteConfig(const mc_rtc::Configuration & config, const std::string & robotName)
{
  mc_rtc::Configuration newConfig;
  newConfig.load(config);

  if(newConfig.has("OverwriteConfigKeys") && !newConfig.has("OverwriteConfigList"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[overwriteConfig] OverwriteConfigKeys is set but OverwriteConfigList is not defined.");
    return newConfig;
  }

  // Set overwrite key
  std::vector<std::string> overwriteConfigKeys = {robotName};
  if(newConfig.has("OverwriteConfigKeys"))
  {
    for(const std::string & overwriteConfigKey : newConfig("OverwriteConfigKeys"))
    {
      overwriteConfigKeys.push_back(overwriteConfigKey);
    }
  }

  // Overwrite configuration
  for(const std::string & overwriteConfigKey : overwriteConfigKeys)
  {
    if(!newConfig("OverwriteConfigList").has(overwriteConfigKey))
    {
      if(overwriteConfigKey != robotName)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("[overwriteConfig] Overwrite configuration {} is not found.",
                                                         overwriteConfigKey);
      }
      continue;
    }

    newConfig.load(newConfig("OverwriteConfigList")(overwriteConfigKey));
    mc_rtc::log::success("[overwriteConfig] Overwrite with configuration {}.", overwriteConfigKey);
  }

  return newConfig;
}
} // namespace BWC
