#pragma once
#include <cstdint>

namespace map_server
{
// tdt traversability: 0 = obstacle, positive = traversable; larger = more clearance.
// Unknown is metadata (1); consumers apply their unknown-space policy explicitly.
inline constexpr uint8_t FREE_SPACE = 255;
inline constexpr uint8_t INSCRIBED_INFLATED_OBSTACLE = 2;
inline constexpr uint8_t LETHAL_OBSTACLE = 0;
inline constexpr uint8_t NO_INFORMATION = 1;
inline constexpr double NO_SPEED_LIMIT = 0.0;

// MPPI's objective uses increasing obstacle penalties, independent of grid encoding.
namespace penalty {
inline constexpr uint8_t FREE_SPACE = 0;
inline constexpr uint8_t INSCRIBED_INFLATED_OBSTACLE = 253;
inline constexpr uint8_t LETHAL_OBSTACLE = 254;
inline constexpr uint8_t NO_INFORMATION = 255;
}
inline constexpr uint8_t toObstacleCost(uint8_t value)
{
  return value == LETHAL_OBSTACLE ? penalty::LETHAL_OBSTACLE :
    value == NO_INFORMATION ? penalty::NO_INFORMATION : 255 - value;
}
inline constexpr uint8_t fromObstacleCost(uint8_t value)
{
  return value == penalty::LETHAL_OBSTACLE ? LETHAL_OBSTACLE :
    value == penalty::NO_INFORMATION ? NO_INFORMATION : 255 - value;
}
}  // namespace map_server
