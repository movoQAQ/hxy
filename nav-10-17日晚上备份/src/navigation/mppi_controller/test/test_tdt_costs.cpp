#include <gtest/gtest.h>
#include "mppi_controller/critics/cost_critic.hpp"

class CostProbe : public mppi::critics::CostCritic {
public:
  CostProbe() {
    consider_footprint_ = false; is_tracking_unknown_ = false; possible_collision_cost_ = -1;
  }
  bool collision(uint8_t raw) {return inCollision(map_server::toObstacleCost(raw), .55, .55, 0);}
  void footprint(map_server::Costmap2D * map) {
    consider_footprint_ = true; collision_checker_.setCostmap(map);
    costmap_config_.footprint.resize(4);
    auto & f = costmap_config_.footprint;
    f[0].x = -.1; f[0].y = -.1; f[1].x = .1; f[1].y = -.1;
    f[2].x = .1; f[2].y = .1; f[3].x = -.1; f[3].y = .1;
  }
};
TEST(TdtMppi, RawOccupancyIsConvertedBeforeCollisionScoring) {
  CostProbe probe;
  EXPECT_FALSE(probe.collision(map_server::FREE_SPACE));
  EXPECT_TRUE(probe.collision(map_server::LETHAL_OBSTACLE));
  EXPECT_TRUE(probe.collision(map_server::INSCRIBED_INFLATED_OBSTACLE));
  EXPECT_TRUE(probe.collision(map_server::NO_INFORMATION));
  EXPECT_LT(map_server::toObstacleCost(200), map_server::toObstacleCost(20));
}
TEST(TdtMppi, FullFootprintDetectsZeroObstacle) {
  map_server::Costmap2D map(20, 20, .1, 0, 0);
  CostProbe probe; probe.footprint(&map);
  EXPECT_FALSE(probe.collision(map_server::FREE_SPACE));
  map.setCost(4, 4, 0);
  EXPECT_TRUE(probe.collision(map_server::FREE_SPACE));
}
TEST(TdtMppi, CircumscribedThresholdRemainsAnIncreasingPenalty) {
  mppi::CostmapConfig config;
  config.robot_radius = .2; config.inflation_radius = .6; config.inflation_cost_scaling_factor = 10;
  EXPECT_EQ(config.circumscribedCost(), 253);
}
