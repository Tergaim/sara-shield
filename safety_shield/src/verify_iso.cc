#include "safety_shield/verify_iso.h"

namespace safety_shield {


bool VerifyISO::robotObstacleCollision(const std::vector<reach_lib::Cylinder>& robot_cylinder, 
      const std::vector<reach_lib::Cylinder>& obstacle_cylinder) {
  // Check position cylinder
  for(auto& obstacle_cylinder : obstacle_cylinder) {
    for (auto& robot_cylinder : robot_cylinder) {
      // If there is a collision, return true
      if (cylinderCollisionCheck(robot_cylinder, obstacle_cylinder)) {
        return true;
      }
    }
  }
  return false;
}

bool VerifyISO::verify_obstacle_reach(const std::vector<reach_lib::Cylinder>& robot_cylinder, 
      std::vector<std::vector<reach_lib::Cylinder>> obstacle_cylinder) {
  try {
    for (const auto& cylinder_list : obstacle_cylinder) {
      // If no collision occured, we are safe and don't have to check the rest.
      if(!robotObstacleCollision(robot_cylinder, cylinder_list)) {
        return true;
      }
    }
    return false;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in VerifyISO::verify_obstacle_reach: {}", exc.what());
    return false;
  }
}
} // namespace safety_shield