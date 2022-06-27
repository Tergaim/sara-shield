#include <gtest/gtest.h>
#include <math.h> 

#include <eigen3/Eigen/Dense>
#include "spdlog/spdlog.h" 

#include "robot_reach_fixture.h"
#include "safety_shield/robot_reach.h"

namespace safety_shield {

TEST_F(RobotReachTest, InitializationTest){
  EXPECT_DOUBLE_EQ(0, 0);
}

TEST_F(RobotReachTest, ForwardKinematicTest){
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  robot_reach_->forwardKinematic(0.0, 0, result);
  Eigen::Matrix4d expect;
  expect << 0, -1, 0, 0,
            1, 0 , 0, 0,
            0, 0 , 1, 0.1,
            0, 0 , 0, 1;
  EXPECT_TRUE(result.isApprox(expect));
}

} // namespace safety_shield

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}