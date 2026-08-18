// Unit tests for T0-R3 (AC0-3): hand-computed cases, see each test's comment.
// Geometry under test: L=1.0 m, W=0.6 m, r=0.15 m, max_steer=0.6 rad.
#include <gtest/gtest.h>
#include "rover_kinematics/double_ackermann.hpp"

using namespace rover_kinematics;

class DoubleAckermannTest : public ::testing::Test {
 protected:
  Config cfg{1.0, 0.6, 0.15, 0.6};
  DoubleAckermann k{cfg};
  WheelCommands w;
};

TEST_F(DoubleAckermannTest, MinTurningRadiusDerivedFromGeometry) {
  // R_min = (L/2)/tan(max_steer) + W/2
  EXPECT_NEAR(k.min_turning_radius(), 0.5 / std::tan(0.6) + 0.3, 1e-12);
}

TEST_F(DoubleAckermannTest, StraightLine) {
  ASSERT_TRUE(k.ik({1.5, 0.0}, w));
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(w.steer_rad[i], 0.0, 1e-12);
    EXPECT_NEAR(w.omega_rads[i], 1.5 / 0.15, 1e-12);  // 10 rad/s
  }
}

TEST_F(DoubleAckermannTest, PureArcLeft) {
  // vx=1, wz=0.5 -> R=2 m, ICR at (0, +2).
  // FL: atan(0.5/(2-0.3)) ; FR: atan(0.5/(2+0.3)) ; rears mirrored negative.
  // |v_i| = wz * dist(ICR, wheel); omega = v/r.
  ASSERT_TRUE(k.ik({1.0, 0.5}, w));
  EXPECT_NEAR(w.steer_rad[0],  std::atan(0.5 / 1.7), 1e-9);
  EXPECT_NEAR(w.steer_rad[1],  std::atan(0.5 / 2.3), 1e-9);
  EXPECT_NEAR(w.steer_rad[2], -std::atan(0.5 / 1.7), 1e-9);
  EXPECT_NEAR(w.steer_rad[3], -std::atan(0.5 / 2.3), 1e-9);
  EXPECT_NEAR(w.omega_rads[0], 0.5 * std::hypot(0.5, 1.7) / 0.15, 1e-9);
  EXPECT_NEAR(w.omega_rads[1], 0.5 * std::hypot(0.5, 2.3) / 0.15, 1e-9);
  EXPECT_LT(w.omega_rads[0], w.omega_rads[1]);  // inner wheel slower
}

TEST_F(DoubleAckermannTest, PureArcRightMirrorsLeft) {
  WheelCommands wl, wr;
  ASSERT_TRUE(k.ik({1.0,  0.5}, wl));
  ASSERT_TRUE(k.ik({1.0, -0.5}, wr));
  EXPECT_NEAR(wr.steer_rad[0], -wl.steer_rad[1], 1e-9);
  EXPECT_NEAR(wr.steer_rad[1], -wl.steer_rad[0], 1e-9);
  EXPECT_NEAR(wr.omega_rads[0], wl.omega_rads[1], 1e-9);
}

TEST_F(DoubleAckermannTest, ReverseArcKeepsSteerFlipsDrive) {
  WheelCommands fwd, bwd;
  ASSERT_TRUE(k.ik({ 1.0,  0.5}, fwd));  // R = +2
  ASSERT_TRUE(k.ik({-1.0, -0.5}, bwd));  // R = +2, driving backwards
  EXPECT_NEAR(bwd.steer_rad[0], fwd.steer_rad[0], 1e-9);
  EXPECT_LT(bwd.omega_rads[0], 0.0);
}

TEST_F(DoubleAckermannTest, ZeroTwistIsAllZero) {
  ASSERT_TRUE(k.ik({0.0, 0.0}, w));
  for (int i = 0; i < 4; ++i) EXPECT_NEAR(w.omega_rads[i], 0.0, 1e-12);
}

TEST_F(DoubleAckermannTest, PointTurnRejectedByDesign) {
  // §4.2: point-turn not used in base navigation; IK must refuse, not invent output.
  EXPECT_FALSE(k.ik({0.0, 1.0}, w));
}

TEST_F(DoubleAckermannTest, BelowMinRadiusRejected) {
  EXPECT_FALSE(k.ik({1.0, 1.0}, w));  // R=1 < ~1.031
}

TEST_F(DoubleAckermannTest, ForwardInverseRoundTrip) {
  ASSERT_TRUE(k.ik({0.8, 0.3}, w));
  const BodyTwist t = k.fk(w);
  EXPECT_NEAR(t.vx, 0.8, 1e-9);
  EXPECT_NEAR(t.wz, 0.3, 1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
