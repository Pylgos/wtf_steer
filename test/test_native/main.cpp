#include "../../src/anglelib.hpp"
#include "../../src/steer_4w_controller.hpp"
#include <stdio.h>
#include <unity.h>


using namespace anglelib;


void test_anglelib() {
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(M_PI/2).direction().rad(), Directionf::from_rad(M_PI/2).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(M_PI).direction().rad(), Directionf::from_rad(M_PI).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(3*M_PI/2).direction().rad(), Directionf::from_rad(3*M_PI/2).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(2*M_PI).direction().rad(), Directionf::zero().rad());

  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(2*M_PI + M_PI/2).direction().rad(), Directionf::from_rad(M_PI/2).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(2*M_PI + M_PI).direction().rad(), Directionf::from_rad(M_PI).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(2*M_PI + 3*M_PI/2).direction().rad(), Directionf::from_rad(3*M_PI/2).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(2*M_PI + 2*M_PI).direction().rad(), Directionf::zero().rad());

  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(4*M_PI + M_PI/2).direction().rad(), Directionf::from_rad(M_PI/2).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(4*M_PI + M_PI).direction().rad(), Directionf::from_rad(M_PI).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(4*M_PI + 3*M_PI/2).direction().rad(), Directionf::from_rad(3*M_PI/2).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(4*M_PI + 2*M_PI).direction().rad(), Directionf::zero().rad());

  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(-2*M_PI + M_PI/2).direction().rad(), Directionf::from_rad(M_PI/2).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(-2*M_PI + M_PI).direction().rad(), Directionf::from_rad(M_PI).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(-2*M_PI + 3*M_PI/2).direction().rad(), Directionf::from_rad(3*M_PI/2).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_rad(-2*M_PI + 2*M_PI).direction().rad(), Directionf::zero().rad());

  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(0) + Anglef::from_deg(180)).rad(), Directionf::from_deg(180).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(90) + Anglef::from_deg(180)).rad(), Directionf::from_deg(270).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(180) + Anglef::from_deg(180)).rad(), Directionf::from_deg(0).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(270) + Anglef::from_deg(180)).rad(), Directionf::from_deg(90).rad());

  TEST_ASSERT_EQUAL_FLOAT(Directionf::from_xy(1, 0).rad(), Directionf::from_deg(0).rad());
  TEST_ASSERT_EQUAL_FLOAT(Directionf::from_xy(1, 1).rad(), Directionf::from_deg(45).rad());
  TEST_ASSERT_EQUAL_FLOAT(Directionf::from_xy(0, 1).rad(), Directionf::from_deg(90).rad());
  TEST_ASSERT_EQUAL_FLOAT(Directionf::from_xy(-1, 1).rad(), Directionf::from_deg(135).rad());
  TEST_ASSERT_EQUAL_FLOAT(Directionf::from_xy(-1, 0).rad(), Directionf::from_deg(180).rad());
  TEST_ASSERT_EQUAL_FLOAT(Directionf::from_xy(-1, -1).rad(), Directionf::from_deg(225).rad());
  TEST_ASSERT_EQUAL_FLOAT(Directionf::from_xy(0, -1).rad(), Directionf::from_deg(270).rad());
  TEST_ASSERT_EQUAL_FLOAT(Directionf::from_xy(1, -1).rad(), Directionf::from_deg(315).rad());

  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(0).closest_angle_of(Directionf::from_deg(0)).rad(), Anglef::from_deg(0).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(0).closest_angle_of(Directionf::from_deg(90)).rad(), Anglef::from_deg(90).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(0).closest_angle_of(Directionf::from_deg(179)).rad(), Anglef::from_deg(179).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(0).closest_angle_of(Directionf::from_deg(181)).rad(), Anglef::from_deg(-179).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(0).closest_angle_of(Directionf::from_deg(270)).rad(), Anglef::from_deg(-90).rad());

  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(360).closest_angle_of(Directionf::from_deg(0)).rad(), Anglef::from_deg(360).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(360).closest_angle_of(Directionf::from_deg(90)).rad(), Anglef::from_deg(360 + 90).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(360).closest_angle_of(Directionf::from_deg(179)).rad(), Anglef::from_deg(360 + 179).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(360).closest_angle_of(Directionf::from_deg(181)).rad(), Anglef::from_deg(360 + -179).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(360).closest_angle_of(Directionf::from_deg(270)).rad(), Anglef::from_deg(360 + -90).rad());

  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(720).closest_angle_of(Directionf::from_deg(0)).rad(), Anglef::from_deg(720).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(720).closest_angle_of(Directionf::from_deg(90)).rad(), Anglef::from_deg(720 + 90).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(720).closest_angle_of(Directionf::from_deg(179)).rad(), Anglef::from_deg(720 + 179).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(720).closest_angle_of(Directionf::from_deg(181)).rad(), Anglef::from_deg(720 + -179).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(720).closest_angle_of(Directionf::from_deg(270)).rad(), Anglef::from_deg(720 + -90).rad());

  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(-360).closest_angle_of(Directionf::from_deg(0)).rad(), Anglef::from_deg(-360).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(-360).closest_angle_of(Directionf::from_deg(90)).rad(), Anglef::from_deg(-360 + 90).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(-360).closest_angle_of(Directionf::from_deg(179)).rad(), Anglef::from_deg(-360 + 179).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(-360).closest_angle_of(Directionf::from_deg(181)).rad(), Anglef::from_deg(-360 + -179).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(-360).closest_angle_of(Directionf::from_deg(270)).rad(), Anglef::from_deg(-360 + -90).rad());

  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(-720).closest_angle_of(Directionf::from_deg(0)).rad(), Anglef::from_deg(-720).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(-720).closest_angle_of(Directionf::from_deg(90)).rad(), Anglef::from_deg(-720 + 90).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(-720).closest_angle_of(Directionf::from_deg(179)).rad(), Anglef::from_deg(-720 + 179).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(-720).closest_angle_of(Directionf::from_deg(181)).rad(), Anglef::from_deg(-720 + -179).rad());
  TEST_ASSERT_EQUAL_FLOAT(Anglef::from_deg(-720).closest_angle_of(Directionf::from_deg(270)).rad(), Anglef::from_deg(-720 + -90).rad());

  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(0).angle_to(Directionf::from_deg(0))).rad(), Anglef::from_deg(0).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(0).angle_to(Directionf::from_deg(90))).rad(), Anglef::from_deg(90).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(0).angle_to(Directionf::from_deg(179))).rad(), Anglef::from_deg(179).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(0).angle_to(Directionf::from_deg(181))).rad(), Anglef::from_deg(-179).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(0).angle_to(Directionf::from_deg(270))).rad(), Anglef::from_deg(-90).rad());

  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(90).angle_to(Directionf::from_deg(0))).rad(), Anglef::from_deg(-90).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(90).angle_to(Directionf::from_deg(90))).rad(), Anglef::from_deg(0).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(90).angle_to(Directionf::from_deg(179))).rad(), Anglef::from_deg(89).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(90).angle_to(Directionf::from_deg(181))).rad(), Anglef::from_deg(91).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(90).angle_to(Directionf::from_deg(270))).rad(), Anglef::from_deg(180).rad());

  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(180).angle_to(Directionf::from_deg(0))).rad(), Anglef::from_deg(180).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(180).angle_to(Directionf::from_deg(90))).rad(), Anglef::from_deg(-90).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(180).angle_to(Directionf::from_deg(180))).rad(), Anglef::from_deg(0).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(180).angle_to(Directionf::from_deg(270))).rad(), Anglef::from_deg(90).rad());

  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(270).angle_to(Directionf::from_deg(0))).rad(), Anglef::from_deg(90).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(270).angle_to(Directionf::from_deg(90))).rad(), Anglef::from_deg(-180).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(270).angle_to(Directionf::from_deg(180))).rad(), Anglef::from_deg(-90).rad());
  TEST_ASSERT_EQUAL_FLOAT((Directionf::from_deg(270).angle_to(Directionf::from_deg(270))).rad(), Anglef::from_deg(0).rad());
}


int main() {
  UNITY_BEGIN();
  RUN_TEST(test_anglelib);
  UNITY_END();

  return 0;
}