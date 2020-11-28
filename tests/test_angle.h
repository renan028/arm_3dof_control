#pragma once

#include <gtest/gtest.h> 
#include <types.h>

using namespace remy_robot_control;

TEST(Angle, methods) 
{
  Angled a(0);
  ASSERT_FLOAT_EQ(0, a());

  a = 1.57;
  ASSERT_FLOAT_EQ(1.57, a());

  a = -4;
  ASSERT_FLOAT_EQ(2 * M_PI - 4, a());

  a = 8;
  ASSERT_FLOAT_EQ(8 - 2 * M_PI, a());

  a = 13;
  ASSERT_FLOAT_EQ(13 - 4 * M_PI, a());
  
  a += a;
  ASSERT_FLOAT_EQ(26 - 8 * M_PI, a());

  Angled b(0, - M_PI_2, M_PI_2);
  ASSERT_FLOAT_EQ(0, b());

  b = 1.57;
  ASSERT_FLOAT_EQ(1.57, b());

  b = 1.60;
  ASSERT_FLOAT_EQ(M_PI_2, b());

  b = -4;
  ASSERT_FLOAT_EQ(M_PI_2, b());

  b = 7;
  ASSERT_FLOAT_EQ(7 - 2 * M_PI, b());

  ASSERT_FLOAT_EQ(1.57, a(1.57));
}