/**
 * @file    test.cpp
 * @author  nrparikh
 * @copyright GNU public license
 *
 * @brief Test cases for the implementation of PID controller.
 *
 */

#include <gtest/gtest.h>
#include "pid_controller.hpp"

/**
 * @brief Test if the initialization functions work
 */
TEST(controllerTest, InitializationTest) {
  PIDController control;
  EXPECT_EQ(control.getKProp(), 0);
  EXPECT_EQ(control.getKIntegral(), 0);
  EXPECT_EQ(control.getKDerivative(), 0);
}

/**
 * @brief First error test. Check the returned error up to 4th decimal place
 */
TEST(controllerTest, errorTest1) {
  PIDController controller(0.01, 0.001, 0.01, 0.1, 0);
  EXPECT_NEAR(controller.controller(10.0), 0.003034, 0.0001);
}

/**
 * @brief Second error test. Check the returned error up to 4th decimal place
 */
TEST(controllerTest, errorTest2) {
  PIDController controller(0.01, 0.001, 0.01, 0.1, 0);
  EXPECT_NEAR(controller.controller(5.0), -0.00288725, 0.0001);
}
