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
 * @brief First initialization test to check return functions
 */
TEST(controllerTest, InitializationTest1) {
  PIDController control;

  //tests getKProp
  EXPECT_EQ(control.getKProp(), 0);

  //tests getKIntegral
  EXPECT_EQ(control.getKIntegral(), 0);

  //tests getKDerivative
  EXPECT_EQ(control.getKDerivative(), 0);
}

/**
 * @brief Second initialization function to check value set functions
 */
TEST(controllerTest, InitializationTest2) {
  PIDController control;
  control.setKProp(10);
  control.setKIntegral(10);
  control.setKDerivative(10);
  EXPECT_EQ(control.getKProp(), 10);
  EXPECT_EQ(control.getKIntegral(), 10);
  EXPECT_EQ(control.getKDerivative(), 10);
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

/**
 * @brief test to cover get methods
 * -Added by Jessica Howard
 */
TEST(controllerTest, getTests) {
  PIDController controller(0.01, 0.001, 0.01, 0.1, 0);
  EXPECT_FLOAT_EQ(controller.getTimeInterval(), 0.1);
}

/**
 * @brief test to cover set methods
 * -Added by Jessica Howard
 */
TEST(controllerTest, setTests) {
  PIDController controller;
  controller.setTimeInterval(0.5);
  EXPECT_FLOAT_EQ(controller.getTimeInterval(), 0.5);
  controller.setCurrentPoint(5.3);
  EXPECT_FLOAT_EQ(controller.getCurrentPoint(), 5.3);
}

/**
 * @brief test to make sure the control signal is min/max capped
 * -Added by Jessica Howard
 */

TEST(controllerTest, controlBounding) {
  PIDController controller(0.01, 0.001, 0.01, 0.1, 0);
  controller.controller(1000);
  EXPECT_NEAR(controller.getCurrentPoint(), 1000, 0.01);
}
