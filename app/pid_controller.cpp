/**
 * @file    pid_controller.cpp
 * @author  Jessica Howard
 * @copyright GNU public license
 *
 * @brief Source file for the class PIDController
 *
 * @section DESCRIPTION
 * The PIDController class has variables for each of the term as well  as time
 * interval. It also has methods for setting and retrieving the values of each
 * of the constants. It also has the method to implement PID controller to
 * achieve the desired goal.
 *
 */

#include <cmath>
#include "pid_controller.hpp"

/**
 * The PIDController class has variables for each of the term as well  as time
 * interval. It also has methods for setting and retrieving the values of each
 * of the constants. It also has the method to implement PID controller to
 * achieve the desired goal.
 */
PIDController::PIDController() {
  k_prop_ = 0;
  k_integral_ = 0;
  k_derivative_ = 0;
  time_interval_ = 1;
  current_point_ = 1;
  cum_error_ = 0;
  prev_error_ = 0;
}

/**
 * @brief Constructor for the class PIDController
 * @param k_prop: Proportional constant
 * @param k_integral: Integral constant
 * @param k_derivative: Derivative constant
 * @param time_interval: Time step
 * @param curr_point: Current point
 */
PIDController::PIDController(float k_prop, float k_integral, float k_derivative,
                             float time_interval, float curr_point) {
  /**
   * set the class variables to the parameter values
   */
  k_prop_ = k_prop;
  k_integral_ = k_integral;
  k_derivative_ = k_derivative;
  time_interval_ = time_interval;
  current_point_ = curr_point;

  /**
   * set the error tracking variables to zero
   */
  cum_error_ = 0;
  prev_error_ = 0;
}

/**
 * @brief Destructor for the class PIDController
 */
PIDController::~PIDController() {
}

/**
 * @brief Method to get the proportional constant
 * @return Return proportional constant
 */
auto PIDController::getKProp() -> float {
  return k_prop_;
}

/**
 * @brief Method to get integral constant
 * @return Return integral constant
 */
auto PIDController::getKIntegral() -> float {
  return k_integral_;
}

/**
 * @brief Method to get derivative constant
 * @return Return derivative constant
 */
auto PIDController::getKDerivative() -> float {
  return k_derivative_;
}

/**
 * @brief Method to get time step
 * @return Return the time step
 */
auto PIDController::getTimeInterval() -> float {
  return time_interval_;
}

/**
 * @brief Method to get current point
 * @return Return current point
 */
auto PIDController::getCurrentPoint() -> float {
  return current_point_;
}

/**
 * @brief Method to set the proportional constant
 * @param k_prop: Value to be set as proportional constant
 * @return Return nothing
 */
auto PIDController::setKProp(float k_prop) -> void {
  k_prop_ = k_prop;
}

/**
 * @brief Method to set the integral constant
 * @param k_integral: Value to be set as integral constant
 * @return Return nothing
 */
auto PIDController::setKIntegral(float k_integral) -> void {
  k_integral_ = k_integral;
}

/**
 * @brief Method to set derivative constant
 * @param k_derivative: Value to be set as derivative constant
 * @return Return nothing
 */
auto PIDController::setKDerivative(float k_derivative) -> void {
  k_derivative_ = k_derivative;
}

/**
 * @brief Method to set time step
 * @param time_interval: Value to be set as time interval
 * @return Return nothing
 */
auto PIDController::setTimeInterval(float time_interval) -> void {
  time_interval_ = time_interval;
}

/**
 * @brief Method to set current velocity
 * @param curr_point: Value to set as current velocity
 * @return Return nothing
 */
auto PIDController::setCurrentPoint(float curr_point) -> void {
  current_point_ = curr_point;
}

/**
 * @brief Method which implements PID controller to achieve desired point from
 * the current point
 * @param desired_point: Desired point which has to be reached
 * @param current_point: Current point
 * @return Return nothing
 */
auto PIDController::controller(float desired_point) -> float {
  // TODO(jeshoward): Use PID control method to reach desired pointr
  // Check: Use loop while squared error is greater than 0.0001; break when
  // achieved. Return error for test cases. Clip the control signal to +10 and 
  // -10. Please make sure that control signal is clipped otherwise the test
  // cases will fail.

  // Calculate error
  // LOOP: Check if squared error is greater than 0.0001
  // Calculate the control signal
  // Check if the control signal is greater than 10: If yes, control signal=10
  // Check if the control signal is smaller than -10: If yes, control signal=-10
  // Update current_point_ as current_point_+=control signalror
  // Calculate the error again 
  // ENDLOOP

  //Calculate the error between desired and current points
  float localError = desired_point - current_point_;

  //Begin the loop, run it so long as the squared error exceeds 0.0001
  while ((localError * localError) > 0.0001) {


    //add error to cumulative error
    cum_error_ += localError;

    //calculate integral value
    float integral = cum_error_ * time_interval_;

    //calculate derivative value
    float derivative = (localError - prev_error_) / time_interval_;

    //Calculate the control signal
    float control = k_prop_ * localError + k_integral_ * integral +
        k_derivative_ * derivative;

    //Clip the control signal to max of 10, min of -10
    if (control > 10.0)
      control = 10.0;
    if (control < -10.0)
      control = -10.0;

    //Update the current point
    current_point_ += control;

    //Update the error
    prev_error_ = localError;
    localError = desired_point - current_point_;
  }
  return localError;
}
