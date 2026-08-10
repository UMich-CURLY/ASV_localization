/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   inekf.h
 *  @author Tingjun Li, Tzu-Yuan Lin, Ross Hartley
 *  @brief  Header file for Invariant EKF.
 *  Original paper:
 *  https://journals.sagepub.com/doi/full/10.1177/0278364919894385
 *  Original github repo:
 *  https://github.com/RossHartley/invariant-ekf
 *
 *  @date   May 16, 2023
 **/

#ifndef FILTER_INEKF_INEKF_H
#define FILTER_INEKF_INEKF_H
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <map>
#include <vector>

#include "drift/math/lie_group.h"
#include "drift/state/robot_state.h"

using namespace math;
using namespace state;

namespace filter::inekf {

enum ErrorType { LeftInvariant, RightInvariant };


/// @}
// ======================================================================
/**
 * @brief Corrects the state using Right Invariant observation model with
 * given measurement, output and matrices.
 *
 * @param[in] Z: innovation matrix
 * @param[in] H: measurement error matrix
 * @param[in] N: measurement noise matrix
 * @param[in,out] state: Robot state
 * @param[in] error_type: Error type， RightInvariant or LeftInvariant
 * @return None
 */
void CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                           const Eigen::MatrixXd& N, RobotState& state,
                           ErrorType error_type);

// ======================================================================
/**
 * @brief Corrects the state using Left Invariant observation model with
 * given measurement, output and matrices.
 *
 * @param[in] Z: innovation matrix
 * @param[in] H: measurement error matrix
 * @param[in] N: measurement noise matrix
 * @param[in,out] state: Robot state
 * @param[in] error_type: Error type， RightInvariant or LeftInvariant
 * @param[in] decouple_group_a, decouple_group_b: if both size == dimP with
 * disjoint nonzero support, zero the *prior* covariance's cross-block between
 * these two index groups before computing the gain only (the stored state
 * covariance, updated afterward via the Joseph form, still uses the true
 * prior). Since covariance is symmetric, this is necessarily bidirectional:
 * neither group's measurement can correct the other through their prior
 * correlation, in either direction.
 * @return None
 */
void CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& N, RobotState& state,
                          ErrorType error_type,
                          const Eigen::VectorXd& decouple_group_a
                          = Eigen::VectorXd(),
                          const Eigen::VectorXd& decouple_group_b
                          = Eigen::VectorXd());


}    // namespace filter::inekf
#endif    // end FILTER_INEKF_INEKF_H