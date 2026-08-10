#ifndef FILTER_INEKF_CORRECTION_POSE_CORRECTION_H
#define FILTER_INEKF_CORRECTION_POSE_CORRECTION_H

#include <iostream>

#include "drift/filter/base_correction.h"
#include "drift/filter/inekf/inekf.h"
#include "drift/math/lie_group.h"
#include "drift/measurement/odom.h"
#include "drift/utils/type_def.h"

using namespace math;
using namespace state;
using namespace measurement;

namespace filter::inekf {

/**
 * @class PoseCorrection
 * @brief A class for state correction using full pose (SE(3)) measurement data.
 *
 * A class for state correction using full pose (position + orientation) measurement data.
 * This class handles the correction of the state estimate using the measured pose
 * between the body frame and the world frame. Default is a left-invariant
 * measurement model.
 */
class PoseCorrection : public Correction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @name Constructors
  /// @{
  /**
   * @brief Constructor for pose correction class.
   *
   * @param[in] sensor_data_buffer_ptr: Pointer to the buffer of sensor data
   * @param[in] sensor_data_buffer_mutex_ptr: Pointer to the mutex for the
   * sensor data buffer
   * @param[in] error_type: Error type for the correction. LeftInvariant or
   * RightInvariant
   * @param[in] yaml_filepath: Path to the yaml file for the correction
   */
  PoseCorrection(OdomQueuePtr sensor_data_buffer_ptr,
                 std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
                 const ErrorType& error_type,
                 const std::string& yaml_filepath);
  /// @}

  ~PoseCorrection();

  /// @name Correction Methods
  /// @{
  /**
   * @brief Corrects the state estimate using full SE(3) pose measurement
   * including position and orientation in the world frame.
   *
   * @param[in,out] state: the current state estimate
   * @return bool: true if the correction was successful, false otherwise
   */
  bool Correct(RobotState& state) override;
  /// @}

  /// @name Getters
  /// @{
  /**
   * @brief Return the pointer to the sensor data buffer
   *
   * @return OdomQueuePtr: pointer to the sensor data buffer
   */
  const OdomQueuePtr get_sensor_data_buffer_ptr() const;

  /// @name Setters
  /**
   * @brief Set the initial pose of the robot
   *
   * @param[in,out] state: the current state estimate, which will be initialized
   * @return bool: whether the initialization is successful
   */
  bool initialize(RobotState& state) override;

  /**
   * @brief Clear the sensor_data_buffer
   */
  void clear() override;
  /// @}

 private:
  const ErrorType error_type_;             /**< Error type for the correction. */
  OdomQueuePtr sensor_data_buffer_ptr_;    /**< Pointer to the sensor buffer. */
  Eigen::Matrix<double, 6, 6> covariance_; /**< Pose covariance matrix. */
  Eigen::Matrix3d pos_cov_;
  Eigen::Matrix3d ori_cov_;

  double t_diff_thres_; /**< Maximum allowed time difference for valid measurement. */
  bool fix_z_;          /**< If true, constrain pose measurements to fixed_z_value_. */
  double fixed_z_value_; /**< Fixed world-frame z value used when fix_z_ is true. */

  /**
   * If true (default), before computing the joint 12-row correction's
   * Kalman gain, the position-rotation and position-bias blocks of the
   * *prior* covariance are zeroed in a local copy used for that one gain
   * computation only: the stored state covariance 
   * still uses the true, un-zeroed prior, so this doesn't change what the
   * filter believes about its own uncertainty going forward, only what
   * this one update's gain is allowed to exploit.
   *
   * Why: the position measurement's residual and the rotation/bias states
   * are correlated in the prior (via propagation and prior corrections),
   * and an unconditioned joint gain will use that correlation to let GPS
   * position information perturb attitude. 
   * I have found that this correlation is not a useful heading source 
   * It degrades heading accuracy, sometimes by an order of magnitude, with no
   * compensating benefit. Zeroing it before the gain computation removes
   * that while leaving the correction a single batch solve with the
   * same linearization point and same constant Jacobians.
   *
   * If false, the joint gain is computed from the prior covariance as-is,
   * cross-covariance included: this is the original, unconditioned
   * behavior, kept only for direct comparison against that baseline.
   */
  bool decouple_prior_covariance_;
};

}  // namespace filter::inekf

#endif  // FILTER_INEKF_CORRECTION_POSE_CORRECTION_H
