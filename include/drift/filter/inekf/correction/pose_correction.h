#ifndef FILTER_INEKF_CORRECTION_POSE_CORRECTION_H
#define FILTER_INEKF_CORRECTION_POSE_CORRECTION_H

#include <fstream>
#include <iomanip>
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

  std::ofstream est_pose_outfile_;
};

}  // namespace filter::inekf

#endif  // FILTER_INEKF_CORRECTION_POSE_CORRECTION_H
