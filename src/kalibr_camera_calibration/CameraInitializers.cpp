#include "kalibr_camera_calibration/CameraInitializers.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/HomogeneousPoint.hpp>
#include <aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <cstddef>
#include <memory>
#include <numeric>
#include <print>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <sm/kinematics/transformations.hpp>
#include <vector>
#include "math_utils.hpp"

#include "aslam/backend/Optimizer2Options.hpp"
#include "aslam/backend/RotationQuaternion.hpp"
#include "aslam/backend/TransformationBasic.hpp"
#include "kalibr_camera_calibration/MulticamGraph.hpp"

namespace kalibr {

// ============================================================================
// addPoseDesignVariable implementation
// ============================================================================

std::shared_ptr<aslam::backend::TransformationBasic> addPoseDesignVariable(
    aslam::backend::OptimizationProblem& problem,
    const sm::kinematics::Transformation& T0) {
  auto q_Dv = std::make_shared<aslam::backend::RotationQuaternion>(T0.q());
  q_Dv->setActive(true);
  problem.addDesignVariable(q_Dv);

  // Create translation design variable
  auto t_Dv = std::make_shared<aslam::backend::EuclideanPoint>(T0.t());
  t_Dv->setActive(true);
  problem.addDesignVariable(t_Dv);

  // Create TransformationBasic on heap to ensure stable address for expressions
  return std::make_shared<aslam::backend::TransformationBasic>(
      q_Dv->toExpression(), t_Dv->toExpression());
}

// ============================================================================
// Helper functions implementation
// ============================================================================

aslam::backend::Optimizer2Options createCameraOptimizerOptions(
    bool verbose, int maxIterations) {
  aslam::backend::Optimizer2Options options;
  options.verbose = verbose;
  options.nThreads = 4;
  options.convergenceDeltaX = 1e-3;
  options.convergenceDeltaJ = 1;
  options.maxIterations = maxIterations;
  options.trustRegionPolicy =
      std::make_shared<aslam::backend::LevenbergMarquardtTrustRegionPolicy>(10);
  return options;
}

std::tuple<double, double, double> computeReprojectionErrorStats(
    const std::vector<std::shared_ptr<aslam::backend::ErrorTerm>>& errors) {
  if (errors.empty()) {
    return {0.0, 0.0, 0.0};
  }

  std::vector<double> e2_vals;
  e2_vals.reserve(errors.size());
  for (const auto& err : errors) {
    if (err) {
      e2_vals.push_back(err->evaluateError());
    }
  }

  if (e2_vals.empty()) {
    return {0.0, 0.0, 0.0};
  }

  // Compute mean
  double sum = std::accumulate(e2_vals.begin(), e2_vals.end(), 0.0);
  double mean = sum / static_cast<double>(e2_vals.size());

  // Compute median
  std::vector<double> sorted_vals = e2_vals;
  std::sort(sorted_vals.begin(), sorted_vals.end());
  double median;
  size_t n = sorted_vals.size();
  if (n % 2 == 0) {
    median = (sorted_vals[n / 2 - 1] + sorted_vals[n / 2]) / 2.0;
  } else {
    median = sorted_vals[n / 2];
  }

  // Compute std
  double sq_sum = 0.0;
  for (double val : e2_vals) {
    sq_sum += (val - mean) * (val - mean);
  }
  double stddev = std::sqrt(sq_sum / static_cast<double>(e2_vals.size()));

  return {mean, median, stddev};
}

// ============================================================================
// stereoCalibrate implementation
// ============================================================================

StereoCalibrateResult stereoCalibrate(
    CameraGeometry& camL_geometry, CameraGeometry& camH_geometry,
    const std::vector<std::pair<
        std::optional<aslam::cameras::GridCalibrationTargetObservation>,
        std::optional<aslam::cameras::GridCalibrationTargetObservation>>>&
        obslist,
    bool distortionActive,
    const std::optional<sm::kinematics::Transformation>& baseline) {
  StereoCalibrateResult result;

  // Step 1: Find initial guess as median of all PnP solutions
  sm::kinematics::Transformation baseline_HL;

  if (!baseline.has_value()) {
    std::vector<Eigen::Vector3d> t_vec;
    std::vector<Eigen::Vector3d> r_vec;
    sm::kinematics::RotationVector rv;

    for (const auto& [obsL, obsH] : obslist) {
      // If we have observations for both cameras
      if (obsL.has_value() && obsH.has_value()) {
        sm::kinematics::Transformation T_L, T_H;
        bool successL =
            camL_geometry.getGeometry()->estimateTransformation(*obsL, T_L);
        bool successH =
            camH_geometry.getGeometry()->estimateTransformation(*obsH, T_H);

        if (successL && successH) {
          auto baseline_est = T_H.inverse() * T_L;
          t_vec.push_back(baseline_est.t());
          r_vec.push_back(rv.rotationMatrixToParameters(baseline_est.C()));
        }
      }
    }

    if (t_vec.empty()) {
      std::println(std::cerr,
                   "stereoCalibrate: No valid observation pairs found");
      result.success = false;
      return result;
    }

    Eigen::Vector3d r_median = math_utils::computeMedian(r_vec);
    Eigen::Vector3d t_median = math_utils::computeMedian(t_vec);
    Eigen::Matrix3d R_median = rv.parametersToRotationMatrix(r_median);

    baseline_HL = sm::kinematics::Transformation(
        sm::kinematics::rt2Transform(R_median, t_median));
  } else {
    baseline_HL = baseline.value();
  }

  // Step 2: Set up bundle adjustment problem
  aslam::backend::OptimizationProblem problem;

  // Add baseline design variable
  auto baseline_dv = addPoseDesignVariable(problem, baseline_HL);

  // Add target pose design variables for all views
  std::vector<std::shared_ptr<aslam::backend::TransformationBasic>>
      target_pose_dvs;
  for (const auto& [obsL, obsH] : obslist) {
    sm::kinematics::Transformation T_t_cL;

    if (obsL.has_value()) {
      camL_geometry.getGeometry()->estimateTransformation(*obsL, T_t_cL);
    } else if (obsH.has_value()) {
      sm::kinematics::Transformation T_t_cH;
      camH_geometry.getGeometry()->estimateTransformation(*obsH, T_t_cH);
      T_t_cL = T_t_cH * baseline_HL;  // Apply baseline for second camera
    }

    target_pose_dvs.push_back(addPoseDesignVariable(problem, T_t_cL));
  }

  // Add camera design variables
  camL_geometry.setDvActiveStatus(true, distortionActive, false);
  camH_geometry.setDvActiveStatus(true, distortionActive, false);

  if (camL_geometry.getDistortionDv()) {
    problem.addDesignVariable(camL_geometry.getDistortionDv());
  }
  if (camL_geometry.getProjectionDv()) {
    problem.addDesignVariable(camL_geometry.getProjectionDv());
  }
  if (camL_geometry.getShutterDv()) {
    problem.addDesignVariable(camL_geometry.getShutterDv());
  }
  if (camH_geometry.getDistortionDv()) {
    problem.addDesignVariable(camH_geometry.getDistortionDv());
  }
  if (camH_geometry.getProjectionDv()) {
    problem.addDesignVariable(camH_geometry.getProjectionDv());
  }
  if (camH_geometry.getShutterDv()) {
    problem.addDesignVariable(camH_geometry.getShutterDv());
  }

  // Step 3: Add reprojection error terms
  double cornerUncertainty = 1.0;
  Eigen::Matrix2d R =
      Eigen::Matrix2d::Identity() * cornerUncertainty * cornerUncertainty;
  Eigen::Matrix2d invR = R.inverse();

  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> reprojectionErrors0;
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> reprojectionErrors1;

  auto targetL = camL_geometry.getTargetDetector()->getDetector()->target();
  auto targetH = camH_geometry.getTargetDetector()->getDetector()->target();

  // Add error terms for camera L (index 0)
  for (size_t view_id = 0; view_id < obslist.size(); ++view_id) {
    const auto& obsL = obslist[view_id].first;
    if (obsL.has_value()) {
      auto T_cam_w = target_pose_dvs[view_id]->toExpression().inverse();

      for (size_t i = 0; i < targetL->size(); ++i) {
        auto p_target = aslam::backend::HomogeneousExpression(
            sm::kinematics::toHomogeneous(targetL->point(i)));

        Eigen::Vector2d y;
        bool valid = obsL->imagePoint(static_cast<int>(i), y);
        if (valid) {
          auto rerr = camL_geometry.createReprojectionError(y, invR,
                                                            T_cam_w * p_target);
          if (rerr) {
            problem.addErrorTerm(rerr);
            reprojectionErrors0.push_back(rerr);
          }
        }
      }
    }
  }

  // Add error terms for camera H (index 1)
  for (size_t view_id = 0; view_id < obslist.size(); ++view_id) {
    const auto& obsH = obslist[view_id].second;
    if (obsH.has_value()) {
      auto T_cam0_w = target_pose_dvs[view_id]->toExpression().inverse();
      auto T_cam_w = baseline_dv->toExpression() * T_cam0_w;

      for (size_t i = 0; i < targetH->size(); ++i) {
        auto p_target = aslam::backend::HomogeneousExpression(
            sm::kinematics::toHomogeneous(targetH->point(i)));

        Eigen::Vector2d y;
        bool valid = obsH->imagePoint(static_cast<int>(i), y);
        if (valid) {
          auto rerr = camH_geometry.createReprojectionError(y, invR,
                                                            T_cam_w * p_target);
          if (rerr) {
            problem.addErrorTerm(rerr);
            reprojectionErrors1.push_back(rerr);
          }
        }
      }
    }
  }

  std::println("stereoCalibrate: added {} camera error terms",
               (reprojectionErrors0.size() + reprojectionErrors1.size()));

  // Step 4: Optimize
  auto options = createCameraOptimizerOptions(true, 200);
  aslam::backend::Optimizer2 optimizer(options);
  optimizer.setProblem(std::make_shared<aslam::backend::OptimizationProblem>(
      std::move(problem)));

  try {
    auto retval = optimizer.optimize();
    result.success = !retval.linearSolverFailure;

    if (retval.linearSolverFailure) {
      std::println(std::cerr, "stereoCalibrate: Optimization failed!");
    }
  } catch (const std::exception& e) {
    std::println(std::cerr,
                 "stereoCalibrate: Optimization failed with exception: {}",
                 e.what());
    result.success = false;
  }

  if (result.success) {
    result.baseline_HL =
        sm::kinematics::Transformation(baseline_dv->toTransformationMatrix());
  } else {
    result.baseline_HL = baseline_HL;  // Return initial guess on failure
  }

  return result;
}

// ============================================================================
// calibrateIntrinsics implementation
// ============================================================================

bool calibrateIntrinsics(
    CameraGeometry& cam_geometry,
    const std::vector<aslam::cameras::GridCalibrationTargetObservation>&
        obslist,
    bool distortionActive, bool intrinsicsActive) {
  // Set up bundle adjustment problem
  aslam::backend::OptimizationProblem problem;

  // Add camera design variables
  cam_geometry.setDvActiveStatus(intrinsicsActive, distortionActive, false);

  if (cam_geometry.getDistortionDv()) {
    problem.addDesignVariable(cam_geometry.getDistortionDv());
  }
  if (cam_geometry.getProjectionDv()) {
    problem.addDesignVariable(cam_geometry.getProjectionDv());
  }
  if (cam_geometry.getShutterDv()) {
    problem.addDesignVariable(cam_geometry.getShutterDv());
  }

  // Corner uncertainty
  double cornerUncertainty = 1.0;
  Eigen::Matrix2d R =
      Eigen::Matrix2d::Identity() * cornerUncertainty * cornerUncertainty;
  Eigen::Matrix2d invR = R.inverse();

  auto target = cam_geometry.getTargetDetector()->getDetector()->target();

  // Add target pose design variables and error terms for all views
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> reprojectionErrors;
  std::vector<std::shared_ptr<aslam::backend::TransformationBasic>>
      target_pose_dvs;

  for (const auto& obs : obslist) {
    sm::kinematics::Transformation T_t_c;
    cam_geometry.getGeometry()->estimateTransformation(obs, T_t_c);

    auto target_pose_dv = addPoseDesignVariable(problem, T_t_c);
    target_pose_dvs.push_back(target_pose_dv);

    auto T_cam_w = target_pose_dv->toExpression().inverse();

    // Add error terms for all target points
    for (size_t i = 0; i < target->size(); ++i) {
      Eigen::Vector4d p_target_homo =
          sm::kinematics::toHomogeneous(target->point(i));
      aslam::backend::HomogeneousExpression p_target(p_target_homo);

      Eigen::Vector2d y;
      bool valid = obs.imagePoint(static_cast<int>(i), y);
      if (valid) {
        auto rerr =
            cam_geometry.createReprojectionError(y, invR, T_cam_w * p_target);
        if (rerr) {
          problem.addErrorTerm(rerr);
          reprojectionErrors.push_back(rerr);
        }
      }
    }
  }

  std::println("calibrateIntrinsics: added {} camera error terms",
               reprojectionErrors.size());

  // Optimize
  auto options = createCameraOptimizerOptions(false, 200);
  aslam::backend::Optimizer2 optimizer(options);
  optimizer.setProblem(
      std::make_shared<aslam::backend::OptimizationProblem>(problem));

  bool success = false;
  try {
    auto retval = optimizer.optimize();
    success = !retval.linearSolverFailure;

    if (retval.linearSolverFailure) {
      std::println(std::cerr, "calibrateIntrinsics: Optimization failed!");
    }
  } catch (const std::exception& e) {
    std::println(std::cerr,
                 "calibrateIntrinsics: Optimization failed with exception: {}",
                 e.what());
    success = false;
  }

  return success;
}

// ============================================================================
// solveFullBatch implementation
// ============================================================================

FullBatchResult solveFullBatch(
    std::vector<CameraGeometry*>& cameras,
    const std::vector<sm::kinematics::Transformation>& baseline_guesses,
    MulticamGraph& graph) {
  FullBatchResult result;

  // Set up bundle adjustment problem
  aslam::backend::OptimizationProblem problem;

  // Add camera design variables
  for (auto* cam : cameras) {
    cam->setDvActiveStatus(true, true, false);
    if (cam->getDistortionDv()) {
      problem.addDesignVariable(cam->getDistortionDv());
    }
    if (cam->getProjectionDv()) {
      problem.addDesignVariable(cam->getProjectionDv());
    }
    if (cam->getShutterDv()) {
      problem.addDesignVariable(cam->getShutterDv());
    }
  }

  // Add baseline design variables
  std::vector<std::shared_ptr<aslam::backend::TransformationDesignVariable>>
      baseline_dvs;
  for (size_t i = 0; i < cameras.size() - 1 && i < baseline_guesses.size();
       ++i) {
    auto baseline_dv =
        std::make_shared<aslam::backend::TransformationDesignVariable>(
            baseline_guesses[i], true, true);
    for (int j = 0; j < baseline_dv->numDesignVariables(); ++j) {
      problem.addDesignVariable(baseline_dv->getDesignVariable(j));
    }
    baseline_dvs.push_back(baseline_dv);
  }

  // Corner uncertainty
  double cornerUncertainty = 1.0;
  Eigen::Matrix2d R =
      Eigen::Matrix2d::Identity() * cornerUncertainty * cornerUncertainty;
  Eigen::Matrix2d invR = R.inverse();

  // Get the target from first camera
  auto target = cameras[0]->getTargetDetector()->getDetector()->target();

  // Add reprojection error terms for all views
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> reprojectionErrors;
  std::vector<std::shared_ptr<aslam::backend::TransformationBasic>>
      target_pose_dvs;

  // Get observation database from graph
  const auto& obsDb = graph.getObsDb();
  auto timestamps = obsDb.getAllViewTimestamps();

  for (size_t view_id = 0; view_id < timestamps.size(); ++view_id) {
    double timestamp = timestamps[view_id];

    // Get all observations at this timestamp
    auto obs_tuple = obsDb.getAllObsAtTimestamp(timestamp);

    // Create target pose design variable
    auto T0 = graph.getTargetPoseGuess(timestamp, cameras, baseline_guesses);
    auto target_pose_dv = addPoseDesignVariable(problem, T0);
    target_pose_dvs.push_back(target_pose_dv);

    for (const auto& [cidx, obs_ptr] : obs_tuple) {
      if (!obs_ptr) continue;
      const auto& obs = *obs_ptr;

      auto* cam = cameras[cidx];

      // Build pose chain: target -> cam0 -> baselines -> camN
      auto T_cam0_calib = target_pose_dv->toExpression().inverse();
      auto T_camN_calib = T_cam0_calib;
      for (int idx = 0;
           idx < cidx && idx < static_cast<int>(baseline_dvs.size()); ++idx) {
        T_camN_calib = baseline_dvs[idx]->toExpression() * T_camN_calib;
      }

      // Add error terms
      for (size_t i = 0; i < target->size(); ++i) {
        Eigen::Vector4d p_target_homo =
            sm::kinematics::toHomogeneous(target->point(i));
        aslam::backend::HomogeneousExpression p_target(p_target_homo);

        Eigen::Vector2d y;
        bool valid = obs.imagePoint(static_cast<int>(i), y);
        if (valid) {
          auto rerr =
              cam->createReprojectionError(y, invR, T_camN_calib * p_target);
          if (rerr) {
            problem.addErrorTerm(rerr);
            reprojectionErrors.push_back(rerr);
          }
        }
      }
    }
  }

  std::println("solveFullBatch: added {} camera error terms",
               reprojectionErrors.size());

  // Optimize
  auto options = createCameraOptimizerOptions(false, 250);
  aslam::backend::Optimizer2 optimizer(options);
  optimizer.setProblem(std::make_shared<aslam::backend::OptimizationProblem>(
      std::move(problem)));

  try {
    auto retval = optimizer.optimize();
    result.success = !retval.linearSolverFailure;

    if (retval.linearSolverFailure) {
      std::println(std::cerr, "solveFullBatch: Optimization failed!");
    }
  } catch (const std::exception& e) {
    std::println(std::cerr,
                 "solveFullBatch: Optimization failed with exception: {}",
                 e.what());
    result.success = false;
  }

  // Extract optimized baselines
  for (const auto& baseline_dv : baseline_dvs) {
    result.baselines.push_back(
        sm::kinematics::Transformation(baseline_dv->T()));
  }

  return result;
}

}  // namespace kalibr
