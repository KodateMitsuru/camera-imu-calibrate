#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/backend/HomogeneousPoint.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/ReprojectionError.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/cameras.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <format>
#include <kalibr_camera_calibration/CameraCalibrator.hpp>
#include <memory>
#include <print>
#include <stdexcept>

#include "kalibr_camera_calibration/CameraInitializers.hpp"
#include "sm/kinematics/homogeneous_coordinates.hpp"

namespace kalibr {

namespace {

// Helper template to create CameraDesignVariable and ReprojectionError factory
// for a specific camera geometry type
template <typename CameraGeometryT>
struct CameraHelper {
  using CameraDv = aslam::backend::CameraDesignVariable<CameraGeometryT>;
  using ReprojError = aslam::backend::ReprojectionError<CameraGeometryT>;

  // Create CameraDesignVariable from base geometry
  static std::shared_ptr<void> createCameraDv(
      const std::shared_ptr<aslam::cameras::CameraGeometryBase>& geometry) {
    auto typedGeometry = std::dynamic_pointer_cast<CameraGeometryT>(geometry);
    if (!typedGeometry) {
      throw std::runtime_error(
          "Failed to cast geometry to correct type in createCameraDv");
    }
    return std::make_shared<CameraDv>(typedGeometry);
  }

  // Create ReprojectionError from typed CameraDesignVariable
  static std::shared_ptr<aslam::backend::ErrorTerm> createReprojError(
      const Eigen::VectorXd& y, const Eigen::MatrixXd& invR,
      const aslam::backend::HomogeneousExpression& p_c, const void* camDvPtr) {
    // Cast the void* back to the typed CameraDesignVariable
    const auto* camDv = static_cast<const std::shared_ptr<CameraDv>*>(camDvPtr);
    if (!camDv || !*camDv) {
      throw std::runtime_error(
          "Invalid CameraDesignVariable in createReprojError");
    }

    // Create 2D measurement and inverse covariance from the input vectors
    typename ReprojError::measurement_t measurement = y.head<2>();
    typename ReprojError::inverse_covariance_t invCov =
        invR.topLeftCorner<2, 2>();

    return std::make_shared<ReprojError>(measurement, invCov, p_c, **camDv);
  }
};

}  // namespace

// ============================================================================
// TargetDetector Implementation
// ============================================================================

TargetDetector::TargetDetector(
    const CalibrationTargetParameters& targetConfig,
    const std::shared_ptr<aslam::cameras::CameraGeometryBase>& cameraGeometry,
    bool showCorners, bool showReproj, bool showOneStep)
    : targetConfig_(targetConfig) {
  auto targetType = targetConfig.getTargetType();

  // Create the calibration target grid based on type
  if (targetType == TargetType::Checkerboard) {
    auto params = targetConfig.getCheckerboardParams();
    aslam::cameras::GridCalibrationTargetCheckerboard::CheckerboardOptions
        options;
    options.filterQuads = true;
    options.normalizeImage = true;
    options.useAdaptiveThreshold = true;
    options.performFastCheck = false;
    options.windowWidth = 5;
    options.showExtractionVideo = showCorners;

    grid_ = std::make_shared<aslam::cameras::GridCalibrationTargetCheckerboard>(
        params.rows, params.cols, params.rowSpacing, params.colSpacing,
        options);

  } else if (targetType == TargetType::Circlegrid) {
    auto params = targetConfig.getCirclegridParams();
    aslam::cameras::GridCalibrationTargetCirclegrid::CirclegridOptions options;
    options.showExtractionVideo = showCorners;
    options.useAsymmetricCirclegrid = params.asymmetric;

    grid_ = std::make_shared<aslam::cameras::GridCalibrationTargetCirclegrid>(
        params.rows, params.cols, params.spacing, options);

  } else if (targetType == TargetType::Aprilgrid) {
    auto params = targetConfig.getAprilgridParams();
    aslam::cameras::GridCalibrationTargetAprilgrid::AprilgridOptions options;
    options.showExtractionVideo = showCorners;
    options.minTagsForValidObs = std::max(params.tagRows, params.tagCols) + 1;

    grid_ = std::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(
        params.tagRows, params.tagCols, params.tagSize, params.tagSpacing,
        options);

  } else {
    throw std::runtime_error("Unknown calibration target type");
  }

  // Create grid detector
  aslam::cameras::GridDetector::GridDetectorOptions detectorOptions;
  detectorOptions.imageStepping = showOneStep;
  detectorOptions.plotCornerReprojection = showReproj;
  detectorOptions.filterCornerOutliers = false;

  detector_ = std::make_shared<aslam::cameras::GridDetector>(
      cameraGeometry, grid_, detectorOptions);
}

// ============================================================================
// CameraGeometry Implementation
// ============================================================================

CameraGeometry::CameraGeometry(
    const CameraParameters& cameraModel,
    const CalibrationTargetParameters& targetConfig,
    const ImageDatasetReader& dataset,
    std::shared_ptr<aslam::cameras::CameraGeometryBase> geometry, bool verbose)
    : dataset_(dataset),
      cameraModel_(cameraModel),
      isGeometryInitialized_(false) {
  // Create camera from parameters
  camera_ = AslamCamera::fromParameters(cameraModel);

  if (geometry == nullptr) {
    geometry_ = camera_->getGeometry();
  } else {
    geometry_ = geometry;
  }

  // Create target detector
  ctarget_ = std::make_shared<TargetDetector>(targetConfig, geometry_, verbose);

  // Initialize design variables
  initializeDesignVariables();

  // Set default active status
  setDvActiveStatus(true, true, false);
}

void CameraGeometry::initializeDesignVariables() {
  // Create CameraDesignVariable based on camera type using the stored type hash
  // from AslamCamera. This uses the frame type to dispatch to the correct
  // template.

  auto frameType = camera_->getFrameType();

  // Helper lambda to initialize DVs for a given camera geometry type
  auto initForType = [this]<typename CamGeomT>() {
    using Helper = CameraHelper<CamGeomT>;
    using CamDvT = typename Helper::CameraDv;

    // Create typed CameraDesignVariable
    auto camDvPtr = std::make_shared<CamDvT>(
        std::dynamic_pointer_cast<CamGeomT>(geometry_));

    // Store type-erased pointer
    cameraDvContainer_ = camDvPtr;

    // Extract base design variables
    projectionDv_ = camDvPtr->projectionDesignVariable();
    distortionDv_ = camDvPtr->distortionDesignVariable();
    shutterDv_ = camDvPtr->shutterDesignVariable();

    // Store factory function for creating ReprojectionError
    // Capture the shared_ptr by value to extend lifetime
    reprojErrorFactory_ =
        [camDvPtr](const Eigen::VectorXd& y, const Eigen::MatrixXd& invR,
                   const aslam::backend::HomogeneousExpression& p_c,
                   const void* /*unused*/) {
          return Helper::createReprojError(y, invR, p_c, &camDvPtr);
        };
  };

  // Dispatch based on frame type to initialize correct CameraDesignVariable
  if (frameType ==
      typeid(aslam::Frame<aslam::cameras::DistortedPinholeCameraGeometry>)
          .hash_code()) {
    initForType
        .template operator()<aslam::cameras::DistortedPinholeCameraGeometry>();
  } else if (frameType ==
             typeid(
                 aslam::Frame<
                     aslam::cameras::EquidistantDistortedPinholeCameraGeometry>)
                 .hash_code()) {
    initForType.template
    operator()<aslam::cameras::EquidistantDistortedPinholeCameraGeometry>();
  } else if (frameType ==
             typeid(aslam::Frame<
                        aslam::cameras::FovDistortedPinholeCameraGeometry>)
                 .hash_code()) {
    initForType.template
    operator()<aslam::cameras::FovDistortedPinholeCameraGeometry>();
  } else if (frameType ==
             typeid(aslam::Frame<aslam::cameras::PinholeCameraGeometry>)
                 .hash_code()) {
    initForType.template operator()<aslam::cameras::PinholeCameraGeometry>();
  } else if (frameType ==
             typeid(aslam::Frame<aslam::cameras::DistortedOmniCameraGeometry>)
                 .hash_code()) {
    initForType
        .template operator()<aslam::cameras::DistortedOmniCameraGeometry>();
  } else if (frameType ==
             typeid(aslam::Frame<aslam::cameras::OmniCameraGeometry>)
                 .hash_code()) {
    initForType.template operator()<aslam::cameras::OmniCameraGeometry>();
  } else if (frameType ==
             typeid(aslam::Frame<
                        aslam::cameras::EquidistantDistortedOmniCameraGeometry>)
                 .hash_code()) {
    initForType.template
    operator()<aslam::cameras::EquidistantDistortedOmniCameraGeometry>();
  } else if (frameType ==
             typeid(
                 aslam::Frame<aslam::cameras::FovDistortedOmniCameraGeometry>)
                 .hash_code()) {
    initForType
        .template operator()<aslam::cameras::FovDistortedOmniCameraGeometry>();
  } else if (frameType ==
             typeid(aslam::Frame<aslam::cameras::ExtendedUnifiedCameraGeometry>)
                 .hash_code()) {
    initForType
        .template operator()<aslam::cameras::ExtendedUnifiedCameraGeometry>();
  } else if (frameType ==
             typeid(aslam::Frame<aslam::cameras::DoubleSphereCameraGeometry>)
                 .hash_code()) {
    initForType
        .template operator()<aslam::cameras::DoubleSphereCameraGeometry>();
  } else {
    std::println(stderr,
                 "Warning: Unknown camera frame type (hash={}), "
                 "design variables not initialized",
                 frameType);
    // Don't throw - allow calibration to proceed without design variables
    // The createReprojectionError will fail if called
  }
}

void CameraGeometry::setDvActiveStatus(bool projectionActive,
                                       bool distortionActive,
                                       bool shutterActive) {
  if (projectionDv_) {
    projectionDv_->setActive(projectionActive);
  }
  if (distortionDv_) {
    distortionDv_->setActive(distortionActive);
  }
  if (shutterDv_) {
    shutterDv_->setActive(shutterActive);
  }
}

bool CameraGeometry::initGeometryFromObservations(
    const std::vector<aslam::cameras::GridCalibrationTargetObservation>&
        observations) {
  // Initialize intrinsics from observations
  bool success = geometry_->initializeIntrinsics(observations);
  if (!success) {
    std::println(stderr, "Initialization of focal length for camera {} failed",
                 dataset_.getTopic());
    return false;
  }

  // Optimize for intrinsics & distortion
  success = calibrateIntrinsics(*this, observations);
  if (!success) {
    std::println(stderr, "Initialization of intrinsics for camera {} failed",
                 dataset_.getTopic());
  }

  isGeometryInitialized_ = success;
  return success;
}

std::shared_ptr<aslam::backend::ErrorTerm>
CameraGeometry::createReprojectionError(
    const Eigen::VectorXd& y, const Eigen::MatrixXd& invR,
    const aslam::backend::HomogeneousExpression& p_c) const {
  // Use the factory function created during initializeDesignVariables
  if (!reprojErrorFactory_) {
    throw std::runtime_error(
        "createReprojectionError: ReprojectionError factory not initialized. "
        "Ensure initializeDesignVariables() was called with a supported "
        "camera type.");
  }

  // Create ReprojectionError using the type-erased factory
  return reprojErrorFactory_(y, invR, p_c, nullptr);
}

// ============================================================================
// CalibrationTarget Implementation
// ============================================================================

CalibrationTarget::CalibrationTarget(
    const std::shared_ptr<aslam::cameras::GridCalibrationTargetBase>& target,
    bool estimateLandmarks)
    : target_(target) {
  // Create design variables and expressions for all target points
  size_t numPoints = target->size();
  P_t_dv_.reserve(numPoints);
  P_t_ex_.reserve(numPoints);

  for (size_t i = 0; i < numPoints; ++i) {
    Eigen::Vector3d p = target->point(i);
    Eigen::Vector4d p_h = sm::kinematics::toHomogeneous(p);

    auto dv = std::make_shared<aslam::backend::HomogeneousPoint>(p_h);
    dv->setActive(estimateLandmarks);

    P_t_dv_.push_back(dv);
    P_t_ex_.push_back(dv->toExpression());
  }
}

aslam::backend::HomogeneousExpression CalibrationTarget::getPointExpression(
    size_t i) const {
  return P_t_ex_.at(i);
}

// ============================================================================
// CalibrationTargetOptimizationProblem Implementation
// ============================================================================

std::shared_ptr<CalibrationTargetOptimizationProblem>
CalibrationTargetOptimizationProblem::fromTargetViewObservations(
    const std::vector<std::shared_ptr<CameraGeometry>>& cameras,
    const std::shared_ptr<CalibrationTarget>& target,
    const std::vector<
        std::shared_ptr<aslam::backend::TransformationDesignVariable>>&
        baselines,
    const aslam::Time& timestamp,
    const sm::kinematics::Transformation& T_tc_guess,
    const std::vector<RigObservation>& rigObservations,
    bool useBlakeZissermanMest) {
  auto rval = std::shared_ptr<CalibrationTargetOptimizationProblem>(
      new CalibrationTargetOptimizationProblem());

  // Store arguments for potential problem rebuilding
  rval->cameras = cameras;
  rval->target = target;
  rval->baselines = baselines;
  rval->timestamp = timestamp;
  rval->T_tc_guess = T_tc_guess;
  rval->rig_observations = rigObservations;

  // 1. Create a design variable for this pose (target-to-camera transformation)
  rval->dv_T_target_camera =
      std::make_shared<aslam::backend::TransformationDesignVariable>(
          T_tc_guess);

  for (int i = 0; i < rval->dv_T_target_camera->numDesignVariables(); ++i) {
    rval->addDesignVariable(rval->dv_T_target_camera->getDesignVariable(i),
                            TRANSFORMATION_GROUP_ID);
  }

  // 2. Add all baseline DVs
  for (const auto& baseline_dv : baselines) {
    for (int i = 0; i < baseline_dv->numDesignVariables(); ++i) {
      rval->addDesignVariable(baseline_dv->getDesignVariable(i),
                              CAM_CALIBRATION_GROUP_ID);
    }
  }

  // 3. Add landmark DVs
  for (size_t i = 0; i < target->size(); ++i) {
    rval->addDesignVariable(target->getPointDv(i), LANDMARK_GROUP_ID);
  }

  // 4. Add camera DVs
  for (const auto& camera : cameras) {
    if (!camera->isGeometryInitialized()) {
      throw std::runtime_error(
          "The camera geometry is not initialized. "
          "Please initialize with initGeometryFromObservations()");
    }
    camera->setDvActiveStatus(true, true, false);

    if (camera->getDistortionDv()) {
      rval->addDesignVariable(camera->getDistortionDv(),
                              CAM_CALIBRATION_GROUP_ID);
    }
    if (camera->getProjectionDv()) {
      rval->addDesignVariable(camera->getProjectionDv(),
                              CAM_CALIBRATION_GROUP_ID);
    }
    if (camera->getShutterDv()) {
      rval->addDesignVariable(camera->getShutterDv(), CAM_CALIBRATION_GROUP_ID);
    }
  }

  // 5. Add all observations for this view
  size_t rerrCnt = 0;
  double cornerUncertainty = 1.0;
  Eigen::Matrix2d R =
      Eigen::Matrix2d::Identity() * cornerUncertainty * cornerUncertainty;
  Eigen::Matrix2d invR = R.inverse();

  for (const auto& [camId, obs] : rigObservations) {
    auto& camera = cameras[camId];
    rval->rerrs[camId] =
        std::vector<std::shared_ptr<aslam::backend::ErrorTerm>>(target->size(),
                                                                nullptr);

    // Build baseline chain (target->cam0->baselines->camN)
    auto T_cam0_target = rval->dv_T_target_camera->toExpression().inverse();
    auto T_camN_calib = T_cam0_target;

    for (size_t idx = 0; idx < camId; ++idx) {
      T_camN_calib = baselines[idx]->toExpression() * T_camN_calib;
    }

    // Add reprojection errors for each visible point
    for (size_t i = 0; i < target->size(); ++i) {
      auto p_target = target->getPointExpression(i);

      Eigen::Vector2d y;
      bool valid = obs.imagePoint(i, y);

      if (valid) {
        rerrCnt++;

        // Transform point to camera frame
        auto p_cam = T_camN_calib * p_target;

        // Create reprojection error
        // Note: This requires the camera-specific error term
        // For now, we use a factory-based approach
        auto rerr = camera->createReprojectionError(y, invR, p_cam);

        if (rerr) {
          // Add Blake-Zisserman M-estimator if requested
          if (useBlakeZissermanMest) {
            auto mest =
                std::make_shared<aslam::backend::BlakeZissermanMEstimator>(2.0);
            rerr->setMEstimatorPolicy(mest);
          }

          rval->addErrorTerm(rerr);
          rval->rerrs[camId][i] = rerr;
        }
      }
    }
  }

  std::println("  Adding view with {} error terms", rerrCnt);

  return rval;
}

const std::vector<std::shared_ptr<aslam::backend::ErrorTerm>>&
CalibrationTargetOptimizationProblem::getReprojectionErrors(
    size_t camId) const {
  auto it = rerrs.find(camId);
  if (it == rerrs.end()) {
    static const std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> empty;
    return empty;
  }
  return it->second;
}

// ============================================================================
// removeCornersFromBatch Implementation
// ============================================================================

std::shared_ptr<CalibrationTargetOptimizationProblem> removeCornersFromBatch(
    const std::shared_ptr<CalibrationTargetOptimizationProblem>& batch,
    const std::vector<std::pair<size_t, std::vector<size_t>>>&
        camIdCornerIdPairs,
    bool useBlakeZissermanMest) {
  // Create a mutable copy of the observations
  auto modifiedObservations = batch->rig_observations;

  // Build observation lookup
  std::unordered_map<size_t, aslam::cameras::GridCalibrationTargetObservation*>
      obsMap;
  for (auto& [camId, obs] : modifiedObservations) {
    obsMap[camId] = &obs;
  }

  // Disable the specified corners
  bool hasCornerRemoved = false;
  for (const auto& [camId, cornerIds] : camIdCornerIdPairs) {
    auto it = obsMap.find(camId);
    if (it != obsMap.end()) {
      for (size_t cornerId : cornerIds) {
        it->second->removeImagePoint(cornerId);
        hasCornerRemoved = true;
      }
    }
  }

  if (!hasCornerRemoved) {
    throw std::runtime_error("Need to remove at least one corner");
  }

  // Rebuild problem
  return CalibrationTargetOptimizationProblem::fromTargetViewObservations(
      batch->cameras, batch->target, batch->baselines, batch->timestamp,
      batch->T_tc_guess, modifiedObservations, useBlakeZissermanMest);
}

// ============================================================================
// CameraCalibration Implementation
// ============================================================================

CameraCalibration::CameraCalibration(
    const std::vector<std::shared_ptr<CameraGeometry>>& cameras,
    const std::vector<sm::kinematics::Transformation>& baselineGuesses,
    bool estimateLandmarks, bool /*verbose*/, bool useBlakeZissermanMest)
    : cameras_(cameras),
      useBlakeZissermanMest_(useBlakeZissermanMest),
      estimator_(CAM_CALIBRATION_GROUP_ID) {
  // Get linear solver and optimizer options
  linearSolverOptions_ = estimator_.getLinearSolverOptions();
  optimizerOptions_ = estimator_.getOptimizerOptions();

  // Create calibration target from first camera's detector
  if (!cameras.empty() && cameras[0]->getTargetDetector()) {
    target_ = std::make_shared<CalibrationTarget>(
        cameras[0]->getTargetDetector()->getDetector()->target(),
        estimateLandmarks);
  }

  // Initialize baseline DVs
  initializeBaselineDVs(baselineGuesses);
}

void CameraCalibration::initializeBaselineDVs(
    const std::vector<sm::kinematics::Transformation>& baselineGuesses) {
  baselines_.clear();
  baselines_.reserve(cameras_.size() - 1);

  for (size_t i = 0; i < cameras_.size() - 1 && i < baselineGuesses.size();
       ++i) {
    auto baselineDv =
        std::make_shared<aslam::backend::TransformationDesignVariable>(
            baselineGuesses[i]);
    baselines_.push_back(baselineDv);
  }
}

bool CameraCalibration::addTargetView(
    const aslam::Time& timestamp,
    const std::vector<CalibrationTargetOptimizationProblem::RigObservation>&
        rigObservations,
    const sm::kinematics::Transformation& T_tc_guess, bool force) {
  // Create the problem for this batch
  auto batchProblem =
      CalibrationTargetOptimizationProblem::fromTargetViewObservations(
          cameras_, target_, baselines_, timestamp, T_tc_guess, rigObservations,
          useBlakeZissermanMest_);

  // Try to add to the estimator
  estimatorReturnValue_ = estimator_.addBatch(batchProblem, force);

  // Check if optimization converged
  if (static_cast<int>(estimatorReturnValue_.numIterations) >=
      optimizerOptions_.maxIterations) {
    std::println(stderr, "Did not converge in maxIterations... restarting...");
    throw OptimizationDiverged();
  }

  bool success = estimatorReturnValue_.batchAccepted;
  if (success) {
    std::println("  The estimator accepted this batch");
    views_.push_back(batchProblem);
  } else {
    std::println("  The estimator did not accept this batch");
  }

  return success;
}

}  // namespace kalibr
