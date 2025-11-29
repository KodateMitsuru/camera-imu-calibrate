# Test configuration for camera_imu_calibrate

# Add test for camera-imu calibration with test data
add_test(
    NAME calibrate_test_data
    COMMAND ${CMAKE_BINARY_DIR}/calibrate imu_camera
        --dataset-name mydata
        --cams ${CMAKE_SOURCE_DIR}/test_data/cam_april-camchain.yaml
        --imu ${CMAKE_SOURCE_DIR}/test_data/imu_adis16448.yaml
        --target ${CMAKE_SOURCE_DIR}/test_data/april_6x6_80x80cm.yaml
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Set test properties
set_tests_properties(calibrate_test_data PROPERTIES
    TIMEOUT 300  # 5 minutes timeout
    LABELS "integration"
)

add_test(
    NAME calibrate_help
    COMMAND ${CMAKE_BINARY_DIR}/calibrate --help
)

set_tests_properties(calibrate_help PROPERTIES
    TIMEOUT 10
    LABELS "smoke"
)
