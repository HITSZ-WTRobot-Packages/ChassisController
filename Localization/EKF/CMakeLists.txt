add_library(__ChassisLocalization_EKF INTERFACE)

target_include_directories(__ChassisLocalization_EKF INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(__ChassisLocalization_EKF INTERFACE Chassis::Core)
target_link_libraries(__ChassisLocalization_EKF INTERFACE Math::EKF)
target_link_libraries(__ChassisLocalization_EKF INTERFACE SensorGyro::HWT101CT)
target_link_libraries(__ChassisLocalization_EKF INTERFACE libs::Concurrency)
target_link_libraries(__ChassisLocalization_EKF INTERFACE libs::Deque)

# alias for external use
add_library(ChassisLocalization::EKF ALIAS __ChassisLocalization_EKF)
