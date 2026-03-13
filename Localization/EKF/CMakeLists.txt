add_library(ChassisLocalizationEKF STATIC
    "./LocEKF.cpp"
)

target_include_directories(ChassisLocalizationEKF
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(ChassisLocalizationEKF PUBLIC Chassis::Core)
target_link_libraries(ChassisLocalizationEKF PUBLIC SensorGyro::HWT101CT)
target_link_libraries(ChassisLocalizationEKF PUBLIC Math::EKF)
target_link_libraries(ChassisLocalizationEKF PUBLIC libs::Deque)

# alias for external use
add_library(ChassisLocalization::EKF ALIAS ChassisLocalizationEKF)
