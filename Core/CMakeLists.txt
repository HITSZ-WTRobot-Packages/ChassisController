add_library(ChassisCore STATIC
    "./IChassis.cpp"
)

target_include_directories(ChassisCore
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(ChassisCore PUBLIC FreeRTOS)
target_link_libraries(ChassisCore PUBLIC libs::MIT_PD)
target_link_libraries(ChassisCore PUBLIC MotorDrivers::Controller)
target_link_libraries(ChassisCore PUBLIC VelocityProfile::SCurve)

# alias for external use
add_library(Chassis::Core ALIAS ChassisCore)
