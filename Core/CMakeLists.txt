add_library(__Chassis_Core INTERFACE)

target_include_directories(__Chassis_Core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(__Chassis_Core INTERFACE FreeRTOS)
target_link_libraries(__Chassis_Core INTERFACE libs::MIT_PD)
target_link_libraries(__Chassis_Core INTERFACE MotorDrivers::Controller)

# alias for external use
add_library(Chassis::Core ALIAS __Chassis_Core)
