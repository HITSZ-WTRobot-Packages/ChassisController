add_library(__Chassis_ControllerMaster INTERFACE)

target_include_directories(__Chassis_ControllerMaster INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(__Chassis_ControllerMaster INTERFACE Chassis::Core)
target_link_libraries(__Chassis_ControllerMaster INTERFACE VelocityProfile::SCurve)

# alias for external use
add_library(Chassis::ControllerMaster ALIAS __Chassis_ControllerMaster)
