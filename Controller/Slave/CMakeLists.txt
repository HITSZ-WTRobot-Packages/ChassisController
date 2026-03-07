add_library(__Chassis_ControllerSlave INTERFACE)

target_include_directories(__Chassis_ControllerSlave INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(__Chassis_ControllerSlave INTERFACE Chassis::Core)
target_link_libraries(__Chassis_ControllerSlave INTERFACE libs::RingBuffer)

# alias for external use
add_library(Chassis::ControllerSlave ALIAS __Chassis_ControllerSlave)
