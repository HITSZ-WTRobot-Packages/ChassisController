add_library(__Chassis_Controller INTERFACE)

target_include_directories(__Chassis_Controller INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(__Chassis_Controller INTERFACE Chassis::Core)
target_link_libraries(__Chassis_Controller INTERFACE libs::RingBuffer)

# alias for external use
add_library(Chassis::Controller ALIAS __Chassis_Controller)
