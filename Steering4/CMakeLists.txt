add_library(ChassisSteering4 STATIC
    "./SteeringWheel.cpp"
    "./Steering4.cpp"
)

target_include_directories(ChassisSteering4
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(ChassisSteering4 PUBLIC Chassis::Core)
target_link_libraries(ChassisSteering4 PUBLIC FreeRTOS)
target_link_libraries(ChassisSteering4 PUBLIC bsp::GPIO_Driver)

# alias for external use
add_library(Chassis::Steering4 ALIAS ChassisSteering4)
