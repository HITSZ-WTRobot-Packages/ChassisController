add_library(ChassisSpecial_Steering4 STATIC
        "Special_SteeringWheel.cpp"
        "Special_Steering4.cpp"
)

target_include_directories(ChassisSpecial_Steering4
        PUBLIC
        .
)

# link dependencies if any
target_link_libraries(ChassisSpecial_Steering4 PUBLIC Chassis::Core)
target_link_libraries(ChassisSpecial_Steering4 PUBLIC FreeRTOS)
target_link_libraries(ChassisSpecial_Steering4 PUBLIC bsp::GPIO_Driver)

# alias for external use
add_library(Chassis::Special_Steering4 ALIAS ChassisSpecial_Steering4)
