add_library(ChassisOmni4 STATIC
        "Omni4.cpp"
)

target_include_directories(ChassisOmni4
        PUBLIC
        .
)

# link dependencies if any
target_link_libraries(ChassisOmni4 PUBLIC Chassis::Core)
target_link_libraries(ChassisOmni4 PUBLIC FreeRTOS)

# alias for external use
add_library(Chassis::Omni4 ALIAS ChassisOmni4)
