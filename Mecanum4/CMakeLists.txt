add_library(ChassisMecanum4 STATIC
    "./Mecanum4.cpp"
)

target_include_directories(ChassisMecanum4
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(ChassisMecanum4 PUBLIC Chassis::Core)
target_link_libraries(ChassisMecanum4 PUBLIC FreeRTOS)

# alias for external use
add_library(Chassis::Mecanum4 ALIAS ChassisMecanum4)
