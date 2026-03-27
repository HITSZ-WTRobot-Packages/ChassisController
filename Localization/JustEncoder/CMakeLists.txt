add_library(ChassisLocalizationJustEncoder STATIC
    "./JustEncoder.cpp"
)

target_include_directories(ChassisLocalizationJustEncoder
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(ChassisLocalizationJustEncoder PUBLIC Chassis::Core)

# alias for external use
add_library(ChassisLocalization::JustEncoder ALIAS ChassisLocalizationJustEncoder)
