configure_file(../../source/resources/external_control.script ${PROJECT_BINARY_DIR}/test/integration/ COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}/example/resource/input_recipe.txt ${PROJECT_BINARY_DIR}/test/integration/ COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}/example/resource/output_recipe.txt ${PROJECT_BINARY_DIR}/test/integration/ COPYONLY)

add_executable(
    ExtrapolateConvergenceIntegrationTest
    ExtrapolateConvergenceIntegrationTest.cpp
    common/SimpleArgParser.cpp
)

target_include_directories(
    ExtrapolateConvergenceIntegrationTest
    PUBLIC
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/Common
    ${PROJECT_SOURCE_DIR}/include/Elite
    ${PROJECT_SOURCE_DIR}/include/Control
)

target_link_libraries(
    ExtrapolateConvergenceIntegrationTest
    elite_cs_series_sdk::shared
    ${SYSTEM_LIB}
)

target_link_directories(
    ExtrapolateConvergenceIntegrationTest
    PRIVATE
    ${CMAKE_BINARY_DIR}
)
