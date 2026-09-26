find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/librm/libs/eigen)
if(MSVC)
  add_compile_options(/utf-8 /EHsc /UNDEBUG)
  add_compile_definitions(_USE_MATH_DEFINES NOMINMAX)
else()
  add_compile_options(-UNDEBUG)
endif()

function(offline_test name)
  add_executable(${name} ${ARGN})
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} TIMEOUT 45)
endfunction()
execute_process(COMMAND ${Python3_EXECUTABLE} -c
  "from tools.rmv.protocol import examples, encode; print(chr(10).join(encode(f).hex(' ') for _, f in examples()))"
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_FILE ${CMAKE_BINARY_DIR}/rmv_vectors.txt
  RESULT_VARIABLE vector_result)
if(NOT vector_result EQUAL 0)
  message(FATAL_ERROR "Could not generate RMV golden wire vectors")
endif()
add_executable(rmv_core_test ../rmv/core_test.cpp)
add_test(NAME rmv_core_test COMMAND rmv_core_test ${CMAKE_BINARY_DIR}/rmv_vectors.txt)
offline_test(rmv_transport_test ../rmv/transport_test.cpp)
offline_test(rmv_session_test ../rmv/session_test.cpp)
add_executable(rmv_session_peer ../rmv/session_peer.cpp)
add_test(NAME rmv_cpp_interop COMMAND ${Python3_EXECUTABLE} -m tools.rmv.cpp_interop $<TARGET_FILE:rmv_session_peer>)
set_tests_properties(rmv_cpp_interop PROPERTIES WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} TIMEOUT 45)
offline_test(trajectory_test trajectory_test.cpp ../../tools/trajectory.cpp)
foreach(name measurement_noise yaw_eskf armor_motion_filter robust_range_filter
             shot_delay_adapter rotation_model_manager gyro_phase_scheduler gyro_face_selector)
  offline_test(${name}_test ../../tasks/auto_aim/${name}_test.cpp)
endforeach()
add_test(NAME rmv_python COMMAND ${Python3_EXECUTABLE} -m unittest discover -s tests/rmv -v)
set_tests_properties(rmv_python PROPERTIES WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} TIMEOUT 45)
add_test(NAME replay_python COMMAND ${Python3_EXECUTABLE} -m unittest discover -s tests/replay -v)
set_tests_properties(replay_python PROPERTIES WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} TIMEOUT 45)
add_test(NAME benchmark_python COMMAND ${Python3_EXECUTABLE} -m unittest discover -s tests/benchmark -v)
set_tests_properties(benchmark_python PROPERTIES WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} TIMEOUT 45)

# Uses the existing serial backend, but no test opens a real serial device.
add_subdirectory(${PROJECT_SOURCE_DIR}/io/serial ${CMAKE_BINARY_DIR}/serial)
offline_test(rmv_serial_compile_test ../rmv/serial_compile_test.cpp)
target_link_libraries(rmv_serial_compile_test PRIVATE serial)

option(SP_OFFLINE_WITH_VISION "Test actual OpenCV geometry and aiming sources" ON)
if(SP_OFFLINE_WITH_VISION)
  find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc calib3d imgcodecs)
  find_package(yaml-cpp CONFIG REQUIRED)
  find_package(fmt CONFIG REQUIRED)
  find_package(spdlog CONFIG REQUIRED)
  if(TARGET yaml-cpp::yaml-cpp)
    set(OFFLINE_YAML yaml-cpp::yaml-cpp)
  else()
    set(OFFLINE_YAML yaml-cpp)
  endif()
  add_library(offline_vision STATIC
    ../../tasks/auto_aim/armor.cpp ../../tasks/auto_aim/solver.cpp
    ../../tasks/auto_aim/target.cpp ../../tasks/auto_aim/aimer.cpp
    ../../tasks/auto_aim/adaptive_corner_refiner.cpp
    ../../tools/math_tools.cpp ../../tools/trajectory.cpp
    ../../tools/extended_kalman_filter.cpp ../../tools/logger.cpp)
  target_include_directories(offline_vision PUBLIC ${OpenCV_INCLUDE_DIRS})
  # The vendored Eigen subset intentionally excludes Tensor; only cv2eigen /
  # eigen2cv matrix conversion is used by these production sources.
  target_compile_definitions(offline_vision PUBLIC OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)
  target_link_libraries(offline_vision PUBLIC ${OpenCV_LIBS} ${OFFLINE_YAML} fmt::fmt spdlog::spdlog)
  offline_test(adaptive_corner_refiner_test ../../tasks/auto_aim/adaptive_corner_refiner_test.cpp)
  target_link_libraries(adaptive_corner_refiner_test PRIVATE offline_vision)
  foreach(name ekf_consistency iterated_ekf)
    offline_test(${name}_test ../${name}_test.cpp)
    target_link_libraries(${name}_test PRIVATE offline_vision)
  endforeach()
  offline_test(math_tools_test math_tools_test.cpp)
  target_link_libraries(math_tools_test PRIVATE offline_vision)
  offline_test(vision_pipeline_test vision_pipeline_test.cpp)
  target_link_libraries(vision_pipeline_test PRIVATE offline_vision)
  add_library(offline_planner STATIC
    ../../tasks/auto_aim/planner/planner.cpp
    ../../tasks/auto_aim/planner/tinympc/admm.cpp
    ../../tasks/auto_aim/planner/tinympc/tiny_api.cpp
    ../../tasks/auto_aim/planner/tinympc/rho_benchmark.cpp)
  target_link_libraries(offline_planner PUBLIC offline_vision)
  offline_test(planner_time_test planner_time_test.cpp)
  target_link_libraries(planner_time_test PRIVATE offline_planner)
  option(SP_OFFLINE_WITH_INFERENCE "Build file-only actual YOLO26 replay (requires local model)" OFF)
  if(SP_OFFLINE_WITH_INFERENCE)
    find_package(OpenVINO REQUIRED COMPONENTS Runtime)
    find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc calib3d imgcodecs videoio dnn highgui)
    find_package(nlohmann_json CONFIG REQUIRED)
    include(CheckCXXSourceCompiles)
    set(CMAKE_REQUIRED_INCLUDES ${OpenCV_INCLUDE_DIRS})
    check_cxx_source_compiles("#include <opencv2/videoio.hpp>
int main() { return static_cast<int>(cv::CAP_PROP_N_THREADS); }" SP_HAS_VIDEO_THREAD_CONTROL)
    unset(CMAKE_REQUIRED_INCLUDES)
    add_executable(detector_replay detector_replay.cpp
      ../../tasks/auto_aim/yolos/yolo26.cpp ../../tasks/auto_aim/detector.cpp
      ../../tasks/auto_aim/classifier.cpp ../../tools/img_tools.cpp)
    target_compile_definitions(detector_replay PRIVATE ENABLE_OPENVINO)
    if(SP_HAS_VIDEO_THREAD_CONTROL)
      target_compile_definitions(detector_replay PRIVATE SP_HAS_VIDEO_THREAD_CONTROL)
    else()
      message(STATUS "OpenCV lacks decoder thread control: C++ replay accepts images/image lists only")
    endif()
    target_link_libraries(detector_replay PRIVATE offline_vision openvino::runtime
      ${OpenCV_LIBS} nlohmann_json::nlohmann_json)
  endif()
endif()
