# Bind a running executable to the source that built it, including dirty edits.
function(sp_add_runtime_identity target)
  file(GLOB_RECURSE identity_inputs CONFIGURE_DEPENDS
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/*.hpp"
    "${PROJECT_SOURCE_DIR}/tasks/auto_aim/*.cpp"
    "${PROJECT_SOURCE_DIR}/tasks/auto_aim/*.hpp"
    "${PROJECT_SOURCE_DIR}/tools/*.cpp"
    "${PROJECT_SOURCE_DIR}/tools/*.hpp"
    "${PROJECT_SOURCE_DIR}/io/*.cpp"
    "${PROJECT_SOURCE_DIR}/io/*.hpp"
    "${PROJECT_SOURCE_DIR}/io/*.h"
    "${PROJECT_SOURCE_DIR}/io/*.c")
  list(APPEND identity_inputs "${PROJECT_SOURCE_DIR}/CMakeLists.txt"
    "${PROJECT_SOURCE_DIR}/cmake/RuntimeIdentity.cmake")
  list(SORT identity_inputs)
  set(identity_text "")
  foreach(input IN LISTS identity_inputs)
    file(SHA256 "${input}" input_hash)
    file(RELATIVE_PATH relative "${PROJECT_SOURCE_DIR}" "${input}")
    string(APPEND identity_text "${relative}:${input_hash}\n")
  endforeach()
  string(SHA256 source_hash "${identity_text}")
  file(SHA256 "${PROJECT_SOURCE_DIR}/configs/standard.yaml" config_hash)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${identity_inputs} "${PROJECT_SOURCE_DIR}/configs/standard.yaml")
  execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    OUTPUT_VARIABLE revision OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(NOT revision)
    set(revision "unrecorded")
  endif()
  file(WRITE "${CMAKE_BINARY_DIR}/runtime-source-manifest.txt" "${identity_text}")
  target_compile_definitions(${target} PRIVATE
    SP_SOURCE_REVISION="${revision}"
    SP_SOURCE_SHA256="${source_hash}"
    SP_CONFIGURED_STANDARD_SHA256="${config_hash}")
endfunction()
