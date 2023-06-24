function(pad_integer_with_zeros output i count)
  # Convert integer i to string s
  set(s "${i}")

  # Calculate the number of leading zeros required
  string(LENGTH "${s}" digit_count)
  math(EXPR zero_padding "${count} - ${digit_count}")

  # Add leading zeros if necessary
  if (zero_padding GREATER 0)
    set(leading_zeros "")
    foreach(_ RANGE 1 ${zero_padding})
      string(APPEND leading_zeros "0")
    endforeach()
    set(s "${leading_zeros}${s}")
  endif()

  # Set output
  set(${output} "${s}" PARENT_SCOPE)
endfunction()


function(generate_app_name app_name target_name app_id padding_count)
    pad_integer_with_zeros(app_prefix ${app_id} ${padding_count})
    set(app_name "${app_prefix}-${target_name}" PARENT_SCOPE)
endfunction()


set(app_targets "")
set(app_count 1)
set(padding_zero_count 2)

function(define_app_target target app_dir)
    generate_app_name(app_name ${target} ${app_count} ${padding_zero_count})

    add_executable(${app_name} ${app_dir}/main.cc)
    set_target_properties(${app_name} PROPERTIES OUTPUT_NAME "${app_name}")
    target_include_directories(${app_name} PRIVATE src)
    target_link_libraries(${app_name} PRIVATE meshlib)
    target_compile_definitions(${app_name} PRIVATE "$<$<CONFIG:Debug>:ENABLE_DEBUG_LOG>")
    message(STATUS "Add Application Target: ${app_name}")

    set(app_targets ${app_targets} ${app_name} PARENT_SCOPE)
    math(EXPR incremented_app_id "${app_count} + 1")
    set(app_count ${incremented_app_id} PARENT_SCOPE)
endfunction()

# basic usage
define_app_target(openmesh openmesh)

# geometry properties
define_app_target(geometry-property-visualization geom-prop-vis)

# parameterization
define_app_target(multi-dimension-scaling multi-dimension-scaling)
define_app_target(parameterization-tutte param-tutte)
define_app_target(parameterization-lscm lscm)

# decimation
define_app_target(decimation decimation)
define_app_target(quadric-mesh-simplification qslim)

# segmentation
define_app_target(k-means k-means)
define_app_target(iterative-cluster iterative-cluster)
define_app_target(segmentation mesh-segmentation)

# remesh
define_app_target(remesh remesh)

message(STATUS "app targets: ${app_targets}")

add_custom_target(build_all DEPENDS ${app_targets})