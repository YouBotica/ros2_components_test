

function(conan_message MESSAGE_OUTPUT)
    if(NOT CONAN_CMAKE_SILENT_OUTPUT)
        message(${ARGV${0}})
    endif()
endfunction()


macro(conan_find_apple_frameworks FRAMEWORKS_FOUND FRAMEWORKS FRAMEWORKS_DIRS)
    if(APPLE)
        foreach(_FRAMEWORK ${FRAMEWORKS})
            # https://cmake.org/pipermail/cmake-developers/2017-August/030199.html
            find_library(CONAN_FRAMEWORK_${_FRAMEWORK}_FOUND NAME ${_FRAMEWORK} PATHS ${FRAMEWORKS_DIRS} CMAKE_FIND_ROOT_PATH_BOTH)
            if(CONAN_FRAMEWORK_${_FRAMEWORK}_FOUND)
                list(APPEND ${FRAMEWORKS_FOUND} ${CONAN_FRAMEWORK_${_FRAMEWORK}_FOUND})
            else()
                message(FATAL_ERROR "Framework library ${_FRAMEWORK} not found in paths: ${FRAMEWORKS_DIRS}")
            endif()
        endforeach()
    endif()
endmacro()


function(conan_package_library_targets libraries package_libdir deps out_libraries out_libraries_target build_type package_name)
    unset(_CONAN_ACTUAL_TARGETS CACHE)
    unset(_CONAN_FOUND_SYSTEM_LIBS CACHE)
    foreach(_LIBRARY_NAME ${libraries})
        find_library(CONAN_FOUND_LIBRARY NAME ${_LIBRARY_NAME} PATHS ${package_libdir}
                     NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
        if(CONAN_FOUND_LIBRARY)
            conan_message(STATUS "Library ${_LIBRARY_NAME} found ${CONAN_FOUND_LIBRARY}")
            list(APPEND _out_libraries ${CONAN_FOUND_LIBRARY})
            if(NOT ${CMAKE_VERSION} VERSION_LESS "3.0")
                # Create a micro-target for each lib/a found
                string(REGEX REPLACE "[^A-Za-z0-9.+_-]" "_" _LIBRARY_NAME ${_LIBRARY_NAME})
                set(_LIB_NAME CONAN_LIB::${package_name}_${_LIBRARY_NAME}${build_type})
                if(NOT TARGET ${_LIB_NAME})
                    # Create a micro-target for each lib/a found
                    add_library(${_LIB_NAME} UNKNOWN IMPORTED)
                    set_target_properties(${_LIB_NAME} PROPERTIES IMPORTED_LOCATION ${CONAN_FOUND_LIBRARY})
                    set(_CONAN_ACTUAL_TARGETS ${_CONAN_ACTUAL_TARGETS} ${_LIB_NAME})
                else()
                    conan_message(STATUS "Skipping already existing target: ${_LIB_NAME}")
                endif()
                list(APPEND _out_libraries_target ${_LIB_NAME})
            endif()
            conan_message(STATUS "Found: ${CONAN_FOUND_LIBRARY}")
        else()
            conan_message(STATUS "Library ${_LIBRARY_NAME} not found in package, might be system one")
            list(APPEND _out_libraries_target ${_LIBRARY_NAME})
            list(APPEND _out_libraries ${_LIBRARY_NAME})
            set(_CONAN_FOUND_SYSTEM_LIBS "${_CONAN_FOUND_SYSTEM_LIBS};${_LIBRARY_NAME}")
        endif()
        unset(CONAN_FOUND_LIBRARY CACHE)
    endforeach()

    if(NOT ${CMAKE_VERSION} VERSION_LESS "3.0")
        # Add all dependencies to all targets
        string(REPLACE " " ";" deps_list "${deps}")
        foreach(_CONAN_ACTUAL_TARGET ${_CONAN_ACTUAL_TARGETS})
            set_property(TARGET ${_CONAN_ACTUAL_TARGET} PROPERTY INTERFACE_LINK_LIBRARIES "${_CONAN_FOUND_SYSTEM_LIBS};${deps_list}")
        endforeach()
    endif()

    set(${out_libraries} ${_out_libraries} PARENT_SCOPE)
    set(${out_libraries_target} ${_out_libraries_target} PARENT_SCOPE)
endfunction()


include(FindPackageHandleStandardArgs)

conan_message(STATUS "Conan: Using autogenerated Findlum_drivers_lidar_iris_line_aggregator.cmake")
# Global approach
set(lum_drivers_lidar_iris_line_aggregator_FOUND 1)
set(lum_drivers_lidar_iris_line_aggregator_VERSION "1.0.0")

find_package_handle_standard_args(lum_drivers_lidar_iris_line_aggregator REQUIRED_VARS
                                  lum_drivers_lidar_iris_line_aggregator_VERSION VERSION_VAR lum_drivers_lidar_iris_line_aggregator_VERSION)
mark_as_advanced(lum_drivers_lidar_iris_line_aggregator_FOUND lum_drivers_lidar_iris_line_aggregator_VERSION)


set(lum_drivers_lidar_iris_line_aggregator_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/lum_drivers_lidar_iris_line_aggregator/include")
set(lum_drivers_lidar_iris_line_aggregator_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/lum_drivers_lidar_iris_line_aggregator/include")
set(lum_drivers_lidar_iris_line_aggregator_INCLUDES "${CMAKE_CURRENT_LIST_DIR}/lum_drivers_lidar_iris_line_aggregator/include")
set(lum_drivers_lidar_iris_line_aggregator_RES_DIRS )
set(lum_drivers_lidar_iris_line_aggregator_DEFINITIONS )
set(lum_drivers_lidar_iris_line_aggregator_LINKER_FLAGS_LIST
        "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:>"
        "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:>"
        "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:>"
)
set(lum_drivers_lidar_iris_line_aggregator_COMPILE_DEFINITIONS )
set(lum_drivers_lidar_iris_line_aggregator_COMPILE_OPTIONS_LIST "" "")
set(lum_drivers_lidar_iris_line_aggregator_COMPILE_OPTIONS_C "")
set(lum_drivers_lidar_iris_line_aggregator_COMPILE_OPTIONS_CXX "")
set(lum_drivers_lidar_iris_line_aggregator_LIBRARIES_TARGETS "") # Will be filled later, if CMake 3
set(lum_drivers_lidar_iris_line_aggregator_LIBRARIES "") # Will be filled later
set(lum_drivers_lidar_iris_line_aggregator_LIBS "") # Same as lum_drivers_lidar_iris_line_aggregator_LIBRARIES
set(lum_drivers_lidar_iris_line_aggregator_SYSTEM_LIBS stdc++)
set(lum_drivers_lidar_iris_line_aggregator_FRAMEWORK_DIRS )
set(lum_drivers_lidar_iris_line_aggregator_FRAMEWORKS )
set(lum_drivers_lidar_iris_line_aggregator_FRAMEWORKS_FOUND "") # Will be filled later
set(lum_drivers_lidar_iris_line_aggregator_BUILD_MODULES_PATHS )

conan_find_apple_frameworks(lum_drivers_lidar_iris_line_aggregator_FRAMEWORKS_FOUND "${lum_drivers_lidar_iris_line_aggregator_FRAMEWORKS}" "${lum_drivers_lidar_iris_line_aggregator_FRAMEWORK_DIRS}")

mark_as_advanced(lum_drivers_lidar_iris_line_aggregator_INCLUDE_DIRS
                 lum_drivers_lidar_iris_line_aggregator_INCLUDE_DIR
                 lum_drivers_lidar_iris_line_aggregator_INCLUDES
                 lum_drivers_lidar_iris_line_aggregator_DEFINITIONS
                 lum_drivers_lidar_iris_line_aggregator_LINKER_FLAGS_LIST
                 lum_drivers_lidar_iris_line_aggregator_COMPILE_DEFINITIONS
                 lum_drivers_lidar_iris_line_aggregator_COMPILE_OPTIONS_LIST
                 lum_drivers_lidar_iris_line_aggregator_LIBRARIES
                 lum_drivers_lidar_iris_line_aggregator_LIBS
                 lum_drivers_lidar_iris_line_aggregator_LIBRARIES_TARGETS)

# Find the real .lib/.a and add them to lum_drivers_lidar_iris_line_aggregator_LIBS and lum_drivers_lidar_iris_line_aggregator_LIBRARY_LIST
set(lum_drivers_lidar_iris_line_aggregator_LIBRARY_LIST lum_drivers_lidar_iris_line_aggregator)
set(lum_drivers_lidar_iris_line_aggregator_LIB_DIRS "${CMAKE_CURRENT_LIST_DIR}/lum_drivers_lidar_iris_line_aggregator/lib")

# Gather all the libraries that should be linked to the targets (do not touch existing variables):
set(_lum_drivers_lidar_iris_line_aggregator_DEPENDENCIES "${lum_drivers_lidar_iris_line_aggregator_FRAMEWORKS_FOUND} ${lum_drivers_lidar_iris_line_aggregator_SYSTEM_LIBS} lum_drivers_lidar_iris_internal_types::lum_drivers_lidar_iris_internal_types;lum_drivers_lidar_iris_types::lum_drivers_lidar_iris_types;lum_common_log::lum_common_log;lum_common_types::lum_common_types;lum_common_types_observable::lum_common_types_observable")

conan_package_library_targets("${lum_drivers_lidar_iris_line_aggregator_LIBRARY_LIST}"  # libraries
                              "${lum_drivers_lidar_iris_line_aggregator_LIB_DIRS}"      # package_libdir
                              "${_lum_drivers_lidar_iris_line_aggregator_DEPENDENCIES}"  # deps
                              lum_drivers_lidar_iris_line_aggregator_LIBRARIES            # out_libraries
                              lum_drivers_lidar_iris_line_aggregator_LIBRARIES_TARGETS    # out_libraries_targets
                              ""                          # build_type
                              "lum_drivers_lidar_iris_line_aggregator")                                      # package_name

set(lum_drivers_lidar_iris_line_aggregator_LIBS ${lum_drivers_lidar_iris_line_aggregator_LIBRARIES})

foreach(_FRAMEWORK ${lum_drivers_lidar_iris_line_aggregator_FRAMEWORKS_FOUND})
    list(APPEND lum_drivers_lidar_iris_line_aggregator_LIBRARIES_TARGETS ${_FRAMEWORK})
    list(APPEND lum_drivers_lidar_iris_line_aggregator_LIBRARIES ${_FRAMEWORK})
endforeach()

foreach(_SYSTEM_LIB ${lum_drivers_lidar_iris_line_aggregator_SYSTEM_LIBS})
    list(APPEND lum_drivers_lidar_iris_line_aggregator_LIBRARIES_TARGETS ${_SYSTEM_LIB})
    list(APPEND lum_drivers_lidar_iris_line_aggregator_LIBRARIES ${_SYSTEM_LIB})
endforeach()

# We need to add our requirements too
set(lum_drivers_lidar_iris_line_aggregator_LIBRARIES_TARGETS "${lum_drivers_lidar_iris_line_aggregator_LIBRARIES_TARGETS};lum_drivers_lidar_iris_internal_types::lum_drivers_lidar_iris_internal_types;lum_drivers_lidar_iris_types::lum_drivers_lidar_iris_types;lum_common_log::lum_common_log;lum_common_types::lum_common_types;lum_common_types_observable::lum_common_types_observable")
set(lum_drivers_lidar_iris_line_aggregator_LIBRARIES "${lum_drivers_lidar_iris_line_aggregator_LIBRARIES};lum_drivers_lidar_iris_internal_types::lum_drivers_lidar_iris_internal_types;lum_drivers_lidar_iris_types::lum_drivers_lidar_iris_types;lum_common_log::lum_common_log;lum_common_types::lum_common_types;lum_common_types_observable::lum_common_types_observable")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/lum_drivers_lidar_iris_line_aggregator/" ${CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/lum_drivers_lidar_iris_line_aggregator/" ${CMAKE_PREFIX_PATH})

if(NOT ${CMAKE_VERSION} VERSION_LESS "3.0")
    # Target approach
    if(NOT TARGET lum_drivers_lidar_iris_line_aggregator::lum_drivers_lidar_iris_line_aggregator)
        add_library(lum_drivers_lidar_iris_line_aggregator::lum_drivers_lidar_iris_line_aggregator INTERFACE IMPORTED)
        if(lum_drivers_lidar_iris_line_aggregator_INCLUDE_DIRS)
            set_target_properties(lum_drivers_lidar_iris_line_aggregator::lum_drivers_lidar_iris_line_aggregator PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                  "${lum_drivers_lidar_iris_line_aggregator_INCLUDE_DIRS}")
        endif()
        set_property(TARGET lum_drivers_lidar_iris_line_aggregator::lum_drivers_lidar_iris_line_aggregator PROPERTY INTERFACE_LINK_LIBRARIES
                     "${lum_drivers_lidar_iris_line_aggregator_LIBRARIES_TARGETS};${lum_drivers_lidar_iris_line_aggregator_LINKER_FLAGS_LIST}")
        set_property(TARGET lum_drivers_lidar_iris_line_aggregator::lum_drivers_lidar_iris_line_aggregator PROPERTY INTERFACE_COMPILE_DEFINITIONS
                     ${lum_drivers_lidar_iris_line_aggregator_COMPILE_DEFINITIONS})
        set_property(TARGET lum_drivers_lidar_iris_line_aggregator::lum_drivers_lidar_iris_line_aggregator PROPERTY INTERFACE_COMPILE_OPTIONS
                     "${lum_drivers_lidar_iris_line_aggregator_COMPILE_OPTIONS_LIST}")
        
        # Library dependencies
        include(CMakeFindDependencyMacro)

        if(NOT lum_drivers_lidar_iris_internal_types_FOUND)
            find_dependency(lum_drivers_lidar_iris_internal_types REQUIRED)
        else()
            message(STATUS "Dependency lum_drivers_lidar_iris_internal_types already found")
        endif()


        if(NOT lum_drivers_lidar_iris_types_FOUND)
            find_dependency(lum_drivers_lidar_iris_types REQUIRED)
        else()
            message(STATUS "Dependency lum_drivers_lidar_iris_types already found")
        endif()


        if(NOT lum_common_log_FOUND)
            find_dependency(lum_common_log REQUIRED)
        else()
            message(STATUS "Dependency lum_common_log already found")
        endif()


        if(NOT lum_common_types_FOUND)
            find_dependency(lum_common_types REQUIRED)
        else()
            message(STATUS "Dependency lum_common_types already found")
        endif()


        if(NOT lum_common_types_observable_FOUND)
            find_dependency(lum_common_types_observable REQUIRED)
        else()
            message(STATUS "Dependency lum_common_types_observable already found")
        endif()

    endif()
endif()

foreach(_BUILD_MODULE_PATH ${lum_drivers_lidar_iris_line_aggregator_BUILD_MODULES_PATHS})
    include(${_BUILD_MODULE_PATH})
endforeach()
