

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

conan_message(STATUS "Conan: Using autogenerated Findlum_common_log.cmake")
# Global approach
set(lum_common_log_FOUND 1)
set(lum_common_log_VERSION "1.0.0")

find_package_handle_standard_args(lum_common_log REQUIRED_VARS
                                  lum_common_log_VERSION VERSION_VAR lum_common_log_VERSION)
mark_as_advanced(lum_common_log_FOUND lum_common_log_VERSION)


set(lum_common_log_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/lum_common_log/include")
set(lum_common_log_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/lum_common_log/include")
set(lum_common_log_INCLUDES "${CMAKE_CURRENT_LIST_DIR}/lum_common_log/include")
set(lum_common_log_RES_DIRS )
set(lum_common_log_DEFINITIONS )
set(lum_common_log_LINKER_FLAGS_LIST
        "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:>"
        "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:>"
        "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:>"
)
set(lum_common_log_COMPILE_DEFINITIONS )
set(lum_common_log_COMPILE_OPTIONS_LIST "" "")
set(lum_common_log_COMPILE_OPTIONS_C "")
set(lum_common_log_COMPILE_OPTIONS_CXX "")
set(lum_common_log_LIBRARIES_TARGETS "") # Will be filled later, if CMake 3
set(lum_common_log_LIBRARIES "") # Will be filled later
set(lum_common_log_LIBS "") # Same as lum_common_log_LIBRARIES
set(lum_common_log_SYSTEM_LIBS stdc++)
set(lum_common_log_FRAMEWORK_DIRS )
set(lum_common_log_FRAMEWORKS )
set(lum_common_log_FRAMEWORKS_FOUND "") # Will be filled later
set(lum_common_log_BUILD_MODULES_PATHS )

conan_find_apple_frameworks(lum_common_log_FRAMEWORKS_FOUND "${lum_common_log_FRAMEWORKS}" "${lum_common_log_FRAMEWORK_DIRS}")

mark_as_advanced(lum_common_log_INCLUDE_DIRS
                 lum_common_log_INCLUDE_DIR
                 lum_common_log_INCLUDES
                 lum_common_log_DEFINITIONS
                 lum_common_log_LINKER_FLAGS_LIST
                 lum_common_log_COMPILE_DEFINITIONS
                 lum_common_log_COMPILE_OPTIONS_LIST
                 lum_common_log_LIBRARIES
                 lum_common_log_LIBS
                 lum_common_log_LIBRARIES_TARGETS)

# Find the real .lib/.a and add them to lum_common_log_LIBS and lum_common_log_LIBRARY_LIST
set(lum_common_log_LIBRARY_LIST lum_common_log)
set(lum_common_log_LIB_DIRS "${CMAKE_CURRENT_LIST_DIR}/lum_common_log/lib")

# Gather all the libraries that should be linked to the targets (do not touch existing variables):
set(_lum_common_log_DEPENDENCIES "${lum_common_log_FRAMEWORKS_FOUND} ${lum_common_log_SYSTEM_LIBS} lum_common_types::lum_common_types;lum_common_casts::lum_common_casts;lum_platform_process::lum_platform_process")

conan_package_library_targets("${lum_common_log_LIBRARY_LIST}"  # libraries
                              "${lum_common_log_LIB_DIRS}"      # package_libdir
                              "${_lum_common_log_DEPENDENCIES}"  # deps
                              lum_common_log_LIBRARIES            # out_libraries
                              lum_common_log_LIBRARIES_TARGETS    # out_libraries_targets
                              ""                          # build_type
                              "lum_common_log")                                      # package_name

set(lum_common_log_LIBS ${lum_common_log_LIBRARIES})

foreach(_FRAMEWORK ${lum_common_log_FRAMEWORKS_FOUND})
    list(APPEND lum_common_log_LIBRARIES_TARGETS ${_FRAMEWORK})
    list(APPEND lum_common_log_LIBRARIES ${_FRAMEWORK})
endforeach()

foreach(_SYSTEM_LIB ${lum_common_log_SYSTEM_LIBS})
    list(APPEND lum_common_log_LIBRARIES_TARGETS ${_SYSTEM_LIB})
    list(APPEND lum_common_log_LIBRARIES ${_SYSTEM_LIB})
endforeach()

# We need to add our requirements too
set(lum_common_log_LIBRARIES_TARGETS "${lum_common_log_LIBRARIES_TARGETS};lum_common_types::lum_common_types;lum_common_casts::lum_common_casts;lum_platform_process::lum_platform_process")
set(lum_common_log_LIBRARIES "${lum_common_log_LIBRARIES};lum_common_types::lum_common_types;lum_common_casts::lum_common_casts;lum_platform_process::lum_platform_process")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/lum_common_log/" ${CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/lum_common_log/" ${CMAKE_PREFIX_PATH})

if(NOT ${CMAKE_VERSION} VERSION_LESS "3.0")
    # Target approach
    if(NOT TARGET lum_common_log::lum_common_log)
        add_library(lum_common_log::lum_common_log INTERFACE IMPORTED)
        if(lum_common_log_INCLUDE_DIRS)
            set_target_properties(lum_common_log::lum_common_log PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                  "${lum_common_log_INCLUDE_DIRS}")
        endif()
        set_property(TARGET lum_common_log::lum_common_log PROPERTY INTERFACE_LINK_LIBRARIES
                     "${lum_common_log_LIBRARIES_TARGETS};${lum_common_log_LINKER_FLAGS_LIST}")
        set_property(TARGET lum_common_log::lum_common_log PROPERTY INTERFACE_COMPILE_DEFINITIONS
                     ${lum_common_log_COMPILE_DEFINITIONS})
        set_property(TARGET lum_common_log::lum_common_log PROPERTY INTERFACE_COMPILE_OPTIONS
                     "${lum_common_log_COMPILE_OPTIONS_LIST}")
        
        # Library dependencies
        include(CMakeFindDependencyMacro)

        if(NOT lum_common_types_FOUND)
            find_dependency(lum_common_types REQUIRED)
        else()
            message(STATUS "Dependency lum_common_types already found")
        endif()


        if(NOT lum_common_casts_FOUND)
            find_dependency(lum_common_casts REQUIRED)
        else()
            message(STATUS "Dependency lum_common_casts already found")
        endif()


        if(NOT lum_platform_process_FOUND)
            find_dependency(lum_platform_process REQUIRED)
        else()
            message(STATUS "Dependency lum_platform_process already found")
        endif()

    endif()
endif()

foreach(_BUILD_MODULE_PATH ${lum_common_log_BUILD_MODULES_PATHS})
    include(${_BUILD_MODULE_PATH})
endforeach()
