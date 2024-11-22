# - Try to find YAMLCPP
# Once done this will define
# YAMLCPP_FOUND - System has YAMLCPP
# YAMLCPP_INCLUDE_DIRS - The YAMLCPP include directories
# YAMLCPP_LIBRARIES - The libraries needed to use YAMLCPP
# YAMLCPP_DEFINITIONS - Compiler switches required for using YAMLCPP

find_path ( YAMLCPP_INCLUDE_DIR 
            yaml.h 
            /usr/include/yaml-cpp  
            /usr/local/include/yaml-cpp)

# First, try to find libyaml-cpp.a
find_library ( YAMLCPP_OLD_LIBRARY NAMES libyaml-cpp.a 
               PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib)

# If libyaml-cpp.a is not found, try to find libyaml.a
if (NOT YAMLCPP_OLD_LIBRARY)
    find_library ( YAMLCPP_LIBRARY NAMES libyaml.a 
                   PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib)
else()
    set ( YAMLCPP_LIBRARY ${YAMLCPP_OLD_LIBRARY} )
endif()

set ( YAMLCPP_LIBRARIES ${YAMLCPP_LIBRARY} )
set ( YAMLCPP_INCLUDE_DIRS ${YAMLCPP_INCLUDE_DIR} )

if (YAMLCPP_INCLUDE_DIRS AND YAMLCPP_LIBRARY)
    message(STATUS "yaml-cpp found")
endif()

include ( FindPackageHandleStandardArgs )
# Set YAMLCPP_FOUND to TRUE if all listed variables are TRUE
find_package_handle_standard_args ( yaml-cpp  DEFAULT_MSG YAMLCPP_LIBRARY YAMLCPP_INCLUDE_DIR )
