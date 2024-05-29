# - Try to find YAMLCPP
# Once done this will define
# YAMLCPP_FOUND - System has YAMLCPP
# YAMLCPP_INCLUDE_DIRS - The YAMLCPP include directories
# YAMLCPP_LIBRARIES - The libraries needed to use YAMLCPP
# YAMLCPP_DEFINITIONS - Compiler switches required for using YAMLCPP

find_path ( YAMLCPP_INCLUDE_DIR yaml-cpp/yaml.h /usr/include /usr/local/include)

find_library ( YAMLCPP_LIBRARY libyaml-cpp.a /usr/lib /usr/local/lib)

set ( YAMLCPP_LIBRARIES ${YAMLCPP_LIBRARY} )
set ( YAMLCPP_INCLUDE_DIRS ${YAMLCPP_INCLUDE_DIR} )

include ( FindPackageHandleStandardArgs )
# Set YAMLCPP_FOUND to TRUE if all listed variables are TRUE
find_package_handle_standard_args ( YAMLCPP DEFAULT_MSG YAMLCPP_LIBRARY YAMLCPP_INCLUDE_DIR )
