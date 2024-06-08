find_path(MATPLOT_INCLUDE_DIR 
    NAMES matplot/matplot.h 
    HINTS
    /usr/include 
    /usr/local/include 
    ${CMAKE_SOURCE_DIR}/admin/reqLib/include)

find_library(MATPLOT_LIBRARIES 
    NAMES libmatplot.a 
    HINTS
    /usr/lib 
    /usr/local/lib 
    ${CMAKE_SOURCE_DIR}/admin/reqLib/lib)

# Check if both include directory and library are found
if (MATPLOT_INCLUDE_DIR AND MATPLOT_LIBRARIES)
    # Matplot++ found
    message(STATUS "Matplot++ found")
    set(Matplot++_FOUND TRUE)
    
    # Create an imported target for Matplot++
    add_library(Matplot++ INTERFACE IMPORTED)
    
    # Set the include directory
    target_include_directories(Matplot++ INTERFACE ${MATPLOT_INCLUDE_DIR})
    
    # Set the library
    target_link_libraries(Matplot++ INTERFACE ${MATPLOT_LIBRARIES})
else ()
    # Matplot++ not found
    message(WARNING "Matplot++ not found. Please set MATPLOT_INCLUDE_DIR and MATPLOT_LIBRARIES variables manually.")
endif ()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Matplot++ DEFAULT_MSG
                                  MATPLOT_INCLUDE_DIR
                                  MATPLOT_LIBRARIES)