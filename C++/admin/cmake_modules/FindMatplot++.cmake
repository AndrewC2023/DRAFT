find_path(MATPLOT_INCLUDE_DIRS "matplot/matplot.h")
find_library(MATPLOT_LIBRARIES "matplot")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Matplot++ DEFAULT_MSG
                                  MATPLOT_INCLUDE_DIRS
                                  MATPLOT_LIBRARIES)

if (MATPLOT_LIBRARIES)
    set(Matplot++_FOUND TRUE)
endif ()

mark_as_advanced(MATPLOT_INCLUDE_DIRS MATPLOT_LIBRARIES)