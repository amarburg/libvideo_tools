# Set source files
file(GLOB VIDEOIO_SOURCE_FILES *.cpp
        util/*.cpp
        undistorter/*.cpp )

# build shared library.
fips_begin_module( videoio )
  fips_files( ${VIDEOIO_SOURCE_FILES} )
  fips_libs( ${Boost_LIBRARIES} ${OpenCV_LIBS}
            ${ZLIB_LIBRARIES} ${TINYXML2_LIBRARIES}
            ${YAML_CPP_LIBRARIES} )
  fips_deps( g3logger )

  include_directories(
    ${VIDEOIO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
  )
fips_end_module()
