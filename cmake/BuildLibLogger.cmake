

IF( DEFINED LOCAL_LIBLOGGER )
	message( "Building local copy of LibLogger")

	set( LIBLOGGER_DIR ${LOCAL_LIBLOGGER} )

	include( ${LOCAL_LIBLOGGER}/cmake/BuildLibActiveObject.cmake )

	include_directories( ${LIBLOGGER_DIR}/include
												${LIBACTIVE_OBJECT_INCLUDE_DIRS} )

	add_subdirectory( ${LOCAL_LIBLOGGER}/lib liblogger )


ELSE()

	# SET( G3LOG_PREFIX_DIR ${PROJECT_BINARY_DIR}/g3log )
	# SET( G3LOG_INSTALL_DIR ${G3LOG_PREFIX_DIR} )
	# SET( G3LOG_SOURCE_DIR ${G3LOG_PREFIX_DIR}/src/g3log )
	#
	# SET( G3LOG_CMAKE_OPTS -DADD_FATAL_EXAMPLE:bool=OFF  )
	# IF( DEFINED CMAKE_BUILD_TYPE )
	# 	LIST(APPEND G3LOG_CMAKE_OPTS -DCMAKE_BUILD_TYPE:string=${CMAKE_BUILD_TYPE} )
	# 	IF( ${CMAKE_BUILD_TYPE} STREQUAL Release )
	# 		message( "Disabling the DEBUG level in G3LOG" )
	# 		LIST(APPEND G3LOG_CMAKE_OPTS -DIGNORE_DEBUG_LEVEL:bool=ON )
	# 	ENDIF()
	# ENDIF()
	#
	# ## Uses my fork which doesn't create src/g3log/generated_definitions.hpp
	# ## And thus doesn't need to be re-built every time...
	# ExternalProject_Add( g3log
	# 										GIT_REPOSITORY https://github.com/amarburg/g3log.git
	# 										PREFIX g3log
	# 										UPDATE_COMMAND git pull origin master
	# 										BUILD_COMMAND ${EXTERNAL_PROJECT_MAKE_COMMAND}
	# 										CMAKE_ARGS ${G3LOG_CMAKE_OPTS}
	#   									INSTALL_COMMAND "" )
	#
	# ## g3log doesn't have an "install" target
	# set( G3LOG_INCLUDE_DIR ${G3LOG_SOURCE_DIR}/src )
	# set( G3LOG_LIB_DIR ${G3LOG_PREFIX_DIR}/src/g3log-build/ )
	# link_directories(
	#   ${G3LOG_LIB_DIR}
	# )
	#
	# set( G3LOG_LIB g3logger )
	#
	# set_target_properties(g3log PROPERTIES EXCLUDE_FROM_ALL TRUE)

ENDIF()

SET( LIBLOGGER_INCLUDE_DIRS
			${LIBACTIVE_OBJECT_INCLUDE_DIRS}
			${LIBLOGGER_DIR}/include
 		)

SET( LIBLOGGER_LIBS logger active_object)