###############################################################
######                                                   ######
######  	RGBD SLAM                                    ######
######			Author: Pablo R.S. (aka. Bardo91)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
project(ApplePicker)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

# Collect all sources
file(GLOB_RECURSE ARMTEST_SOURCE_FILES "src/*.cpp" "src/*.h" "src/*.inl")

# Clasify sources according to folder structure.
# This creates a nice layout for the project in visual studio
# This macro is courtesy of http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${curdir})
			source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()


GroupSources(src)

#########################################
###### Loading 3rd party libraries.######
#########################################
find_package(RSSDK REQUIRED)

#########################################
######			Executable		   ######
#########################################
set(PROJECT_NAME testRSSDK)

add_executable(${PROJECT_NAME} ${ARMTEST_SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src)

target_include_directories(${PROJECT_NAME} PUBLIC ${RSSDK_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${RSSDK_LIBRARIES})	