SET(execName RTSTool)
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(${execName})

set(CMAKE_CXX_STANDARD 14)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)  
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${Boost_INCLUDE_DIR}
)

link_directories(${Boost_LIBRARY_DIRS})
add_definitions(-DBOOST_ALL_DYN_LINK)

add_definitions(-DCONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config.json" -DPCD_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-format")

if(PCL_FOUND)
    MESSAGE(STATUS PCL_VERSION${PCL_VERSION})
    include_directories(${Pangolin_INCLUDE_DIRS})
    include_directories(${PCL_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

    link_directories(${PCL_LIBRARY_DIRS})
    link_directories(${OpenCV_LIBRARY_DIRS})
    FILE(GLOB_RECURSE SRC_FILES src/*.cpp src/*.c) 
    FILE(GLOB_RECURSE HDR_FILES src/*.h)
    FILE(GLOB_RECURSE HDR_FILES include/*.hpp)


    ADD_EXECUTABLE(${execName} ${SRC_FILES})
    add_definitions(-std=c++14 -g -O3)

    TARGET_LINK_LIBRARIES(${execName} 
			   ${OpenCV_LIBS}
                ${SPECIAL_OS_LIBS}
                ${OpenCV_LIBRARIES}
                ${PCL_LIBRARIES}
			${Pangolin_LIBRARIES}
            ${Boost_LIBRARIES}
)                            
endif()
