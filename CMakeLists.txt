cmake_minimum_required(VERSION 3.5)
project (test-joint-limits)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(TSID REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

list(APPEND DEPENDENCIES_LIBS
    ${TSID_LIBRARY}
    ${EIGEN3_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    pinocchio::pinocchio
)
list(APPEND DEPENDENCIES_INCS
    ${TSID_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${BOOST_INCLUDE_DIRS}
)

add_executable(test-lim src/test-lim.cpp)
target_link_libraries(test-lim
    PUBLIC 
    ${DEPENDENCIES_LIBS})

target_include_directories(test-lim
    PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${DEPENDENCIES_INCS}
)
