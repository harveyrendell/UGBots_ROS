cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(rostest REQUIRED)

rosbuild_add_library(CarrierLib src/robot/carrier.cpp)
#rosbuild_add_library(PickerLib src/robot/picker.cpp)
#rosbuild_add_library(DogLib src/animal/dog.cpp)
#rosbuild_add_library(CatLib src/animal/cat.cpp)
#rosbuild_add_library(PossumLib src/animal/possum.cpp)
#rosbuild_add_library(WorkerLib src/person/worker.cpp)
#rosbuild_add_library(VisitorLib src/person/visitor.cpp)


#Executable nodes
rosbuild_add_executable(CARRIER src/robot/carrier.cpp)
rosbuild_add_executable(PICKER src/robot/picker.cpp)
rosbuild_add_executable(DOG src/animal/dog.cpp)
rosbuild_add_executable(CAT src/animal/cat.cpp)
rosbuild_add_executable(POSSUM src/animal/possum.cpp)
rosbuild_add_executable(WORKER src/person/worker.cpp)
rosbuild_add_executable(VISITOR src/person/visitor.cpp)

#Testing
#rosbuild_add_executable(UnitTests EXCLUDE_FROM_ALL test/unit_tests.cpp)
rosbuild_add_gtest(UnitTests test/unit_tests.cpp)
#rosbuild_add_rostest(test/mynode.test)
target_link_libraries(UnitTests CarrierLib)
