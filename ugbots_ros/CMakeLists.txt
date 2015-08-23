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
rosbuild_genmsg()
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

#Executable nodes
rosbuild_add_executable(CARRIER src/robot/carrier.cpp)
rosbuild_add_executable(PICKER src/robot/picker.cpp)
rosbuild_add_executable(DOG src/animal/dog.cpp)
rosbuild_add_executable(CAT src/animal/cat.cpp)
rosbuild_add_executable(POSSUM src/animal/possum.cpp)
rosbuild_add_executable(WORKER src/person/worker.cpp)
rosbuild_add_executable(VISITOR src/person/visitor.cpp)
rosbuild_add_executable(BEACON src/core/beacon.cpp)
rosbuild_add_executable(CORE src/core/core.cpp)

rosbuild_add_library(CarrierLib src/robot/carrier.cpp)
rosbuild_add_library(PickerLib src/robot/picker.cpp)
rosbuild_add_library(DogLib src/animal/dog.cpp)
rosbuild_add_library(CatLib src/animal/cat.cpp)
rosbuild_add_library(PossumLib src/animal/possum.cpp)
rosbuild_add_library(WorkerLib src/person/worker.cpp)
rosbuild_add_library(VisitorLib src/person/visitor.cpp)

#Carrier
#rosbuild_add_executable(CarrierTests EXCLUDE_FROM_ALL test/carrier_test.cpp)
rosbuild_add_gtest(CarrierTests test/carrier_test.cpp)
target_link_libraries(CarrierTests CarrierLib)

#Picker
#rosbuild_add_executable(PickerTests EXCLUDE_FROM_ALL test/picker_test.cpp)
rosbuild_add_gtest(PickerTests test/picker_test.cpp)
target_link_libraries(PickerTests PickerLib)

#Dog
#rosbuild_add_executable(DogTests EXCLUDE_FROM_ALL test/dog_test.cpp)
rosbuild_add_gtest(DogTests test/dog_test.cpp)
target_link_libraries(DogTests DogLib)

#Cat
#rosbuild_add_executable(CatTests EXCLUDE_FROM_ALL test/cat_test.cpp)
rosbuild_add_gtest(CatTests test/cat_test.cpp)
target_link_libraries(CatTests CatLib)

#Possum
#rosbuild_add_executable(PossumTests EXCLUDE_FROM_ALL test/possum_test.cpp)
rosbuild_add_gtest(PossumTests test/possum_test.cpp)
target_link_libraries(PossumTests PossumLib)

#Worker
#rosbuild_add_executable(WorkerTests EXCLUDE_FROM_ALL test/worker_test.cpp)
rosbuild_add_gtest(WorkerTests test/worker_test.cpp)
target_link_libraries(WorkerTests WorkerLib)

#Visitor
#rosbuild_add_executable(VisitorTests EXCLUDE_FROM_ALL test/visitor_test.cpp)
rosbuild_add_gtest(VisitorTests test/visitor_test.cpp)
target_link_libraries(VisitorTests VisitorLib)

#Using rostest
#rosbuild_add_rostest(test/mynode.test)
