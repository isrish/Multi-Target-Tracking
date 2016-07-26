project(MTTracker)
cmake_minimum_required(VERSION 2.8)


set(CMAKE_CXX_FLAGS "-fpermissive -std=c++0x -Wno-literal-suffix")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -g")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(SHARED_LIBS_DEFAULT ON)




set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMAKE_HELPERS)

find_package(Boost 1.57 COMPONENTS program_options thread system REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
find_package( Eigen2 REQUIRED )


file(GLOB srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
set(srcs ${srcs})

add_library(tracking SHARED ${srcs})

file(GLOB Detectorsrcs ${CMAKE_CURRENT_SOURCE_DIR}/Det/*.cpp)
add_library(DetNodes ${Detectorsrcs})



add_executable(tracker mainMulti.cpp)
target_link_libraries(tracker tracking  DetNodes ${REQUIRED_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} -lpthread -lrt -fopenmp )


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Det)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)




