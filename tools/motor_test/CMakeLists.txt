cmake_minimum_required(VERSION 3.16)
project(motor_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)

add_executable(motor_test motor_test.cpp)
target_link_libraries(motor_test PRIVATE Threads::Threads)

# --- SOEM 탐색 ---
# 1순위: 설치된 CMake 패키지(soemConfig.cmake). 2순위: ~/.local 등 수동 경로.
find_package(soem QUIET)
if(soem_FOUND)
  message(STATUS "SOEM: find_package(soem) 사용")
  target_link_libraries(motor_test PRIVATE soem)
else()
  find_path(SOEM_INCLUDE_DIR soem/ethercat.h
    HINTS $ENV{HOME}/.local/include /usr/local/include /usr/include)
  find_library(SOEM_LIBRARY NAMES soem
    HINTS $ENV{HOME}/.local/lib /usr/local/lib /usr/lib)
  if(NOT SOEM_INCLUDE_DIR OR NOT SOEM_LIBRARY)
    message(FATAL_ERROR
      "SOEM 미발견. 소스 빌드 후 ~/.local 에 설치하세요:\n"
      "  git clone --depth 1 https://github.com/OpenEtherCATsociety/SOEM.git\n"
      "  cd SOEM && cmake -B build -DCMAKE_INSTALL_PREFIX=$HOME/.local && cmake --build build -j4 && cmake --install build")
  endif()
  message(STATUS "SOEM include: ${SOEM_INCLUDE_DIR}")
  message(STATUS "SOEM lib    : ${SOEM_LIBRARY}")
  target_include_directories(motor_test PRIVATE ${SOEM_INCLUDE_DIR})
  target_link_libraries(motor_test PRIVATE ${SOEM_LIBRARY})
  # SOEM(Linux)은 pthread + rt 필요
  find_library(RT_LIBRARY rt)
  if(RT_LIBRARY)
    target_link_libraries(motor_test PRIVATE ${RT_LIBRARY})
  endif()
endif()
