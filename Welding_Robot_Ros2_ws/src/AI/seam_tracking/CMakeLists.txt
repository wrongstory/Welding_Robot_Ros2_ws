cmake_minimum_required(VERSION 3.16)
project(seam_tracking LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(HailoRT REQUIRED)          # /usr/lib/cmake/HailoRT -> HailoRT::libhailort
find_package(OpenCV REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)  # pybind11_add_module 요구
find_package(pybind11 REQUIRED)         # ament: pybind11_vendor

# ---- C++ 코어 라이브러리 (HailoRT + 후처리) ----
add_library(seam_tracking_core
  cpp/src/hailo_seg_engine.cpp
  cpp/src/yolov8_seg_postprocess.cpp
  cpp/src/seam_centerline.cpp
)
target_include_directories(seam_tracking_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(seam_tracking_core PUBLIC HailoRT::libhailort ${OpenCV_LIBS})
target_include_directories(seam_tracking_core PUBLIC ${OpenCV_INCLUDE_DIRS})
# 정적 코어를 pybind11 공유 모듈(.so)에 링크 → -fPIC 필요
set_target_properties(seam_tracking_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- pybind11 파이썬 확장 모듈: seam_tracking_cpp ----
pybind11_add_module(seam_tracking_cpp cpp/src/bindings.cpp)
target_link_libraries(seam_tracking_cpp PRIVATE seam_tracking_core)

# 파이썬 확장 모듈(.so)과 UI 스크립트를 같은 디렉토리에 설치 → UI 가 sys.path 로 co-locate import
install(TARGETS seam_tracking_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# UI 실행 스크립트 + import 모듈(camera_worker)을 .so 와 같은 lib/ 에 co-locate
install(PROGRAMS
  ui/seam_tracking_gui.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  ui/camera_worker.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  ui/seam_tracking.ui
  DESTINATION share/${PROJECT_NAME}/ui
)

# HEF 모델 (share 에 설치; models/ 는 심볼릭/복사)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  OPTIONAL
)

install(TARGETS seam_tracking_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY cpp/include/ DESTINATION include)

ament_package()
