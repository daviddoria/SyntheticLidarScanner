cmake_minimum_required(VERSION 2.6)

PROJECT(SyntheticLidarScanner)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

if( "${VTK_MAJOR_VERSION}" LESS 6 )
  MESSAGE(FATAL_ERROR "You must build this code with VTK >= 6.0!")
endif( "${VTK_MAJOR_VERSION}" LESS 6 )

add_library(LidarScanner vtkLidarScanner.cxx vtkRay.cxx vtkLidarPoint.cxx vtkPTXWriter.cxx Helpers.cpp)
target_link_libraries(LidarScanner ${VTK_LIBRARIES})

# Demo (programmatically generate data and set parameters)
# ADD_EXECUTABLE(LidarScannerDemo LidarScannerDemo.cpp)
# TARGET_LINK_LIBRARIES(LidarScannerDemo ${VTK_LIBRARIES} LidarScanner)

# Non-interactive version
ADD_EXECUTABLE(LidarScannerNonInteractive LidarScannerNonInteractive.cpp)
TARGET_LINK_LIBRARIES(LidarScannerNonInteractive ${VTK_LIBRARIES} LidarScanner)

# Interactive version
option(BuildInteractive "Build the interactive version?")
if(BuildInteractive)
  message("Building SyntheticLidarScanner ")

  if(NOT vtkGUISupportQt_LOADED)
    message(FATAL_ERROR "You must build VTK with the vtkGUISupportQt module enabled!")
  endif()

  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE(${QT_USE_FILE})
  list(APPEND compileflags "USE_QT_PARALLEL")

  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  QT4_WRAP_UI(UISrcs SyntheticLidarScannerWidget.ui)
  QT4_WRAP_CPP(MOCSrcs SyntheticLidarScannerWidget.h)

  ADD_EXECUTABLE(SyntheticLidarScanner SyntheticLidarScanner.cpp
  SyntheticLidarScannerWidget.cpp
  ScannerInteractorStyle.cxx
  ${MOCSrcs} ${UISrcs}
  )
  TARGET_LINK_LIBRARIES(SyntheticLidarScanner ${VTK_LIBRARIES} LidarScanner)

endif(BuildInteractive)
