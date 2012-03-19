cmake_minimum_required(VERSION 2.6)

PROJECT(SyntheticLidarScanner)

#FIND_PACKAGE(VTK 5.8 REQUIRED) #why doesn't this work?

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

add_library(libLidarScanner vtkLidarScanner.cxx vtkRay.cxx vtkLidarPoint.cxx vtkPTXWriter.cxx Helpers.cpp)
target_link_libraries(libLidarScanner ${VTK_LIBRARIES})

# Demo (programmatically generate data and set parameters)
# ADD_EXECUTABLE(LidarScannerDemo LidarScannerDemo.cpp)
# TARGET_LINK_LIBRARIES(LidarScannerDemo ${VTK_LIBRARIES} LidarScanner)

# Non-interactive version
# ADD_EXECUTABLE(LidarScannerNonInteractive LidarScannerNonInteractive.cpp)
# TARGET_LINK_LIBRARIES(LidarScannerNonInteractive ${VTK_LIBRARIES} LidarScanner)

# Interactive version
#SET(BuildInteractive CACHE BOOL "Build the interactive version.")
#if(BuildInteractive)

if(NOT VTK_USE_QT)
  message(FATAL_ERROR "You must compile VTK with VTK_USE_QT=ON.")
endif(NOT VTK_USE_QT)

FIND_PACKAGE(Qt4)
INCLUDE(${QT_USE_FILE})
list(APPEND compileflags "USE_QT_PARALLEL")

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

QT4_WRAP_UI(UISrcs SyntheticLidarScannerWidget.ui)
QT4_WRAP_CPP(MOCSrcs SyntheticLidarScannerWidget.h)

message("Building SyntheticLidarScanner ")
ADD_EXECUTABLE(SyntheticLidarScanner SyntheticLidarScanner.cpp
SyntheticLidarScannerWidget.cpp
ScannerInteractorStyle.cxx
${MOCSrcs} ${UISrcs}
)
TARGET_LINK_LIBRARIES(SyntheticLidarScanner ${VTK_LIBRARIES} ${QT_LIBRARIES} QVTK libLidarScanner)

#endif(BuildInteractive)
