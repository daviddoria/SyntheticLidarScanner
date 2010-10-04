cmake_minimum_required(VERSION 2.6)

PROJECT(SyntheticLidarScanner)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

QT4_WRAP_UI(UISrcs form.ui)
QT4_WRAP_CPP(MOCSrcs form.h)

ADD_EXECUTABLE(SyntheticLidarScanner SyntheticLidarScanner.cpp vtkLidarScanner.cxx vtkRay.cxx vtkLidarPoint.cxx vtkPTXWriter.cxx
form.cpp
${MOCSrcs} ${UISrcs}
)
TARGET_LINK_LIBRARIES(SyntheticLidarScanner vtkHybrid vtkInfovis vtkViews ${QT_LIBRARIES} QVTK)
