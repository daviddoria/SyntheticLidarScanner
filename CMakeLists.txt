cmake_minimum_required(VERSION 2.6)

PROJECT(SyntheticLidarScanner)

#FIND_PACKAGE(VTK 5.8 REQUIRED) #why doesn't this work?

FIND_PACKAGE(VTK REQUIRED)

if(VTK_FOUND)
  if("${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}" LESS 5.8)
    message(FATAL_ERROR "VTK is not a high enough version!")
  else()
    INCLUDE(${VTK_USE_FILE})
  endif()
endif()

# Demo (programmatically generate data and set parameters)
ADD_EXECUTABLE(LidarScannerDemo LidarScannerDemo.cpp vtkLidarScanner.cxx vtkRay.cxx vtkLidarPoint.cxx vtkPTXWriter.cxx)
TARGET_LINK_LIBRARIES(LidarScannerDemo ${VTK_LIBRARIES})

# Non-interactive version
ADD_EXECUTABLE(LidarScannerNonInteractive LidarScannerNonInteractive.cpp vtkLidarScanner.cxx vtkRay.cxx vtkLidarPoint.cxx vtkPTXWriter.cxx)
TARGET_LINK_LIBRARIES(LidarScannerNonInteractive ${VTK_LIBRARIES})

# Interactive version
SET(BuildInteractive CACHE BOOL "Build the interactive version.")
if(BuildInteractive)

  if(NOT VTK_USE_QT)
    message(FATAL_ERROR "You must compile VTK with VTK_USE_QT=ON.")
  endif(NOT VTK_USE_QT)

  FIND_PACKAGE(Qt4)
  if(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
    list(APPEND compileflags "USE_QT_PARALLEL")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

    QT4_WRAP_UI(UISrcs SyntheticLidarScannerWidget.ui)
    QT4_WRAP_CPP(MOCSrcs SyntheticLidarScannerWidget.h)

    ADD_EXECUTABLE(SyntheticLidarScanner SyntheticLidarScanner.cpp vtkLidarScanner.cxx vtkRay.cxx vtkLidarPoint.cxx vtkPTXWriter.cxx
    SyntheticLidarScannerWidget.cpp
    ScannerInteractorStyle.cxx
    ${MOCSrcs} ${UISrcs}
    )
    TARGET_LINK_LIBRARIES(SyntheticLidarScanner ${VTK_LIBRARIES} ${QT_LIBRARIES} QVTK)
  elseif(QT4_FOUND)
    message ( "QT4 NOT found successfully." )
  endif(QT4_FOUND)
endif(BuildInteractive)
