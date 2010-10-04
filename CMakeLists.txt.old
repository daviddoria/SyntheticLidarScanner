cmake_minimum_required(VERSION 2.6)

PROJECT(SyntheticLidarScanner)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LidarScannerExample ../vtkLidarScannerExample.cxx
../vtkLidarScanner.cxx
../vtkLidarPoint.cxx
../vtkRay.cxx)
TARGET_LINK_LIBRARIES(LidarScannerExample vtkHybrid)

ADD_EXECUTABLE(LidarScannerDemo ../vtkLidarScannerDemo.cxx
../vtkLidarScanner.cxx
../vtkLidarPoint.cxx
../vtkRay.cxx)
TARGET_LINK_LIBRARIES(LidarScannerDemo vtkHybrid)

