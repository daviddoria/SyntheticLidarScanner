#include <vector>
#include <sstream>

#include "vtkLidarScanner.h"
#include "vtkLidarPoint.h"

#include <vtkSmartPointer.h>
#include <vtkMath.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkTransform.h>
#include <vtkSphereSource.h>

int main(int argc, char* argv[])
{
  bool createMesh = true;
  
  //convert strings to doubles
  double tx = 0;
  double ty = -2;
  double tz = 0;
  double rx = 0;
  double ry = 0;
  double rz = 0;
  
  unsigned int thetaPoints = 2;
  unsigned int phiPoints = 3;
  
  double thetaSpan = .3;
  double phiSpan = .3;
  
  bool storeRays = true;
  
  //create a sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  {
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("ExampleInput.vtp");
  writer->SetInputConnection(sphereSource->GetOutputPort());
  writer->Write();
  }
  
  //construct a vtkLidarScanner and set all of its parameters
  vtkSmartPointer<vtkLidarScanner> scanner = 
      vtkSmartPointer<vtkLidarScanner>::New();
  
  //Scanner->WriteScanner("scanner_original.vtp");
          
  //double testAngle = vtkMath::Pi()/4.0;
  scanner->SetPhiSpan(phiSpan);
  scanner->SetThetaSpan(thetaSpan);
          
  scanner->SetNumberOfThetaPoints(thetaPoints);
  scanner->SetNumberOfPhiPoints(phiPoints);
  
  scanner->SetStoreRays(storeRays);
  
  //"aim" the scanner.  This is a very simple translation, but any transformation will work
  vtkSmartPointer<vtkTransform> transform = 
      vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply();
  
  transform->RotateX(rx);
  transform->RotateY(ry);
  transform->RotateZ(rz);
  transform->Translate(tx, ty, tz);
  
  scanner->SetTransform(transform);
  scanner->WriteScanner("scanner_transformed.vtp");
  
  scanner->MakeSphericalGrid(); //indicate to use uniform spherical spacing
  
  scanner->SetCreateMesh(createMesh);
  
  scanner->SetInputConnection(sphereSource->GetOutputPort());
  scanner->Update();
  
  //create a writer and write the output vtp file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("ExampleScan.vtp");
  writer->SetInputConnection(scanner->GetOutputPort());
  writer->Write();
	
  return EXIT_SUCCESS;
}
