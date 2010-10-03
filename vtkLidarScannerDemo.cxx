/*
example parameters:
./PerformScan Sphere.vtp Sphere_latest.vtp 1 0 -2 0 0 0 0 2 3 .3 .3 1
*/

#include <vector>
#include <sstream>

#include "vtkLidarScanner.h"
#include "vtkLidarPoint.h"

#include "vtkSmartPointer.h"
#include "vtkMath.h"
#include "vtkPolyData.h"
#include "vtkXMLPolyDataReader.h"
#include "vtkXMLPolyDataWriter.h"
#include "vtkTransform.h"

int main(int argc, char* argv[])
{
  if(argc != 15)
  {
    std::cout << "Incorrect arguments! Required args:" << std::endl
            << "InputFilename OutputFilename CreateMesh? Tx Ty Tz Rx Ry Rz (R in degrees - dictated by VTK) NumThetaPoints NumPhiPoints ThetaSpan(radians) PhiSpan(radians) StoreRays? " << std::endl;
    return EXIT_FAILURE;
  }
  //get the input/output filenames from the command line
  vtkstd::string inputFilename = argv[1];
  vtkstd::string outputFilename = argv[2];
  vtkstd::string strCreateMesh = argv[3];
  vtkstd::string strTx = argv[4];
  vtkstd::string strTy = argv[5];
  vtkstd::string strTz = argv[6];
  vtkstd::string strRx = argv[7];
  vtkstd::string strRy = argv[8];
  vtkstd::string strRz = argv[9];
  vtkstd::string strThetaPoints = argv[10];
  vtkstd::string strPhiPoints = argv[11];
  vtkstd::string strThetaSpan = argv[12];
  vtkstd::string strPhiSpan = argv[13];
  vtkstd::string strStoreRays = argv[14];
  
  vtkstd::cout << "InputFilename: " << inputFilename << std::endl;
  vtkstd::cout << "OutputFilename: " << outputFilename << std::endl;
  
  //convert string to bool
  vtkstd::stringstream ssCreateMesh(strCreateMesh);
  bool createMesh;
  ssCreateMesh >> createMesh;
  
  vtkstd::cout << "Create Mesh? " << createMesh << vtkstd::endl;
  
  //convert strings to doubles
  double tx, ty, tz, rx, ry, rz;
  
  std::stringstream ssTx(strTx);
  ssTx >> tx;
  
  std::stringstream ssTy(strTy);
  ssTy >> ty;
  
  std::stringstream ssTz(strTz);
  ssTz >> tz;
  
  std::stringstream ssRx(strRx);
  ssRx >> rx;
  
  std::stringstream ssRy(strRy);
  ssRy >> ry;
  
  std::stringstream ssRz(strRz);
  ssRz >> rz;
  
  vtkstd::cout << "Tx: " << tx << vtkstd::endl;
  vtkstd::cout << "Ty: " << ty << vtkstd::endl;
  vtkstd::cout << "Tz: " << tz << vtkstd::endl;
  vtkstd::cout << "Rx: " << rx << vtkstd::endl;
  vtkstd::cout << "Ry: " << ry << vtkstd::endl;
  vtkstd::cout << "Rz: " << rz << vtkstd::endl;
    
  unsigned int thetaPoints;
  unsigned int phiPoints;
  
  vtkstd::stringstream ssThetaPoints(strThetaPoints);
  ssThetaPoints >> thetaPoints;
  
  vtkstd::stringstream ssPhiPoints(strPhiPoints);
  ssPhiPoints >> phiPoints;
      
  vtkstd::cout << "Theta points: " << thetaPoints << vtkstd::endl;
  vtkstd::cout << "Phi points: " << phiPoints << vtkstd::endl;
      
  double thetaSpan, phiSpan;
  
  vtkstd::stringstream ssThetaSpan(strThetaSpan);
  ssThetaSpan >> thetaSpan;
  vtkstd::cout << "Theta span: " << thetaSpan << vtkstd::endl;
  
  vtkstd::stringstream ssPhiSpan(strPhiSpan);
  ssPhiSpan >> phiSpan;
  vtkstd::cout << "Phi span: " << phiSpan << vtkstd::endl;
  
  //convert string to bool
  vtkstd::stringstream ssStoreRays(strStoreRays);
  bool storeRays;
  ssStoreRays >> storeRays;
  
  std::cout << "Store Rays? " << storeRays << std::endl;
  
  //read the input vtp file
  vtkSmartPointer<vtkXMLPolyDataReader> reader = 
      vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();
  
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
  
  scanner->SetInputConnection(reader->GetOutputPort());
  scanner->Update();
  
  //create a writer and write the output vtp file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(outputFilename.c_str());
  writer->SetInputConnection(scanner->GetOutputPort());
  writer->Write();
	
  return EXIT_SUCCESS;
}
