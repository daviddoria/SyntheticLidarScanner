/*=========================================================================
 *
 *  Copyright David Doria 2011 daviddoria@gmail.com
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

/*
example parameters:
LidarScannerNonInteractive data/sphere_origin.vtp output.vti 1 5 0 0 0 0 -90 100 100 .3 .3 1
*/

#include <vector>
#include <sstream>

#include "vtkLidarScanner.h"
#include "vtkLidarPoint.h"

#include "vtkSmartPointer.h"
#include "vtkMath.h"
#include "vtkImageData.h"
#include "vtkPolyData.h"
#include "vtkXMLPolyDataReader.h"
#include "vtkXMLImageDataWriter.h"
#include "vtkXMLPolyDataWriter.h"
#include "vtkTransform.h"

int main(int argc, char* argv[])
{
  if(argc != 15)
  {
    std::cout << "Incorrect arguments! Required args:" << std::endl
            << "InputFilename OutputPrefix CreateMesh? Tx Ty Tz Rx Ry Rz (R in degrees - dictated by VTK) "
               "NumThetaPoints NumPhiPoints ThetaSpan(radians) PhiSpan(radians) StoreRays? " << std::endl;
    return EXIT_FAILURE;
  }

  // Get the input/output filenames from the command line
  std::string inputFilename = argv[1];
  std::string outputPrefix = argv[2];
  std::string strCreateMesh = argv[3];
  std::string strTx = argv[4];
  std::string strTy = argv[5];
  std::string strTz = argv[6];
  std::string strRx = argv[7];
  std::string strRy = argv[8];
  std::string strRz = argv[9];
  std::string strThetaPoints = argv[10];
  std::string strPhiPoints = argv[11];
  std::string strThetaSpan = argv[12];
  std::string strPhiSpan = argv[13];
  std::string strStoreRays = argv[14];

  std::cout << "InputFilename: " << inputFilename << std::endl;
  std::cout << "OutputPrefix: " << outputPrefix << std::endl;

  // Convert string to bool
  std::stringstream ssCreateMesh(strCreateMesh);
  bool createMesh;
  ssCreateMesh >> createMesh;

  std::cout << "Create Mesh? " << createMesh << std::endl;

  // Convert strings to doubles
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

  std::cout << "Tx: " << tx << std::endl;
  std::cout << "Ty: " << ty << std::endl;
  std::cout << "Tz: " << tz << std::endl;
  std::cout << "Rx: " << rx << std::endl;
  std::cout << "Ry: " << ry << std::endl;
  std::cout << "Rz: " << rz << std::endl;

  unsigned int thetaPoints;
  unsigned int phiPoints;

  std::stringstream ssThetaPoints(strThetaPoints);
  ssThetaPoints >> thetaPoints;

  std::stringstream ssPhiPoints(strPhiPoints);
  ssPhiPoints >> phiPoints;

  std::cout << "Theta points: " << thetaPoints << std::endl;
  std::cout << "Phi points: " << phiPoints << std::endl;

  double thetaSpan;
  double phiSpan;

  std::stringstream ssThetaSpan(strThetaSpan);
  ssThetaSpan >> thetaSpan;
  std::cout << "Theta span: " << thetaSpan << std::endl;

  std::stringstream ssPhiSpan(strPhiSpan);
  ssPhiSpan >> phiSpan;
  std::cout << "Phi span: " << phiSpan << std::endl;

  // Convert string to bool
  std::stringstream ssStoreRays(strStoreRays);
  bool storeRays;
  ssStoreRays >> storeRays;

  std::cout << "Store Rays? " << storeRays << std::endl;

  // Read the input vtp file
  vtkSmartPointer<vtkXMLPolyDataReader> reader = 
    vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();

  // Construct a vtkLidarScanner and set all of its parameters
  vtkSmartPointer<vtkLidarScanner> scanner = 
    vtkSmartPointer<vtkLidarScanner>::New();

  //Scanner->WriteScanner("scanner_original.vtp");

  //double testAngle = vtkMath::Pi()/4.0;
  scanner->SetPhiSpan(phiSpan);
  scanner->SetThetaSpan(thetaSpan);

  scanner->SetNumberOfThetaPoints(thetaPoints);
  scanner->SetNumberOfPhiPoints(phiPoints);

  scanner->SetStoreRays(storeRays);

  // "Aim" the scanner.  This is a very simple translation, but any transformation will work
  vtkSmartPointer<vtkTransform> transform = 
    vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply();

  transform->RotateX(rx);
  transform->RotateY(ry);
  transform->RotateZ(rz);
  transform->Translate(tx, ty, tz);

  scanner->SetTransform(transform);
  scanner->WriteScanner("scanner_transformed.vtp");

  scanner->SetCreateMesh(createMesh);

  scanner->SetInputConnection(reader->GetOutputPort());
  scanner->Update();

  // Create a writer and write the output vtp file
  vtkSmartPointer<vtkXMLImageDataWriter> writer =
    vtkSmartPointer<vtkXMLImageDataWriter>::New();
  writer->SetFileName((outputPrefix + ".vti").c_str());
  writer->SetInputData(scanner->GetOutput());
  writer->Write();

  // Create a writer and write the output vtp file
  if(createMesh) {
      vtkSmartPointer<vtkPolyData> outputMesh = vtkSmartPointer<vtkPolyData>::New();
      scanner->GetOutputMesh(outputMesh);
      vtkSmartPointer<vtkXMLPolyDataWriter> writer =
        vtkSmartPointer<vtkXMLPolyDataWriter>::New();
      writer->SetFileName((outputPrefix + ".vtp").c_str());
      writer->SetInputData(outputMesh);
      writer->Write();
  }

  vtkSmartPointer<vtkPolyData> scannerRepresentation = vtkSmartPointer<vtkPolyData>::New();
  scanner->CreateRepresentation(scannerRepresentation);
  vtkSmartPointer<vtkXMLPolyDataWriter> scannerWriter =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  scannerWriter->SetFileName("scanner.vtp");
  scannerWriter->SetInputData(scannerRepresentation);
  scannerWriter->Write();

  return EXIT_SUCCESS;
}
