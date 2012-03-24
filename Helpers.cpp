#include "Helpers.h"

namespace Helpers
{
  void WritePolyData(vtkPolyData* const polydata, const std::string& fileName)
  {
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(fileName.c_str());
  writer->SetInputData(polydata);
  writer->Write();
  }
}
