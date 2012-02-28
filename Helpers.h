#ifndef Helpers_h
#define Helpers_h

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>

namespace Helpers
{
void WritePolyData(vtkPolyData* const polydata, const std::string& fileName);
}

#endif
