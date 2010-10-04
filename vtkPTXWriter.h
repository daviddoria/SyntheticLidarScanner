// .NAME vtkPTXWriter - write a Leica PTX file.
// .SECTION Description

#ifndef __vtkPTXWriter_h
#define __vtkPTXWriter_h

#include "vtkPolyDataAlgorithm.h"

class vtkPTXWriter : public vtkPolyDataAlgorithm
{
public:
  static vtkPTXWriter *New();
  vtkTypeMacro(vtkPTXWriter,vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Specify the name of the file to write out.
  vtkSetStringMacro(FileName);
  vtkGetStringMacro(FileName);

  vtkSetMacro(NumberOfThetaPoints, unsigned int);
  vtkGetMacro(NumberOfThetaPoints, unsigned int);

  vtkSetMacro(NumberOfPhiPoints, unsigned int);
  vtkGetMacro(NumberOfPhiPoints, unsigned int);

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

protected:
  vtkPTXWriter();
  ~vtkPTXWriter();

private:
  vtkPTXWriter(const vtkPTXWriter&);  // Not implemented.
  void operator=(const vtkPTXWriter&);  // Not implemented.

  char *FileName;
  unsigned int NumberOfThetaPoints;
  unsigned int NumberOfPhiPoints;

};

#endif
