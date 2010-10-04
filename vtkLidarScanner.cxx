#include <limits>

#include "vtkRay.h"
#include "vtkLidarPoint.h"
#include "vtkLidarScanner.h"

#include "vtkSmartPointer.h"
#include "vtkImageData.h"
#include "vtkConeSource.h"
#include "vtkObjectFactory.h"
#include "vtkIdList.h"
#include "vtkPoints.h"
#include "vtkCellArray.h"
#include "vtkDoubleArray.h"
#include "vtkFloatArray.h"
#include "vtkUnsignedCharArray.h"
#include "vtkPointData.h"
#include "vtkTriangle.h"
#include "vtkLine.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkXMLPolyDataWriter.h"
#include "vtkPolyData.h"
#include "vtkCellData.h"
#include "vtkDelaunay2D.h"
#include "vtkModifiedBSPTree.h"
#include "vtkMath.h"
#include "vtkDenseArray.h"
#include "vtkVertexGlyphFilter.h"

vtkStandardNewMacro(vtkLidarScanner);

// The scanner transform is relative to the coordinate system described at the top of the header file. We need only the Forward vector.
const double vtkLidarScanner::Forward[3] = {0.0, 1.0, 0.0};
double vtkLidarScanner::Origin[3] = {0.0, 0.0, 0.0};

double* vtkLidarScanner::GetLocation() const {return this->GetPosition();} //convenience function

double vtkLidarScanner::GetPhiStep() const {return fabs(MaxPhiAngle - MinPhiAngle)/static_cast<double> (NumberOfPhiPoints - 1);}
double vtkLidarScanner::GetThetaStep() const {return fabs(MaxThetaAngle - MinThetaAngle)/static_cast<double>(NumberOfThetaPoints - 1);}
double vtkLidarScanner::GetNumberOfTotalPoints() const {return NumberOfPhiPoints * NumberOfThetaPoints;}

vtkLidarScanner::vtkLidarScanner()
{
  // Initialze everything to NULL/zero values
  this->Transform = vtkSmartPointer<vtkTransform>::New(); //vtkTransform is initialized to identity by default

  this->Output = vtkSmartPointer<vtkImageData>::New();

  this->NumberOfThetaPoints = 0;
  this->NumberOfPhiPoints = 0;
  this->MinThetaAngle = 0.0;
  this->MaxThetaAngle = 0.0;
  this->MinPhiAngle = 0.0;
  this->MaxPhiAngle = 0.0;

  // Noiseless unless specified
  this->LOSVariance = 0.0;
  this->OrthogonalVariance = 0.0;

  this->CreateMesh = false;
  this->StoreRays = false;

  this->RepresentationLength = 1.0;

  // Don't throw away any edges unless this is specified
  this->MaxMeshEdgeLength = std::numeric_limits<double>::infinity();

  this->Scene = vtkSmartPointer<vtkPolyData>::New();

  //this->RayGrid = vtkSmartPointer<vtkDenseArray<vtkRay*> >::New();
  this->Scan = vtkSmartPointer<vtkDenseArray<vtkLidarPoint*> >::New();


}

int vtkLidarScanner::FillInputPortInformation( int port, vtkInformation* info )
{
  if(port == 0)
    {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData" );
    info->Set(vtkAlgorithm::INPUT_IS_REPEATABLE(), 1);
    return 1;
    }

  vtkErrorMacro("This filter does not have more than 1 input port!");
  return 0;
}
 

vtkLidarScanner::~vtkLidarScanner()
{
 /*
  this->OutputGrid->Delete();

  if(this->Transform)
    {
    this->Transform->Delete();
    }

  if(this->Scene)
    {
    this->Scene->Delete();
    }

  if(this->Tree)
    {
    this->Tree->Delete();
    }
  */
}

void vtkLidarScanner::SetThetaSpan(const double theta) // (radians)
{
  // This is a convenience function that simply divides the span by two and evenly splits the span across zero
  if(fabs(theta) > 2.0*vtkMath::Pi())
    {
    vtkstd::cout << "Error: theta must be in [-2 pi, 2 pi]" << std::endl;
    }
  else
    {
    this->MinThetaAngle = -fabs(theta)/2.0;
    this->MaxThetaAngle = fabs(theta)/2.0;
    this->Modified();
    }
}

void vtkLidarScanner::SetPhiSpan(const double phi)  // (radians)
{
  // This is a convenience function that simply divides the span by two and evenly splits the span across zero
  if(fabs(phi) > vtkMath::Pi())
    {
    std::cout << "Error: phi must be in [-pi, pi]" << std::endl;
    }
  else
    {
    this->MinPhiAngle = -fabs(phi)/2.0;
    this->MaxPhiAngle = fabs(phi)/2.0;
    this->Modified();
    }
}


double* vtkLidarScanner::GetPosition() const
{
  // Return the 3D coordinate of the scanners location. If the scanner transform has not been set, this value is not valid.
  if(Transform)
    {
    return this->Transform->GetPosition();
    }
  else
    {
    return NULL;
    }
}


void vtkLidarScanner::SetScene(vtkSmartPointer<vtkPolyData> scene)
{
  // Set the polydata that we are going to scan
  //this->Scene = scene;

  std::cout << "input to SetScene has " << scene->GetNumberOfPoints() << " points." << std::endl;

  this->Scene->ShallowCopy(scene);

  this->Tree = vtkSmartPointer<TreeType>::New();
  this->Tree->SetDataSet(this->Scene);
  this->Tree->BuildLocator();
}

void vtkLidarScanner::SetMinPhiAngle(const double phi)
{
  if(fabs(phi) > vtkMath::Pi())
    {
    std::cout << "Error: phi must be in [-pi,pi]" << std::endl;
    }
  else
    {
    this->MinPhiAngle = phi;
    this->Modified();
    }
}

void vtkLidarScanner::SetMaxPhiAngle(const double phi)
{
  if(fabs(phi) > vtkMath::Pi())
    {
    std::cout << "Error: phi must be in [-pi,pi]" << std::endl;
    }
  else
    {
    this->MaxPhiAngle = phi;
    this->Modified();
    }
}

void vtkLidarScanner::SetMinThetaAngle(const double theta)
{
  if(fabs(theta) > 2.0*vtkMath::Pi())
    {
    std::cout << "Error: theta must be in [-2 pi, 2 pi]" << std::endl;
    }
  else
    {
    this->MinThetaAngle = theta;
    this->Modified();
    }
}

void vtkLidarScanner::SetMaxThetaAngle(const double theta)
{
  if(fabs(theta) > 2.0*vtkMath::Pi())
    {
    std::cout << "Error: theta must be in [-2 pi, 2 pi]" << std::endl;
    }
  else
    {
    this->MaxThetaAngle = theta;
    this->Modified();
    }
}


//vtkCxxSetObjectMacro(vtkLidarScanner, Transform, vtkTransform);
void vtkLidarScanner::SetTransform(vtkSmartPointer<vtkTransform> transform)
{
  //this->Transform->ShallowCopy(transform);
  this->Transform->DeepCopy(transform);
}

int vtkLidarScanner::RequestData(vtkInformation *vtkNotUsed(request),
		vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // This function calls the scanners input and output to allow it to
  // work in the vtk algorithm pipeline

  // Get the info objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);

  // Get the input and ouptut
  vtkPolyData *input = vtkPolyData::SafeDownCast(
          inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkImageData *output = vtkImageData::SafeDownCast(
          outInfo->Get(vtkDataObject::DATA_OBJECT()));

  std::cout << "input to vtkLidarScanner has " << input->GetNumberOfPoints() << " points." << std::endl;

  this->SetScene(input);

  this->PerformScan();

  ConstructOutput();

  /*
  if(this->CreateMesh)
    {
    this->GetOutputMesh(output);
    }
  else
    {
    this->GetValidOutputPoints(output);
    }
  */

  return 1;
}

void vtkLidarScanner::ConstructOutput()
{
  vtkSmartPointer<vtkDoubleArray> coordinateArray =
    vtkSmartPointer<vtkDoubleArray>::New();
  coordinateArray->SetNumberOfComponents(3);
  coordinateArray->SetNumberOfTuples(this->NumberOfPhiPoints * this->NumberOfThetaPoints);
  
  vtkSmartPointer<vtkDoubleArray> normalArray =
    vtkSmartPointer<vtkDoubleArray>::New();
  normalArray->SetNumberOfComponents(3);
  normalArray->SetNumberOfTuples(this->NumberOfPhiPoints * this->NumberOfThetaPoints);
  
  vtkSmartPointer<vtkIntArray> validArray =
    vtkSmartPointer<vtkIntArray>::New();
  validArray->SetNumberOfComponents(1);
  validArray->SetNumberOfTuples(this->NumberOfPhiPoints * this->NumberOfThetaPoints);

  for(unsigned int phi = 0; phi < this->NumberOfPhiPoints; phi++)
    {
    for(unsigned int theta = 0; theta < this->NumberOfThetaPoints; theta++)
      {
      int ijk[3] = {0,0,0};
      ijk[0] = phi;
      ijk[1] = theta;
      vtkIdType index = this->Output->ComputePointId(ijk);

      double coordinate[3];
      this->Scan->GetValue(phi, theta)->GetCoordinate(coordinate);
      coordinateArray->SetTupleValue(index, coordinate);

      double normal[3];
      this->Scan->GetValue(phi, theta)->GetCoordinate(normal);
      normalArray->SetTupleValue(index, normal);

      validArray->SetValue(index, this->Scan->GetValue(phi, theta)->GetHit());
      }
    }
  
}

void vtkLidarScanner::AcquirePoint(const unsigned int thetaIndex, const unsigned int phiIndex)
{
  // This function performs a ray/mesh intersection of the ray at the specified grid location

  // The grid location must be in bounds.
  if(thetaIndex >= this->NumberOfThetaPoints || phiIndex >= this->NumberOfPhiPoints)
    {
    std::cout << "Out of range!" << std::endl;
    exit(-1);
    }

  // We have computed the grid of rays (in MakeSphericalGrid()) relative to a "default" scanner (i.e. Forward = (0,1,0))
  // so we have to apply the scanner's transform to the ray before casting it
  vtkRay* ray = this->RayGrid->GetValue(phiIndex, thetaIndex);
  ray->ApplyTransform(this->Transform);

  double t;
  double x[3];
  double pcoords[3];
  int subId;
  vtkIdType cellId;
  int hit = this->Tree->IntersectWithLine(ray->GetOrigin(), ray->GetPointAlong(1000.0), .01, t, x, pcoords, subId, cellId);

  // The ray does not intersect the mesh at all, we can stop here
  if(!hit)
    {
    return;
    }

  vtkPoints* triPoints = vtkTriangle::SafeDownCast(this->Scene->GetCell(cellId))->GetPoints();

  double n[3];
  double t0[3];
  double t1[3];
  double t2[3];

  triPoints->GetPoint(0, t0);
  triPoints->GetPoint(1, t1);
  triPoints->GetPoint(2, t2);
  vtkTriangle::ComputeNormal(t0, t1, t2, n);

  // Save the intersection
  this->Scan->GetValue(phiIndex,thetaIndex)->SetCoordinate(x);

  // Save the normal of the intersection
  this->Scan->GetValue(phiIndex, thetaIndex)->SetNormal(n);

  // Set the flag for this point indicating that there was a valid intersection
  this->Scan->GetValue(phiIndex, thetaIndex)->SetHit(true);

  this->AddNoise(Scan->GetValue(phiIndex, thetaIndex));

}


void vtkLidarScanner::PerformScan()
{
  std::cout << "Performing scan..." << std::endl;
  // Loop through the grid and intersect each ray with the scene.
  for(unsigned int theta = 0; theta < this->NumberOfThetaPoints; theta++)
    {
    for(unsigned int phi = 0; phi < this->NumberOfPhiPoints; phi++)
      {
      AcquirePoint(theta, phi);
      }
    }

}


void vtkLidarScanner::MakeSphericalGrid()
{
  // Make a uniformly spaced spherical grid assuming a scanner position of (0,0,0) and facing Forward (0, 1, 0).
  // This grid will later be traversed and the ray at each position will be cast into the scene to look for valid intersections.
  // (the rays will be transformed using the scanner Transform so that they come from the scanner in its current orientation)
  // This function also initializes all of the vtkLidarPoint's in the Scan grid
  
  // Size the grid
  this->Scan->Resize(this->NumberOfPhiPoints, this->NumberOfThetaPoints);

  for(unsigned int thetaCounter = 0; thetaCounter < NumberOfThetaPoints; thetaCounter++)
    {
    for(unsigned int phiCounter = 0; phiCounter < NumberOfPhiPoints; phiCounter++)
      {
      // Compute the (phi,theta) coordinate for the current grid location
      double phi = this->MinPhiAngle + phiCounter * this->GetPhiStep();
      double theta = this->MinThetaAngle + thetaCounter * this->GetThetaStep();

      // Convert the spherical coordinates into a cartesian direction
      vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
      // Caution - these VTK functions take parameters in degrees!
      transform->RotateZ(-theta*180./vtkMath::Pi()); //the negative is to obtain the coordinate system we defined
      transform->RotateX(phi*180./vtkMath::Pi());
      double* rayDir = transform->TransformPoint(this->Forward);

      // Construct a ray
      vtkSmartPointer<vtkRay> ray = vtkSmartPointer<vtkRay>::New();

      ray->SetOrigin(this->Origin);
      ray->SetDirection(rayDir);

      vtkSmartPointer<vtkLidarPoint> lidarPoint =
        vtkSmartPointer<vtkLidarPoint>::New();
      lidarPoint->SetRay(ray);
      
      // Store the ray in the grid
      this->Scan->SetValue(phiCounter, thetaCounter, lidarPoint);

      } // end phi loop

    } //end theta loop

}

void vtkLidarScanner::GetOutputMesh(vtkPolyData* output)
{
  // std::cout << "GetOutputMesh" << std::endl;
  // This function connects the raw points output into a mesh using the connectivity
  // information from the known point acquisition order. The result is stored in 'output'.

  // Create a polydata of the points only
  GetValidOutputPoints(output);

  // Create a grid of theta/phi coordinates (keeps only connectivity, not geometry)
  vtkSmartPointer<vtkPoints> points2D = vtkSmartPointer<vtkPoints>::New();

  unsigned int PointCounter = 0;
  //for(unsigned int theta = 0; theta < OutputGrid.size(); theta++ )
  for(unsigned int theta = 0; theta < this->NumberOfThetaPoints; theta++ )
    {
    //for(unsigned int phi = 0; phi < OutputGrid[0].size(); phi++ )
    for(unsigned int phi = 0; phi < this->NumberOfPhiPoints; phi++ )
      {
      if(this->OutputGrid->GetValue(phi,theta)->GetHit())
        {
        points2D->InsertNextPoint(theta, phi, 0);
        PointCounter++;
        }
      } //end phi loop
    }//end theta loop

  std::cout << PointCounter << " points were added." << std::endl;

  // Add the 2d grid points to a polydata object
  vtkSmartPointer<vtkPolyData> polydata2d = vtkSmartPointer<vtkPolyData>::New();
  polydata2d->SetPoints(points2D);

  // Triangulate the grid points
  vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
  delaunay->SetInput(polydata2d);
  delaunay->Update();

  // Get the resulting triangles from the triangulation
  vtkCellArray* cells = delaunay->GetOutput()->GetPolys();

  // Create the 3d triangle array
  vtkSmartPointer<vtkCellArray> Triangles3D = vtkSmartPointer<vtkCellArray>::New();

  // Initialize some variables
  vtkIdType npts; // the number of points in a cell
  vtkIdType* pts; //indexes to the points

  // Go through all the triangles of the Delaunay triangulation and add them to the 3d polydata if they are shorter than MaxMeshEdgeLength
  cells->InitTraversal();
  while (cells->GetNextCell(npts,pts))
    {
    // Get the 3 points of the current triangle
    double p0[3];
    double p1[3];
    double p2[3];

    //If the rays are stored, point 0 is the scanner position. If the rays are not stored, point 0 is the first scan point
    unsigned int offset;
    if(this->StoreRays)
      {
      offset = 1;
      }
    else
      {
      offset = 0;
      }

    unsigned int TriP0 = pts[0] + offset;
    unsigned int TriP1 = pts[1] + offset;
    unsigned int TriP2 = pts[2] + offset;

    output->GetPoint(TriP0, p0);
    output->GetPoint(TriP1, p1);
    output->GetPoint(TriP2, p2);

    // Throw away triangles that are bigger than a threshold
    double dist1 = vtkMath::Distance2BetweenPoints(p0, p1);
    if(dist1 > this->MaxMeshEdgeLength)
      {
      continue;
      }

    double dist2 = vtkMath::Distance2BetweenPoints(p1, p2);
    if(dist2 > this->MaxMeshEdgeLength)
      {
      continue;
      }

    double dist3 = vtkMath::Distance2BetweenPoints(p0, p2);
    if(dist3 > this->MaxMeshEdgeLength)
      {
      continue;
      }

    // Add the triangle to the 3d polydata
    vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();

    triangle->GetPointIds()->SetId(0,TriP0);
    triangle->GetPointIds()->SetId(1,TriP1);
    triangle->GetPointIds()->SetId(2,TriP2);
    Triangles3D->InsertNextCell(triangle);

    }//end while

  // Save the 3d triangles in the output polydata
  output->SetPolys(Triangles3D);

}

void vtkLidarScanner::GetValidOutputPoints(vtkPolyData* output)
{
  //std::cout << "GetOutputPoints" << std::endl;

  // Declare a geometry and topology array
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> Vertices = vtkSmartPointer<vtkCellArray>::New();

  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

  // Declare an array to store the normals of the triangles that were intersected
  vtkSmartPointer<vtkDoubleArray> norms = vtkSmartPointer<vtkDoubleArray>::New();
  norms->SetNumberOfComponents(3);
  norms->SetName("Normals");

  // Store the scanner position as a point so rays can be drawn between scanner and scan points
  vtkIdType ScannerLocationPid[1];
  if(StoreRays)
    {
    ScannerLocationPid[0] = Points->InsertNextPoint(this->GetLocation());
    double OriginNormal[3] = {0.0, 0.0, 1.0}; //must be a valid normal so tube filter will work (!!! should change this to the scanner's 'up' vector)
    norms->InsertNextTupleValue(OriginNormal); // have to insert a normal for this new point or it will complain "the array is too short"
    }

  // Traverse the grid, storing valid scene intersections in the geometry/topology/normal arrays
  for(unsigned int thetaCounter = 0; thetaCounter < NumberOfThetaPoints; thetaCounter++)
    {
    for(unsigned int phiCounter = 0; phiCounter < NumberOfPhiPoints; phiCounter++)
      {
      // If the ray in this grid location had a valid scene intersection
      if(this->OutputGrid->GetValue(phiCounter, thetaCounter)->GetHit())
        {
        //set the next element in the geometry/topology/normal vector
        double* p = OutputGrid->GetValue(phiCounter, thetaCounter)->GetCoordinate();
        vtkIdType pid[1];
        pid[0] = Points->InsertNextPoint(p);
        Vertices->InsertNextCell(1,pid);

        norms->InsertNextTupleValue(OutputGrid->GetValue(phiCounter, thetaCounter)->GetNormal());

        if(StoreRays)
          {
          vtkSmartPointer<vtkLine> Line = vtkSmartPointer<vtkLine>::New();
          Line->GetPointIds()->SetId(0,ScannerLocationPid[0]);
          Line->GetPointIds()->SetId(1,pid[0]);
          lines->InsertNextCell(Line);
          }
        }//end outputgrid->GetHit if
    else// !outputgrid->GetHit
      {
        // If StoreRays is on, we will save a unit distance point in every direction so the rays can be visualized even if the target is missed
        if(StoreRays)
          {
          // Set the next element in the geometry/topology/normal vector
          vtkRay* R = OutputGrid->GetValue(phiCounter, thetaCounter)->GetRay();
          double* p = R->GetPointAlong(1.0);
          vtkIdType pid[1];
          pid[0] = Points->InsertNextPoint(p);
          Vertices->InsertNextCell(1,pid);

          double n[3] = {1.0, 0.0, 0.0}; //the normal of a miss point is not defined, so we set it to an arbitrary (1,0,0)
          norms->InsertNextTupleValue(n);

          vtkSmartPointer<vtkLine> Line = vtkSmartPointer<vtkLine>::New();
          Line->GetPointIds()->SetId(0,ScannerLocationPid[0]);
          Line->GetPointIds()->SetId(1,pid[0]);
          lines->InsertNextCell(Line);
          } //end if(StoreRays)
      } //end else (!outputgrid->GetHit)
    }//end phi for loop
  } //end theta for loop

  //save these arrays in the polydata
  output->SetPoints(Points);
  output->SetVerts(Vertices);
  if(StoreRays)
    {
    output->SetLines(lines);
    }

  output->GetPointData()->SetNormals(norms);

}


void vtkLidarScanner::GetAllOutputPoints(vtkPolyData* output)
{
  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();

  // An array to store the normals of the triangles that were intersected
  vtkSmartPointer<vtkDoubleArray> normals =
    vtkSmartPointer<vtkDoubleArray>::New();
  normals->SetNumberOfComponents(3);
  normals->SetName("Normals");

  // An array to store whether or not each point is valid
  vtkSmartPointer<vtkIntArray> validArray =
    vtkSmartPointer<vtkIntArray>::New();
  validArray->SetNumberOfComponents(1);
  validArray->SetName("Valid");

  // Traverse the grid
  for(unsigned int theta = 0; theta < this->NumberOfThetaPoints; theta++)
    {
    for(unsigned int phi = 0; phi < this->NumberOfPhiPoints; phi++)
      {
      // If the ray in this grid location had a valid scene intersection

      double* p = OutputGrid->GetValue(phi, theta)->GetCoordinate();
      points->InsertNextPoint(p);

      normals->InsertNextTupleValue(OutputGrid->GetValue(phi, theta)->GetNormal());

      validArray->InsertNextValue(this->OutputGrid->GetValue(phi, theta)->GetHit());
    }//end phi loop
  } //end theta loop

  vtkSmartPointer<vtkPolyData> polydata =
    vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
    vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexFilter->SetInputConnection(polydata->GetProducerPort());
  vertexFilter->Update();

  // Save these arrays in the polydata
  output->ShallowCopy(vertexFilter->GetOutput());
  output->GetPointData()->SetNormals(normals);
  output->GetPointData()->AddArray(validArray);
}

void vtkLidarScanner::WriteScanner(const std::string &filename) const
{
  vtkSmartPointer<vtkPolyData> poly =
    vtkSmartPointer<vtkPolyData>::New();

  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0.0, 0.0, 0.0);
  points->InsertNextPoint(1.0, 0.0, 0.0);
  points->InsertNextPoint(0.0, 1.0, 0.0);
  points->InsertNextPoint(0.0, 0.0, 1.0);

  vtkSmartPointer<vtkCellArray> lines =
    vtkSmartPointer<vtkCellArray>::New();

  vtkSmartPointer<vtkLine> line =
    vtkSmartPointer<vtkLine>::New();
  //x axis
  line->GetPointIds()->SetId(0,0);
  line->GetPointIds()->SetId(1,1);
  lines->InsertNextCell(line);

  //y axis
  line->GetPointIds()->SetId(0,0);
  line->GetPointIds()->SetId(1,2);
  lines->InsertNextCell(line);

  //z axis
  line->GetPointIds()->SetId(0,0);
  line->GetPointIds()->SetId(1,3);
  lines->InsertNextCell(line);

  //setup colors
  unsigned char red[3] = {255, 0, 0};
  unsigned char yellow[3] = {255, 255, 0};
  unsigned char green[3] = {0, 255, 0};

  vtkSmartPointer<vtkUnsignedCharArray> colors =
    vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");
  colors->InsertNextTupleValue(red);
  colors->InsertNextTupleValue(yellow);
  colors->InsertNextTupleValue(green);

  //add points and lines to polydata
  poly->SetPoints(points);
  poly->SetLines(lines);

  //add colors to lines
  poly->GetCellData()->SetVectors(colors);

  vtkSmartPointer<vtkTransformPolyDataFilter> translateFilter =
    vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  translateFilter->SetInput(poly);
  translateFilter->SetTransform(this->Transform);
  translateFilter->Update();

  vtkPolyData* transformed = translateFilter->GetOutput();

  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(filename.c_str());
  writer->SetInput(transformed);
  writer->Write();

}

void vtkLidarScanner::AddNoise(vtkSmartPointer<vtkLidarPoint> point)
{
  if(!point->GetHit())
    {
    return; // Don't change anything if the point is not valid
    }

  // Get the original point information
  double* originalPoint = point->GetCoordinate();
  vtkRay* originalRay = point->GetRay();

  // Create line of sight noise (a vector to add to the point to affect the distance that was seen)
  double* los = originalRay->GetDirection();
  double losX = los[0];
  double losY = los[1];
  double losZ = los[2];
  double losNoise[3];
  if(this->LOSVariance > 0.0)
    {
    double losLength = vtkMath::Gaussian(0.0, this->LOSVariance); //LOS noise should be zero mean
    std::cout << "LOSLength: " << losLength << std::endl;

    losNoise[0] = losLength * losX;
    losNoise[1] = losLength * losY;
    losNoise[2] = losLength * losZ;
    }
  else
    {
    for(unsigned int i = 0; i < 3; i++)
      {
      losNoise[i] = 0.0;
      }
    }

  // Create orthogonal noise
  double orthogonalNoise[3];
  if(this->OrthogonalVariance > 0.0)
    {
    double originalDirection[3];
    originalDirection[0] = losX;
    originalDirection[1] = losY;
    originalDirection[2] = losZ;
    GetOrthogonalVector(originalDirection, orthogonalNoise);
    double orthogonalLength = vtkMath::Gaussian(0.0, this->OrthogonalVariance); //LOS noise should be zero mean
    std::cout << "OrthogonalLength: " << orthogonalLength << std::endl;
    for(unsigned int i = 0; i < 3; i++)
      {
      orthogonalNoise[i] = orthogonalLength * orthogonalNoise[i];
      }
    }
  else // OrthogonalVariance < 0
    {
    for(unsigned int i = 0; i < 3; i++)
      {
      orthogonalNoise[i] = 0.0;
      }
    }	//end else OrthogonalVariance

  // Combine the noise and add it to the point
  double noiseVector[3];
  for(unsigned int i = 0; i < 3; i++)
    {
    noiseVector[i] = losNoise[i] + orthogonalNoise[i];
    }

  double newPoint[3];
  for(unsigned int i = 0; i < 3; i++)
    {
    newPoint[i] = originalPoint[i] + noiseVector[i];
    }

  // Set the noisy point
  //std::cout << "Adding noise..." << std::endl;
  point->SetCoordinate(newPoint);
}

void GetOrthogonalVector(const double* v, double* orthogonalVector)
{
  //Gram Schmidt Orthogonalization

  // Create a random vector
  double randomVector[3] = {vtkMath::Random(0.0, 1.0), vtkMath::Random(0.0, 1.0), vtkMath::Random(0.0, 1.0)};
  vtkMath::Normalize(randomVector);

  double projectionOfRandOnV[3];
  Project(randomVector, v, projectionOfRandOnV);

  for(unsigned int i = 0; i < 3; i++)
    {
    orthogonalVector[i] = randomVector[i] - projectionOfRandOnV[i];
    }

  vtkMath::Normalize(orthogonalVector);

}

void Project(const double* a, const double* b, double* projection)
{
  // The projection of A on B
  double scale = vtkMath::Dot(a,b)/pow(vtkMath::Norm(b), 2);
  for(unsigned int i = 0; i < 3; i++)
    {
    projection[i] = scale * b[i];
    }

}

void vtkLidarScanner::CreateRepresentation(vtkPolyData* representation)
{
  vtkSmartPointer<vtkConeSource> coneSource =
    vtkSmartPointer<vtkConeSource>::New();
  coneSource->SetResolution(4);
  coneSource->SetHeight(1);
  coneSource->Update();

  vtkSmartPointer<vtkTransform> transform =
    vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply();
  transform->RotateX(45);
  transform->RotateZ(-90);
  transform->Translate(0, 0.5, 0);

  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
    vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetTransform(transform);
  transformFilter->SetInputConnection(coneSource->GetOutputPort());
  transformFilter->Update();

  vtkPoints* points = transformFilter->GetOutput()->GetPoints();

  // Point id: location
  // 0: tip (cone is looking from tip to base down the -x axis
  // 1: (+x, +z)
  double topRight[3];
  points->GetPoint(1, topRight);

  // 2: (-x, +z)
  double topLeft[3];
  points->GetPoint(2, topLeft);

  // 3: (-x, -z)
  double bottomLeft[3];
  points->GetPoint(3, bottomLeft);

  // 4: (+x, -z)
  double bottomRight[3];
  points->GetPoint(4, bottomRight);

  // Set the left/right (theta) angles
  // Moving points topLeft and bottomLeft (left, minThetaAngle) or topRight and bottomRight (right, maxThetaAngle) +/- x does this
  topLeft[0] = this->RepresentationLength * sign(MinThetaAngle) * tan(vtkMath::RadiansFromDegrees(MinThetaAngle));
  bottomLeft[0] = this->RepresentationLength * sign(MinThetaAngle) * tan(vtkMath::RadiansFromDegrees(MinThetaAngle));

  topRight[0] = this->RepresentationLength * sign(MaxThetaAngle) * tan(vtkMath::RadiansFromDegrees(MaxThetaAngle));
  bottomRight[0] = this->RepresentationLength * sign(MaxThetaAngle) * tan(vtkMath::RadiansFromDegrees(MaxThetaAngle));

  // Set the up/down (phi) angles
  // Moving points topLeft and topRight (top, maxPhiAngle) or bottomLeft and bottomRight (bottom, minPhiAngle) +/- z does this
  topLeft[2] = this->RepresentationLength * sign(MaxPhiAngle) * tan(vtkMath::RadiansFromDegrees(MaxPhiAngle));
  topRight[2] = this->RepresentationLength * sign(MaxPhiAngle) * tan(vtkMath::RadiansFromDegrees(MaxPhiAngle));

  bottomLeft[2] = this->RepresentationLength * sign(MinPhiAngle) * tan(vtkMath::RadiansFromDegrees(MinPhiAngle));
  bottomRight[2] = this->RepresentationLength * sign(MinPhiAngle) * tan(vtkMath::RadiansFromDegrees(MinPhiAngle));

  representation->ShallowCopy(transformFilter->GetOutput());

}


////////// External Operators /////////////

void vtkLidarScanner::PrintSelf(ostream &os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  os << "Scanner" << std::endl
		  << "-----------" << std::endl;
  if(Transform)
    {
    std::cout << "Transform: " << *(this->Transform) << std::endl
              << "Location: " << this->GetPosition()[0] << " "
              << this->GetPosition()[1] << " " << this->GetPosition()[2] << std::endl;
    }
  else
    {
    std::cout << "Transform (and hence Location) are NULL" << std::endl;
    }

  std::cout << "NumberOfThetaPoints: " << this->NumberOfThetaPoints << std::endl
      << "NumberOfPhiPoints: " << this->NumberOfPhiPoints << std::endl
      << "MinThetaAngle: " << this->MinThetaAngle << std::endl
      << "MaxThetaAngle: " << this->MaxThetaAngle << std::endl
      << "MinPhiAngle: " << this->MinPhiAngle << std::endl
      << "MaxPhiAngle: " << this->MaxPhiAngle << std::endl << std::endl;
}

namespace
{
  int sign(double v)
  {
  return v > 0 ? 1 : (v < 0 ? -1 : 0);
  }
}