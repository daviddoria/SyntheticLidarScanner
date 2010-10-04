#ifndef __vtkLidarScanner_h
#define __vtkLidarScanner_h

/*
This Lidar scanner class models the output of the Leica HDS 3000. It acquires data in a series of vertical strips, from low to high, left to right. The output is either a pointcloud (with CreateMesh == 0) or a connected mesh (with CreateMesh == 1).

Coordinate System:

The scanner coordinate system is as follows:
z = up
y = forward
therefore (to be right handed), x = right

Theta:
The angle in the "XY" (forward-right) plane (a rotation around Z), measured from +y (Forward). It's range is -pi to pi. -pi/2 is left, pi/2 is right. This is obtained by rotating around the "up" axis.

Phi:
The elevation angle, in the YZ (forward-up) plane (a rotation around X), measured from +y. It's range is -pi/2 (down) to pi/2 (up). This is obtained by rotating around the "right" axis (AFTER the new right axis is obtained by setting Theta).
*/

#include "vtkPolyDataAlgorithm.h" //superclass
#include "vtkSmartPointer.h"

class vtkPolyData;
class vtkTransform;
class vtkInformation;
class vtkInformationVector;
class vtkModifiedBSPTree;

template <typename T> class vtkDenseArray;

class vtkRay;
class vtkLidarPoint;

class vtkLidarScanner : public vtkPolyDataAlgorithm
{
public:
  static vtkLidarScanner *New();
  vtkTypeMacro(vtkLidarScanner,vtkObject);
  void PrintSelf(ostream &os, vtkIndent indent);

  vtkGetMacro(NumberOfThetaPoints, unsigned int);
  vtkGetMacro(NumberOfPhiPoints, unsigned int);
  vtkGetMacro(MinPhiAngle, double);
  vtkGetMacro(MaxPhiAngle, double);
  vtkGetMacro(MinThetaAngle, double);
  vtkGetMacro(MaxThetaAngle, double);
  vtkGetMacro(LOSVariance, double);
  vtkGetMacro(OrthogonalVariance, double);

  ///////// Calculated Accessors /////////////////
  double GetPhiStep() const;
  double GetThetaStep() const;
  double GetNumberOfTotalPoints() const;

  vtkSetMacro(NumberOfThetaPoints, unsigned int);
  vtkSetMacro(NumberOfPhiPoints, unsigned int);

  void SetMinPhiAngle(const double phi);
  void SetMaxPhiAngle(const double phi);
  void SetMinThetaAngle(const double theta);
  void SetMaxThetaAngle(const double theta);

  vtkSetMacro(CreateMesh, bool);
  vtkSetMacro(MaxMeshEdgeLength, double);
  vtkSetMacro(StoreRays, bool);
  vtkSetMacro(LOSVariance, double);
  vtkSetMacro(OrthogonalVariance, double);

  vtkSetMacro(RepresentationLength, double);
  vtkGetMacro(RepresentationLength, double);

  void SetScene(vtkSmartPointer<vtkPolyData> scene);
  void SetTransform(vtkSmartPointer<vtkTransform> transform);

  void SetThetaSpan(const double theta); // (radians)
  void SetPhiSpan(const double phi);  // (radians)

  ///////////// Accessors ////////////
  double* GetPosition() const;
  double* GetLocation() const; //convenience function
  vtkRay* GetRay(const double theta, const double phi) const;

  //////////////Functions///////////
  bool AcquirePoint(const unsigned int thetaIndex, const unsigned int phiIndex); //do a single ray/scene intersection

  void PerformScan(); //actually do all of the ray/scene intersections

  void MakeSphericalGrid(); //use a uniform spherical spacing

  void GetValidOutputPoints(vtkPolyData* output); //put all of the valid scene intersections into a PolyData
  void GetAllOutputPoints(vtkPolyData* output); //put all returns (including misses) into a PolyData
  void GetOutputMesh(vtkPolyData* output); //put all of the valid scene intersections into a PolyData and connect them using Delaunay triangulation

  void AddNoise(vtkSmartPointer<vtkLidarPoint> point);

  void WriteScanner(const std::string &filename) const; //write a vtp file of a coordinate system indicating the scanner's location and orientation

  void CreateRepresentation(vtkPolyData*);

protected:
  vtkLidarScanner();
  ~vtkLidarScanner();
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *); //the function that makes this class work with the vtk pipeline

private:
  static const double Forward[3]; //the direction of the "default" scanner
  static double Origin[3];

  unsigned int NumberOfThetaPoints; //the number of strips
  unsigned int NumberOfPhiPoints; //the number of points per strip
  double MinPhiAngle; //phi angle of the first point in each strip (radians)
  double MaxPhiAngle; //phi angle of the last point in each strip (radians)
  double MinThetaAngle; //theta angle of the first strip (radians)
  double MaxThetaAngle; //theta angle of the last strip (radians)

  vtkstd::vector<double> PhiAngles; // a list of the phi angles
  vtkstd::vector<double> ThetaAngles;// a list of the theta angles

  vtkSmartPointer<vtkTransform> Transform; //the transformation to take the scanner from its default orientation and position to the correction orientation and position

  bool StoreRays;
  bool CreateMesh;
  double MaxMeshEdgeLength;

  //vtkstd::vector<vtkstd::vector<vtkLidarPoint*> > OutputGrid; //outer vector is size NumberOfThetaPoints, inner vector is size NumberOfPhiPoints
  //vtkDenseArray<vtkLidarPoint*>* OutputGrid; //outer vector is size NumberOfThetaPoints, inner vector is size NumberOfPhiPoints
  //vtkDenseArray<vtkSmartPointer<vtkLidarPoint> >* OutputGrid; //outer vector is size NumberOfThetaPoints, inner vector is size NumberOfPhiPoints
  vtkSmartPointer<vtkDenseArray<vtkSmartPointer<vtkLidarPoint> > > OutputGrid; //outer vector is size NumberOfThetaPoints, inner vector is size NumberOfPhiPoints

  double LOSVariance;
  double OrthogonalVariance;

  double RepresentationLength;

private:
  vtkSmartPointer<vtkPolyData> Scene; //the mesh that is to be intersected

  typedef vtkModifiedBSPTree TreeType;
  //TreeType* Tree; //an efficient storage of the scene
  vtkSmartPointer<TreeType> Tree; //an efficient storage of the scene

};

void Project(const double* a, const double* b, double* projection);
void GetOrthogonalVector(const double* v, double* orthogonalVector);

namespace
{
  int sign(double v);
}

#endif
