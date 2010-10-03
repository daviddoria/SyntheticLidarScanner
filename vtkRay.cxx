#include "vtkRay.h"

#include "vtkObjectFactory.h" //for new() macro
#include "vtkMath.h"
#include "vtkTransform.h"
#include "vtkSmartPointer.h"

vtkCxxRevisionMacro(vtkRay, "$Revision: 1.1 $");
vtkStandardNewMacro(vtkRay);

vtkRay::vtkRay()
{
	
}

vtkRay::~vtkRay() 
{
}


void vtkRay::PrintSelf(ostream &os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
  
  //Print the rays origin and direction when << is called
  os << "Origin: " << this->Origin[0] << " " << this->Origin[1] << " " << this->Origin[2] << vtkstd::endl
      << "Direction: " << this->Direction[0] << " " << this->Direction[1] << " " << this->Direction[2] << vtkstd::endl;
	
}

double* vtkRay::GetPointAlong(const double dist)
{
	//return a point 'dist' units along the ray in the "forward" direction
	double* NewPoint = new double[3];
  NewPoint[0] = this->Origin[0] + this->Direction[0] * dist;
  NewPoint[1] = this->Origin[1] + this->Direction[1] * dist;
  NewPoint[2] = this->Origin[2] + this->Direction[2] * dist;
	return NewPoint;
}

void vtkRay::SetDirection(double* Dir)
{
	//set the rays direction to the unit length Dir
	vtkMath::Normalize(Dir);
  this->Direction[0] = Dir[0];
  this->Direction[1] = Dir[1];
  this->Direction[2] = Dir[2];
}

bool vtkRay::IsInfront(double* P)
{
	//create a vector (OtherRay) from the rays origin to the query point P
  vtkSmartPointer<vtkRay> OtherRay = vtkSmartPointer<vtkRay>::New();
	OtherRay->SetOrigin(this->Origin);
	double dir[3] = {P[0] - this->Origin[0], P[1] - this->Origin[1], P[2] - this->Origin[2]};
	OtherRay->SetDirection(dir);
		
	//if the dot product between the above computed direction and the rays direction is greater than
	//zero, the query point is "in front of" the ray
	double dotprod = vtkMath::Dot(this->Direction, OtherRay->GetDirection());
	
	if(dotprod > 0.0)
    {
		return true;
    }
	else
    {
		return false;
    }
  
}

void vtkRay::ApplyTransform(vtkTransform* Trans)
{
	//transform the rays origin and a point 1 unit along the ray (p)
	//store the direction from the transformed origin and p as the rays new direction
	
	double* p = this->GetPointAlong(1.0);
	Trans->TransformPoint(p, p);
	Trans->TransformPoint(this->Origin, this->Origin);
	
	this->Direction[0] = p[0] - this->Origin[0];
	this->Direction[1] = p[1] - this->Origin[1];
	this->Direction[2] = p[2] - this->Origin[2];

}
