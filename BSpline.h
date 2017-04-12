#pragma once

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <vector>

using namespace std;
using namespace Eigen;

template <class Scalar>
class BSpline3D
{
public:
	BSpline3D(int order, int numControlPoints);
	Vector3d Sample(Scalar t) const;
protected:
	int KnotSpan(Scalar t) const;
	Scalar BasisFunction(Scalar u, int controlPoint, int degree);
private:
	vector<Vector2d> controlPoints;
	vector<Scalar> knots;
	int _order;

};


