#pragma once

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <vector>

using namespace std;
using namespace Eigen;

template <class T>
class BSpline3D
{
public:
	BSpline3D(int order, int numControlPoints);
	Vector3d Sample(T t) const;
protected:
	int KnotSpan(T t) const;
	T BasisFunction(T u, int controlPoint, int degree);
private:
	vector<Vector2d> controlPoints;
	vector<T> knots;
	int _order;

};


