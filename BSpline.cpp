#include "BSpline.h"
template <class Scalar>
BSpline3D<Scalar>::BSpline3D(int order, int numControlPoints)
{
	if (numControlPoints <= order)
	{
		numControlPoints = order + 1;
	}

	for (size_t i = 0; i < numControlPoints; i++)
	{
		controlPoints.push_back(Vector2d(i,i));
	}

	for (size_t i = 0; i < numControlPoints + order; i++)
	{
		if (i < order)
		{
			knots.push_back(0);
			continue;
		}

		if (i >= numControlPoints)
		{
			knots.push_back(1);
			continue;
		}

		Scalar leftOver = numControlPoints - order + 1;
		Scalar offset = i - order + 1;

		knots.push_back(offset / leftOver);
	}
}

template <class Scalar>
int BSpline3D<Scalar>::KnotSpan(Scalar t) const
{
	int numControlPoints = controlPoints.size();
	for (size_t i = _order - 1; i <= numControlPoints; i++)
	{
		if (knots[i + 1] >= t)
		{
			return i;
		}
	}
	return numControlPoints;
}

template <class Scalar>
Scalar BSpline3D<Scalar>::BasisFunction(Scalar u, int controlPoint, int degree)
{
	vector<float> values(degree+1);
	for (size_t i = 0; i <= degree; i++)
	{
		values.push_back((Scalar)((knots[controlPoint + i] <= u && u <= knots[controlPoint + 1 + i])));
	}

	for (size_t i = 0; i < degree; i++)
	{
		int currentDegree = i + 1;

		int currentNumControlPoints = degree - i;
		for (size_t j = 0; j < currentNumControlPoints; j++)
		{
			int currentControlPoint = controlPoint + j;

			Scalar fu = u - knots[currentControlPoint];
			Scalar fl = (knots[currentControlPoint + currentDegree] - knots[currentControlPoint]) + (Scalar)(knots[currentControlPoint + currentDegree] == knots[currentControlPoint]);
			Scalar fff = fu / fl;

			Scalar gu = (knots[currentControlPoint + currentDegree + 1] - u);
			Scalar gl = (knots[currentControlPoint + currentDegree + 1] - knots[currentControlPoint + 1]) + (Scalar)(knots[currentControlPoint + currentDegree + 1] == knots[currentControlPoint + 1]);
			Scalar ggg = gu / gl;

			values[j] = fff*values[j] + ggg*values[j + 1];
		}

	}

	return values[0];
}

template <class Scalar>
Vector3d BSpline3D<Scalar>::Sample(Scalar t) const
{
	Vector3d result(0, 0, 0);
	for (size_t i = 0; i < controlPoints.size(); i++)
	{
		result += BasisFunction(t, i, _order - 1)*controlPoints[i];
	}

	return result;
}