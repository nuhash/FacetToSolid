#include "FeatureCategorisation.h"
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GeomLProp_SLProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <Eigen/Eigenvalues>
#include <tuple>
#include <c:/Program Files (x86)/Microsoft Visual Studio 12.0/VC/INCLUDE/utility>
//#include <ppl.h>

using namespace Eigen;
using namespace std;

FeatureCategorisation::CategorisationType FeatureCategorisation::Categorise(FeatureExtractionAlgo::ExtractedFeature feature, double creaseAngle /*= 5*/)
{
	auto faces = feature.GetFaces();
	auto vertices = feature.GetVertices();
	
	if (PlanarCheck(faces, creaseAngle))
	{
		return PLANAR;
	}

	if (SphericalCheck(vertices))
	{
		return SPHERICAL;
	}

	if (TubularCheck(vertices,0))
	{
		return TUBULAR;
	}

	return COMPLEX;
}

bool FeatureCategorisation::PlanarCheck(const vector<TopoDS_Face> faces, double creaseAngle)
{
	double limit = cos(creaseAngle);
	double sum = 0;
	Vector3d normalAvg(0,0,0);
	
	if (faces.size()==1)
	{
		return true;
	}

	for (size_t i = 0; i < faces.size(); i++)
	{
		auto currentFace = faces[i];

		Standard_Real umin, umax, vmin, vmax;
		BRepTools::UVBounds(currentFace, umin, umax, vmin, vmax);

		auto currentSurface = BRep_Tool::Surface(currentFace);

		GeomLProp_SLProps props(currentSurface, umin, vmin, 1, 0.01);
		gp_Dir normal = props.Normal();
		Vector3d currentNormal(normal.X(), normal.Y(), normal.Z());

		if (i != 0)
		{
			auto dotProduct = normalAvg.dot(currentNormal);
			if (dotProduct < limit)
				return false;
		}
			

		normalAvg = (i*normalAvg) / (i + 1) + currentNormal / (i + 1);
	}
	return true;
}

bool FeatureCategorisation::SphericalCheck(const vector<TopoDS_Vertex> vertices, double allowableError /*= 0.1*/)
{
	//if (faces.size() == 1)
	//	return false;

	MatrixXd A(vertices.size(),4);
	VectorXd b(vertices.size());

	for (size_t i = 0; i < vertices.size(); i++)
	{
		auto pnt = BRep_Tool::Pnt(vertices[i]);
		b(i) = pnt.X()*pnt.X() + pnt.Y()*pnt.Y() + pnt.Z()*pnt.Z();
		A(i, 0) = 2 * pnt.X();
		A(i, 1) = 2 * pnt.Y();
		A(i, 2) = 2 * pnt.Z();
		A(i, 3) = 1;
	}

	auto x = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
	Vector3d centre(x(0), x(1), x(2));
	double radius = sqrt(x(3) + x(0)*x(0) + x(1)*x(1) + x(2)*x(2));

	double maxError = 0;
	for (size_t i = 0; i < vertices.size(); i++)
	{
		auto pnt = BRep_Tool::Pnt(vertices[i]);
		Vector3d currentVertex(pnt.X(), pnt.Y(), pnt.Z());
		auto l = currentVertex - centre;
		auto err = abs(l.norm() - radius);
		if (err>maxError)
		{
			maxError = err;
		}
		if (maxError>allowableError)
		{
			return false;
		}
	}
	return true;
}

bool FeatureCategorisation::TubularCheck(const vector<TopoDS_Vertex> vertices, int numEdgeGroups, double allowableError /*= 0.1*/)
{
	//Edge groups are not currently available
	//if (numEdgeGroups != 2) \\&& edgeType == CONTINUOUS
	//{
	//	return false;
	//}

	MatrixXd v(vertices.size(), 3);
	for (size_t i = 0; i < vertices.size(); i++)
	{
		auto pnt = BRep_Tool::Pnt(vertices[i]);
		v(i, 0) = pnt.X();
		v(i, 1) = pnt.Y();
		v(i, 2) = pnt.Z();
	}

	Vector3d origin = v.colwise().mean();
	auto centered = v.rowwise() - origin.transpose();
	auto covariance = centered.adjoint() * centered;
	SelfAdjointEigenSolver<MatrixXd> eig(covariance);
	Vector3d axis = eig.eigenvectors().col(2).normalized();

	VectorXd p = v*axis;

	VectorXd mu = p.array() - origin.dot(axis);

	MatrixXd d = (v - mu*axis.transpose()).rowwise() - origin.transpose();

	VectorXd l = d.rowwise().norm();

	vector<pair<double, double>> values;
	for (size_t i = 0; i < mu.size(); i++)
	{
		values.push_back(pair<double, double>(mu(i), l(i)));
	}

	std::sort(values.begin(), values.end(), [](pair<double,double> &left, pair<double, double> &right) {
		return left.first < right.first;
	});



	return true;
}
