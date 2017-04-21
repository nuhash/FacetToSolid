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
#include <math.h>
#include "FeatureExtraction.h"

using namespace Eigen;
using namespace std;
using namespace Utilities;
using namespace FeatureExtractionAlgo;

namespace FeatureCategorisation {


	bool PlanarCheck(const vector<TopoDS_Vertex> vertices, const vector<TopoDS_Face> faces, double tolerance=0.1)
	{
		if (vertices.size() == 3)
		{
			return true;
		}

		MatrixXd A(vertices.size(), 3);
		Vector3d sum(0, 0, 0);
		for (size_t i = 0; i < vertices.size(); i++)
		{
			A.row(i) = converter(vertices[i]);
			sum += converter(vertices[i]);
		}
		sum /= vertices.size();
		A = A.rowwise() - sum.transpose();
		BDCSVD<MatrixXd> svd(A, ComputeThinU| ComputeThinV);
		Vector3d normal = svd.matrixV().rightCols<1>();

		if (!faces[0].IsNull())
		{
			auto faceNormal = converter(faces[0]);
			double dotProd = faceNormal.dot(normal);
			normal *= _dsign(dotProd);
		}
		normal.normalize();

		auto dist2 = A*normal;

		auto maxDist2 = dist2.maxCoeff();

		return maxDist2 > tolerance;
		/*return true;*/
	}

	bool SphericalCheck(const vector<TopoDS_Vertex> vertices, double allowableError = 0.1 /*= 0.1*/)
	{
		//if (faces.size() == 1)
		//	return false;

		MatrixXd A(vertices.size(), 4);
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
			if (err > maxError)
			{
				maxError = err;
			}
			if (maxError > allowableError)
			{
				return false;
			}
		}
		return true;
	}

	bool TubularCheck(const vector<TopoDS_Vertex> vertices, EdgeGroups edgeGroups, int numSections = 5, double allowableError = 0.1 /*= 0.1*/)
	{
		if (edgeGroups.size() != 2)
		{
			return false;
		}

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

		std::sort(values.begin(), values.end(), [](pair<double, double> &left, pair<double, double> &right) {
			return left.first < right.first;
		});

		return true;
	}

	FeatureCategorisation::FeatureCategorisationType CategoriseFeatures(FeatureExtractionAlgo::ExtractedFeature feature, double creaseAngle /*= 5*/)
	{
		auto faces = feature.GetFaces();
		auto vertices = feature.GetVertices();
		auto numEdgeGroups = feature.NumEdgeGroups();
		auto edgeGroups = feature.GetEdgeGroups();
		FeatureCategorisationType result;
		if (PlanarCheck(vertices, faces, creaseAngle))
		{
			result = PLANAR;
		}
		else
			if (SphericalCheck(vertices))
			{
				return SPHERICAL;
			}
			else
				if (TubularCheck(vertices, edgeGroups))
				{
					return TUBULAR;
				}

		//feature.Type(result);

		return result;
	}

	bool LinearCheck(MatrixXd V, double tolerance = 0.1)
	{
		auto centroid = V.colwise().mean();
		V.rowwise() -= centroid;
		BDCSVD<MatrixXd> svd(V, ComputeThinU | ComputeThinV);
		Vector3d normal = svd.matrixV().leftCols<1>();
		normal.normalize();
		auto dist2 = V*normal;

		auto maxDist2 = dist2.maxCoeff();

		return maxDist2 > tolerance;
	}

	bool CircleCheck(vector<TopoDS_Vertex> vertices, double tolerance = 0.1)
	{
		TopoDS_Face face;
		face.Nullify();
		return SphericalCheck(vertices) && PlanarCheck(vertices, { face });
	}

	void CategoriseEdges(const ExtractedFeatures &features, EdgeCategoryMap &edgeCategoryMap)
	{
		for (auto f:features)
		{
			auto edges = f.GetEdges();
			for (auto e:edges)
			{
				auto vertices = e.EdgeVertices();
				MatrixXd v(vertices.size(),3);
				for (size_t i = 0; i < vertices.size(); i++)
				{
					v.row(i) = converter(vertices[i]);
				}

				//v.rowwise() -= v.colwise().mean();

				EdgeCategorisationType result;

				if (LinearCheck(v))
				{
					result = LINEAR;
				}
				else
					if (CircleCheck(vertices))
					{
						result = CIRCULAR;
					}
					else
						result = FREE;

				edgeCategoryMap.insert({ e,result });
			}
		}
	}
}