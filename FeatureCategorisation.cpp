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
#include <utility>
#include <math.h>
#include "FeatureExtraction.h"
#include <memory>

using namespace Eigen;
using namespace std;
using namespace Utilities;
using namespace FeatureExtractionAlgo;

namespace FeatureCategorisation {


	bool PlanarCheck(const vector<TopoDS_Vertex> vertices, const vector<TopoDS_Face> faces, shared_ptr<SurfaceCategorisationData> &data, double tolerance=0.1)
	{
		if (vertices.size() == 3)
		{
			return true;
		}

		MatrixXd A(vertices.size(), 3);
		for (size_t i = 0; i < vertices.size(); i++)
		{
			A.row(i) = converter(vertices[i]);

		}
		Vector3d centroid = A.colwise().mean();
		A.rowwise() -= centroid.transpose();
		BDCSVD<MatrixXd> svd(A, ComputeThinU| ComputeThinV);
		Vector3d normal = svd.matrixV().rightCols<1>();

		if (!faces[0].IsNull())
		{
			auto faceNormal = converter(faces[0]);
			double dotProd = faceNormal.dot(normal);
			normal *= (dotProd > 0) - (dotProd < 0);
		}
		//normal.normalize();

		auto dist2 = A*normal;

		auto maxDist2 = dist2.maxCoeff();
		auto temp = make_shared<PlanarSurfaceData>();
		if (maxDist2 < tolerance)
		{
			temp->normal = Vector3d(normal.x(),normal.y(),normal.z());
			temp->type = PLANAR;
			temp->centroid = centroid;
			temp->normal2 = gp_Dir(normal.x(), normal.y(), normal.z());
			temp->normal3 = Vector3d(temp->normal);

			data = temp;
			return true;
		}
		return false;
		/*return true;*/
	}

	bool SphericalCheck(const vector<TopoDS_Vertex> vertices, shared_ptr<SurfaceCategorisationData> &data, double allowableError = 0.1 /*= 0.1*/)
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

		auto temp = make_shared<SphericalSurfaceData>();
		temp->position = centre;
		temp->radius = radius;
		temp->type = SPHERICAL;
		data = temp;
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

		return false;
	}

	FeatureCategorisation::FeatureCategorisationType CategoriseFeature(FeatureExtractionAlgo::ExtractedFeature feature, shared_ptr<SurfaceCategorisationData> &data, double creaseAngle /*= 5*/)
	{
		auto faces = feature.GetFaces();
		auto vertices = feature.GetVertices();
		auto numEdgeGroups = feature.NumEdgeGroups();
		auto edgeGroups = feature.GetEdgeGroups();
		FeatureCategorisationType result;
		//shared_ptr<SurfaceCategorisationData> data = nullptr;
		if (PlanarCheck(vertices, faces, data, creaseAngle))
		{
			result = PLANAR;
		}
		else
			if (SphericalCheck(vertices, data))
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

	void CategoriseFeatures(FeatureExtractionAlgo::ExtractedFeatures features, vector<shared_ptr<SurfaceCategorisationData>> &data, double creaseAngle /*= 5*/)
	{
		data.clear();
		for (auto feature:features)
		{
			shared_ptr<FeatureCategorisation::SurfaceCategorisationData> newData = nullptr;
			auto faces = feature.GetFaces();
			auto vertices = feature.GetVertices();
			auto numEdgeGroups = feature.NumEdgeGroups();
			auto edgeGroups = feature.GetEdgeGroups();
			FeatureCategorisationType result;
			//shared_ptr<SurfaceCategorisationData> data = nullptr;
			if (!PlanarCheck(vertices, faces, newData, creaseAngle))
			{
				if (!SphericalCheck(vertices, newData))
				{
					TubularCheck(vertices, edgeGroups);
					throw;
				}
			}
			data.push_back(newData);
		}
	}

	bool LinearCheck(MatrixXd V, shared_ptr<EdgeCategorisationData> &data, double tolerance = 0.1)
	{
		Vector3d centroid = V.colwise().mean();
		V.rowwise() -= centroid.transpose();
		BDCSVD<MatrixXd> svd(V, ComputeThinU | ComputeThinV);
		Vector3d normal = svd.matrixV().leftCols<1>();
		normal.normalize();
		auto dist2 = V*normal;

		auto maxDist2 = dist2.maxCoeff();

		if (maxDist2 > tolerance)
		{
			auto temp = make_shared<LinearEdgeData>();
			temp->type = LINEAR;
			temp->dir = normal;
			temp->pos = centroid;
			data = temp;
			return true;
		}
		return false;
	}

	bool CircleCheck(vector<TopoDS_Vertex> vertices, shared_ptr<EdgeCategorisationData> &data, double tolerance = 0.1)
	{
		TopoDS_Face face;
		face.Nullify();
		auto temp = make_shared<CircularEdgeData>();
		auto temp2 = make_shared<SurfaceCategorisationData>();
		if (SphericalCheck(vertices, temp2))
		{
			temp->position = static_pointer_cast<SphericalSurfaceData>(temp2)->position;
			temp->radius = static_pointer_cast<SphericalSurfaceData>(temp2)->radius;
			if (PlanarCheck(vertices, { face }, temp2))
			{
				temp->normal = static_pointer_cast<PlanarSurfaceData>(temp2)->normal;
				temp->type = CIRCULAR;
				return true;
			}
		}
		return false;// SphericalCheck(vertices) && PlanarCheck(vertices, { face });
	}

	void CategoriseEdges(const ExtractedFeatures &features, EdgeCategoryMap &edgeCategoryMap)
	{
		edgeCategoryMap.clear();
		for (auto f:features)
		{
			auto edges = f.GetEdges();
			for (auto e:edges)
			{
				//auto vertices = e.EdgeVertices();
				MatrixXd v(e.size(),3);
				for (size_t i = 0; i < e.size(); i++)
				{
					v.row(i) = converter(e[i]);
				}

				//v.rowwise() -= v.colwise().mean();

				EdgeCategorisationType result;
				shared_ptr<EdgeCategorisationData> ptr = nullptr;
				if (!LinearCheck(v, ptr))
				{
					//if (!CircleCheck(vertices, ptr))
					CircleCheck(e, ptr);
				}
				/*else
					if (CircleCheck(vertices, ptr))
					{
						result = CIRCULAR;
					}
					else
						result = FREE;*/

				edgeCategoryMap.insert({ e,ptr });
			}
		}
	}
}