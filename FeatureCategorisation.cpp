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
#include <array>

using namespace Eigen;
using namespace std;
using namespace Utilities;
using namespace FeatureExtractionAlgo;

namespace FeatureCategorisation {


	double PlanarCheck(const vector<TopoDS_Vertex> vertices, const vector<TopoDS_Face> faces, shared_ptr<SurfaceCategorisationData> &data, double tolerance=0.1)
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
			normal *= (dotProd > 0) - (dotProd < 0); //sign(dotProduct);
		}
		//normal.normalize();

		auto dist2 = (A*normal).cwiseAbs2();
		
		auto averageError = dist2.mean();
		
		auto temp = make_shared<PlanarSurfaceData>();
		temp->normal = Vector3d(normal.x(), normal.y(), normal.z());
		temp->type = PLANAR;
		temp->centroid = centroid;
		temp->normal2 = gp_Dir(normal.x(), normal.y(), normal.z());
		temp->normal3 = Vector3d(temp->normal);

		data = temp;
		return averageError;
	}

	float SphericalCheck(const vector<TopoDS_Vertex> vertices, shared_ptr<SurfaceCategorisationData> &data, int numFaces)
	{
		double result;
		if (numFaces < 8)
			result = HUGE_VAL; //prevent small planar features being categorised as spherical\
		else
		{

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

			auto averageError2 = (A.rowwise() - centre.transpose()).rowwise().squaredNorm().mean();

			auto temp = make_shared<SphericalSurfaceData>();
			temp->position = centre;
			temp->radius = radius;
			temp->type = SPHERICAL;
			data = temp;
			result = averageError2;
		}
		return result;
	}

	double CylindricalCheck(const vector<TopoDS_Vertex> vertices, EdgeGroups edgeGroups, shared_ptr<SurfaceCategorisationData> &data, int numSections = 5, double allowableError = 0.1 /*= 0.1*/)
	{
		double result;
		if (edgeGroups.size() != 2)
		{
			result = HUGE_VAL; //Only dealing with perfect tubes
		}
		else
		{
			//Check if ends are circles 

			MatrixXd v(vertices.size(), 3);
			for (size_t i = 0; i < vertices.size(); i++)
			{
				auto pnt = BRep_Tool::Pnt(vertices[i]);
				v(i, 0) = pnt.X();
				v(i, 1) = pnt.Y();
				v(i, 2) = pnt.Z();
			}

			Vector3d centroid = v.colwise().mean();
			v.rowwise() -= centroid.transpose();
			BDCSVD<MatrixXd> svd(v, ComputeThinU | ComputeThinV);
			Vector3d axis = svd.matrixV().leftCols<1>();
			Vector3d normal1 = svd.matrixV().rightCols<1>();
			Vector3d normal2 = svd.matrixV().col(2);

			Matrix3d t;
			t(0, 0) = normal1.dot(Vector3d(1, 0, 0));
			t(0, 1) = normal1.dot(Vector3d(0, 1, 0));
			t(0, 2) = normal1.dot(Vector3d(0, 0, 1));
			t(1, 0) = normal2.dot(Vector3d(1, 0, 0));
			t(1, 1) = normal2.dot(Vector3d(0, 1, 0));
			t(1, 2) = normal2.dot(Vector3d(0, 0, 1));
			t(2, 0) = axis.dot(Vector3d(1, 0, 0));
			t(2, 1) = axis.dot(Vector3d(0, 1, 0));
			t(2, 2) = axis.dot(Vector3d(0, 0, 1));

			MatrixXd vt = v*t.transpose();

			VectorXd distFromAxis = (vt.col(0).cwiseAbs2() + vt.col(1).cwiseAbs2()).cwiseSqrt();
			VectorXd err2DistFromAxis = (distFromAxis.array() - distFromAxis.mean()).cwiseAbs2();

			result = err2DistFromAxis.mean();
		}
		return result;
	}

	//FeatureCategorisation::FeatureCategorisationType CategoriseFeature(FeatureExtractionAlgo::ExtractedFeature feature, shared_ptr<SurfaceCategorisationData> &data, double creaseAngle /*= 5*/)
	//{
	//	auto faces = feature.GetFaces();
	//	auto vertices = feature.GetVertices();
	//	auto numEdgeGroups = feature.NumEdgeGroups();
	//	auto edgeGroups = feature.GetEdgeGroups();
	//	FeatureCategorisationType result;
	//	//shared_ptr<SurfaceCategorisationData> data = nullptr;
	//	if (PlanarCheck(vertices, faces, data, creaseAngle))
	//	{
	//		result = PLANAR;
	//	}
	//	else
	//		if (SphericalCheck(vertices, data, faces.size()))
	//		{
	//			return SPHERICAL;
	//		}
	//		else
	//			if (CylindricalCheck(vertices, edgeGroups))
	//			{
	//				return CYLINDRICAL;
	//			}

	//	//feature.Type(result);

	//	return result;
	//}

	void CategoriseFeatures(FeatureExtractionAlgo::ExtractedFeatures features, vector<shared_ptr<SurfaceCategorisationData>> &data, double tolerance/*= 0.1*/)
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
			//TODO LIST
			//Vector of doubles
			//Array of pointers
			//Each "Check" returns average error squared to vector
			//				returns data into array of pointers
			//Lowest error squared is used
			//Concurrency

			array<shared_ptr<SurfaceCategorisationData>, 3> dataArray;
			dataArray.fill(nullptr);
			Vector3d r2(0, 0, 0);
			r2(0) = PlanarCheck(vertices, faces, dataArray[0], tolerance);
			r2(1) = SphericalCheck(vertices, dataArray[1], faces.size());
			r2(2) = CylindricalCheck(vertices, edgeGroups, dataArray[2]);
			
			int minCoeff;
			r2.minCoeff(&minCoeff);
			data.push_back(dataArray[minCoeff]);
		}
	}

	bool LinearCheck(MatrixXd V, shared_ptr<EdgeCategorisationData> &data, double tolerance = 0.1)
	{
		Vector3d centroid = V.colwise().mean();
		V.rowwise() -= centroid.transpose();
		BDCSVD<MatrixXd> svd(V, ComputeThinU | ComputeThinV);
		Vector3d axis = svd.matrixV().leftCols<1>();
		axis.normalize();
		auto dist2 = V*axis;

		auto maxDist2 = dist2.maxCoeff();

		if (maxDist2 > tolerance)
		{
			auto temp = make_shared<LinearEdgeData>();
			temp->type = LINEAR;
			temp->dir = axis;
			temp->pos = centroid;
			data = temp;
			return true;
		}
		return false;
	}

	double CircleCheck(ExtractedFeatureEdge edge, shared_ptr<EdgeCategorisationData> &data, double tolerance = 0.1)
	{
		double result;
		if (edge.Type() == CONTINUOUS)
		{
			result = HUGE_VAL;
		}
		else
		{
			TopoDS_Face face;
			face.Nullify();

			auto temp = make_shared<CircularEdgeData>();
			auto temp2 = make_shared<SurfaceCategorisationData>();
			result = SphericalCheck(edge, temp2, 0x7FFFFFFF);
			temp->position = static_pointer_cast<SphericalSurfaceData>(temp2)->position;
			temp->radius = static_pointer_cast<SphericalSurfaceData>(temp2)->radius;
			result += PlanarCheck(edge, { face }, temp2);
			temp->normal = static_pointer_cast<PlanarSurfaceData>(temp2)->normal;
			temp->type = CIRCULAR;
			data = temp;
		}
		return result;// SphericalCheck(vertices) && PlanarCheck(vertices, { face });
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

				array<shared_ptr<EdgeCategorisationData>, 2> dataArray;
				dataArray.fill(nullptr);
				Vector2d r2(0, 0);
				r2(0) = LinearCheck(v, dataArray[0]);
				r2(1) = CircleCheck(e, dataArray[1]);
				int minCoeff;
				r2.minCoeff(&minCoeff);
				edgeCategoryMap.insert({ e,dataArray[minCoeff]});
			}
		}
	}
}