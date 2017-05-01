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
		auto temp = make_shared<PlanarSurfaceData>();
		double result;
		temp->type = PLANAR;
		MatrixXd A(vertices.size(), 3);
		for (size_t i = 0; i < vertices.size(); i++)
		{
			A.row(i) = converter(vertices[i]);

		}
		Vector3d centroid = A.colwise().mean();
		if (vertices.size() == 3)
		{
			temp->normal = converter(faces[0]);
			result = 0;
			temp->centroid = centroid;
		}
		else
		{

			A.rowwise() -= centroid.transpose();
			BDCSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
			Vector3d normal = svd.matrixV().rightCols<1>();

			if (!faces[0].IsNull())
			{
				auto faceNormal = converter(faces[0]);
				double dotProd = faceNormal.dot(normal);

				normal *= sgn(dotProd); //sign(dotProduct);
			}
			//normal.normalize();

			auto dist2 = (A*normal).cwiseAbs2();

			auto averageError = dist2.mean();

			//auto temp = make_shared<PlanarSurfaceData>();
			temp->normal = Vector3d(normal.x(), normal.y(), normal.z());
			temp->type = PLANAR;
			temp->centroid = centroid;

			
			result = averageError;
		}
		data = temp;
		return result;
	}

	float SphericalCheck(const vector<TopoDS_Vertex> vertices, shared_ptr<SurfaceCategorisationData> &data, int numFaces, int numEdgeVerts)
	{
		double result;
		if (numFaces < 8 || numEdgeVerts > 0.4 * vertices.size())
			result = HUGE_VAL; //prevent small planar features being categorised as spherical
		else
		{

			MatrixXd A(vertices.size(), 4);
			VectorXd b(vertices.size());
			MatrixXd V(vertices.size(), 3);
			for (size_t i = 0; i < vertices.size(); i++)
			{
				auto pnt = BRep_Tool::Pnt(vertices[i]);
				b(i) = pnt.X()*pnt.X() + pnt.Y()*pnt.Y() + pnt.Z()*pnt.Z();
				A(i, 0) = 2 * pnt.X();
				A(i, 1) = 2 * pnt.Y();
				A(i, 2) = 2 * pnt.Z();
				A(i, 3) = 1;

				V.row(i) = converter(vertices[i]);
			}

			Vector4d x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
			Vector3d centre(x(0), x(1), x(2));
			double radius = sqrt(x(3) + x(0)*x(0) + x(1)*x(1) + x(2)*x(2));

			double maxError = 0;

			auto averageError2 = ((V.rowwise() - centre.transpose()).rowwise().norm().array() - radius).cwiseAbs2().mean();

			auto temp = make_shared<SphericalSurfaceData>();
			temp->position = centre;
			temp->radius = radius;
			temp->type = SPHERICAL;
			data = temp;
			result = averageError2;
		}
		return result;
	}

	Matrix3d TransformToCoordSpace(Vector3d &x, Vector3d &y, Vector3d &z)
	{
		Matrix3d t;
		t(0, 0) = x.dot(Vector3d(1, 0, 0));
		t(0, 1) = x.dot(Vector3d(0, 1, 0));
		t(0, 2) = x.dot(Vector3d(0, 0, 1));
		t(1, 0) = y.dot(Vector3d(1, 0, 0));
		t(1, 1) = y.dot(Vector3d(0, 1, 0));
		t(1, 2) = y.dot(Vector3d(0, 0, 1));
		t(2, 0) = z.dot(Vector3d(1, 0, 0));
		t(2, 1) = z.dot(Vector3d(0, 1, 0));
		t(2, 2) = z.dot(Vector3d(0, 0, 1));
		return t;
	}

	double CylindricalCheck(const vector<TopoDS_Vertex> vertices, EdgeGroups& edgeGroups, shared_ptr<SurfaceCategorisationData> &data, EdgeCategoryMap &edgeCategoryMap)
	{
		double result;
		if (edgeGroups.size() != 2)
		{
			result = HUGE_VAL; //Only dealing with perfect tubes
		}
		else
		{
			//Check if ends are circles 
			bool circleCheck = true;
			for (auto &eG : edgeGroups)
			{
				if (eG.size()>1 || eG.front().Type() != CONTINUOUS || edgeCategoryMap[eG.front()]->type!=CIRCULAR)
				{
					circleCheck = false;
					break;
				}
			}

			if (!circleCheck)
			{
				result = HUGE_VAL;
			}
			else
			{
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
				Vector3d zDir = svd.matrixV().leftCols<1>();
				Vector3d xDir = svd.matrixV().rightCols<1>();
				Vector3d yDir = svd.matrixV().col(1);
				
				auto t = TransformToCoordSpace(xDir, yDir, zDir);

				MatrixXd vt = v*t.transpose();
				Vector3d scale = vt.colwise().maxCoeff() - vt.colwise().minCoeff();
				MatrixXd scaled = vt.array().rowwise() / scale.transpose().array();

				BDCSVD<MatrixXd> svd2(scaled, ComputeThinU | ComputeThinV);
				zDir = svd2.matrixV().leftCols<1>();
				xDir = svd2.matrixV().rightCols<1>();
				yDir = svd2.matrixV().col(1);

				

				auto t2 = TransformToCoordSpace(xDir, yDir, zDir);
				Vector3d axis = t.transpose()*zDir;
				MatrixXd vt2 = vt*t2.transpose();

				

				VectorXd b = vt2.leftCols<2>().rowwise().squaredNorm();
				MatrixXd A(vt2.rows(), 3);
				A.col(0) = 2 * vt2.col(0);
				A.col(1) = 2 * vt2.col(1);
				A.col(2).array() = 1;
				//b=Ax

				Vector3d x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
				Vector3d localCentre(x(0), x(1), 0);
				Vector3d worldCentre = t.transpose()*t2.transpose()*localCentre + centroid;
				double radius = sqrt(x(2) + x(1)*x(1) + x(0)*x(0));
				

				VectorXd distFromAxis = vt2.leftCols<2>().rowwise().norm();
				
				VectorXd err2DistFromAxis = (distFromAxis.array() - radius).cwiseAbs2();

				auto temp = make_shared<CylindricalSurfaceData>();
				temp->dir = axis;
				temp->position = worldCentre;
				temp->radius = radius;
				temp->type = CYLINDRICAL;
				temp->vMax = t.col(2).maxCoeff();
				temp->vMin = t.col(2).minCoeff();
				data = temp;
				result = err2DistFromAxis.mean();
			}
		}
		return result;
	}



	void CategoriseFeatures(FeatureExtractionAlgo::ExtractedFeatures features, vector<shared_ptr<SurfaceCategorisationData>> &data, EdgeCategoryMap &edgeCategoryMap, double tolerance/*= 0.1*/)
	{
		data.clear();
		for (auto feature:features)
		{
			shared_ptr<FeatureCategorisation::SurfaceCategorisationData> newData = nullptr;
			auto faces = feature.GetFaces();
			auto vertices = feature.GetVertices();
			auto numEdgeGroups = feature.NumEdgeGroups();
			EdgeGroups& edgeGroups = feature.GetEdgeGroups();
			FeatureCategorisationType result;
			int numEdgeVerts = feature.NumEdgeVertices();
			array<shared_ptr<SurfaceCategorisationData>, 3> dataArray;
			dataArray.fill(nullptr);
			Vector3d r2(0, 0, 0);
			r2(0) = PlanarCheck(vertices, faces, dataArray[0], tolerance);
			r2(1) = SphericalCheck(vertices, dataArray[1], faces.size(), numEdgeVerts);
			r2(2) = CylindricalCheck(vertices, edgeGroups, dataArray[2], edgeCategoryMap);
			
			int minCoeff;
			r2.minCoeff(&minCoeff);
			data.push_back(dataArray[minCoeff]);
		}
	}

	bool LinearCheck(MatrixXd V, shared_ptr<EdgeCategorisationData> &data, double tolerance = 0.1)
	{
		double result;

		Vector3d centroid = V.colwise().mean();
		V.rowwise() -= centroid.transpose();
		BDCSVD<MatrixXd> svd(V, ComputeThinU | ComputeThinV);
		Vector3d axis = svd.matrixV().leftCols<1>();
		Vector3d normal1 = svd.matrixV().rightCols<1>();
		Vector3d normal2 = svd.matrixV().col(1);
		axis.normalize();
		auto dist2 = V*axis;
		auto t = TransformToCoordSpace(normal1, normal2, axis);
		MatrixXd vt = V*t.transpose();
		VectorXd dist2FromAxis = vt.leftCols<2>().rowwise().squaredNorm();

		result = dist2FromAxis.mean();

		auto temp = make_shared<LinearEdgeData>();
		temp->type = LINEAR;
		temp->dir = axis;
		temp->pos = centroid;
		data = temp;

		return result;
	}

	double CircleCheck(MatrixXd V, shared_ptr<EdgeCategorisationData> &data, int numEdges, double tolerance = 0.1)
	{
		double result;
		if (numEdges < 4)
		{
			result = HUGE_VAL;
		}
		else
		{
			TopoDS_Face face;
			face.Nullify();

			Vector3d centroid = V.colwise().mean();

			V.rowwise() -= centroid.transpose();
			BDCSVD<MatrixXd> svd(V, ComputeThinU | ComputeThinV);
			Vector3d zDir = svd.matrixV().rightCols<1>();
			Vector3d xDir = svd.matrixV().leftCols<1>();
			Vector3d yDir = svd.matrixV().col(1);

			auto t = TransformToCoordSpace(xDir, yDir, zDir);

			MatrixXd vt = V*t.transpose();

			VectorXd b = vt.rowwise().squaredNorm();
			MatrixXd A(V.rows(), 3);
			A.col(0) = 2 * vt.col(0);
			A.col(1) = 2 * vt.col(1);
			A.col(2).array() = 1;
			//b=Ax
			
			Vector3d x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
			Vector3d localCentre(x(0), x(1), 0);
			double radius = sqrt(x(2) + x(1)*x(1) + x(0)*x(0));

			auto averageRadialError2 = ((vt.rowwise() - localCentre.transpose()).rowwise().norm().array() - radius).cwiseAbs2().mean();
			auto averageZError2 = vt.col(2).cwiseAbs2().mean();
			auto averageError2 = averageZError2 + averageRadialError2;

			Vector3d worldCentre = t.transpose()*localCentre + centroid; //Reverse transform to original vertex set

			Vector3d point1 = vt.row(0);
			Vector3d point2 = vt.row(1);
			Vector3d point3 = vt.bottomRows<1>();

			Vector3d radialDir = point1 - localCentre;
			Vector3d localTangentialDir(radialDir.y(), -radialDir.x(), 0);
			double dotProduct = localTangentialDir.dot(point2 - point1);
			localTangentialDir *= Utilities::sgn(dotProduct);
			Vector3d worldTangentialDir = t.transpose()*localTangentialDir;

			result = averageError2;
			auto temp = make_shared<CircularEdgeData>();
			temp->position = worldCentre;
			temp->normal = zDir;
			temp->radius = radius;
			temp->p1 = V.row(0) +centroid.transpose();
			temp->p2 = V.bottomRows<1>()+centroid.transpose();
			temp->tangent = worldTangentialDir;
			temp->type = CIRCULAR;
			//auto temp2 = make_shared<SurfaceCategorisationData>();
			//result = SphericalCheck(edge, temp2, 0x7FFFFFFF);
			//temp->position = static_pointer_cast<SphericalSurfaceData>(temp2)->position;
			//temp->radius = static_pointer_cast<SphericalSurfaceData>(temp2)->radius;
			//result += PlanarCheck(edge, { face }, temp2);
			//temp->normal = static_pointer_cast<PlanarSurfaceData>(temp2)->normal;
			//temp->type = CIRCULAR;
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
				r2(1) = CircleCheck(v, dataArray[1],e.size());
				int minCoeff;
				r2.minCoeff(&minCoeff);
				edgeCategoryMap.insert({ e,dataArray[minCoeff]});
			}
		}
	}

	void Categorise(const ExtractedFeatures &features, EdgeCategoryMap &edgeCategoryMap, vector<shared_ptr<SurfaceCategorisationData>> &data, double tolerance /*= 0.1*/)
	{
		CategoriseEdges(features, edgeCategoryMap);
		CategoriseFeatures(features, data, edgeCategoryMap, tolerance);
	}
}