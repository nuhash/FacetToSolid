#include "FeatureExtraction.h"
#include <TopExp.hxx>

#include <TopoDS.hxx>
#include <assert.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <BRep_Tool.hxx>
#include <BRepTools.hxx>

#include <GeomLProp_SLProps.hxx>
using namespace Eigen;
//#include 
void FeatureExtractionAlgo::NormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle /*= 5.0f*/)
{
	TopTools_IndexedDataMapOfShapeListOfShape vertexToFaceMap;
	TopExp::MapShapesAndAncestors(model, TopAbs_VERTEX, TopAbs_FACE, vertexToFaceMap);
	TopTools_IndexedDataMapOfShapeListOfShape faceToVertexMap;
	TopExp::MapShapesAndAncestors(model, TopAbs_FACE, TopAbs_VERTEX, faceToVertexMap);
	TopTools_IndexedMapOfShape vertices;
	TopExp::MapShapes(model, TopAbs_VERTEX, vertices);
	TopTools_IndexedMapOfShape faces;
	TopExp::MapShapes(model, TopAbs_FACE, faces);
	int numFaces = faces.Size();

	/******************************************************************************/
	/*Check to see if duplicate vertices, need to do decide whether to handle case*/
	/******************************************************************************/
	TopoDS_Vertex first = TopoDS::Vertex(vertices.FindKey(1));
	//auto faces = vertexToFaceMap.FindFromKey(first);
	//assert(faces.Size() > 1); 
	/******************************************************************************/
	/******************************************************************************/
	/******************************************************************************/
	TopoDS_Vertex currentVertex = TopoDS::Vertex(vertices.FindKey(1));
	auto currentFaces = vertexToFaceMap.FindFromKey(first);
	int numProcessed = 0;
	ExtractedFeatures features;
	vector<TopoDS_Vertex> vertexQueue;
	while (numProcessed<numFaces) 
	{
		
		//get vertex
		//get faces

		//compute normal tensor
		Matrix3d normalSum;
		normalSum << 0, 0, 0,
			0, 0, 0,
			0, 0, 0;
		TopTools_ListOfShape tempFaces(currentFaces);
		while(!tempFaces.IsEmpty())
		{
			TopoDS_Face currentFace = TopoDS::Face(tempFaces.First());
			tempFaces.RemoveFirst();
			

			Standard_Real umin, umax, vmin, vmax;
			BRepTools::UVBounds(currentFace, umin, umax, vmin, vmax);

			auto currentSurface = BRep_Tool::Surface(currentFace);

			GeomLProp_SLProps props(currentSurface, umin, vmin, 1, 0.01);
			gp_Dir normal = props.Normal();

			Vector3d currentNormal(normal.X(), normal.Y(), normal.Z());

			TopTools_IndexedDataMapOfShapeListOfShape vertexToEdgeMap;
			TopExp::MapShapesAndAncestors(currentFace, TopAbs_VERTEX, TopAbs_EDGE, vertexToEdgeMap);
			auto currentEdges = vertexToEdgeMap.FindFromKey(currentVertex);
			TopoDS_Edge edge1 = TopoDS::Edge(currentEdges.First());
			auto v11 = BRep_Tool::Pnt(TopExp::FirstVertex(edge1));
			auto v12 = BRep_Tool::Pnt(TopExp::LastVertex(edge1));
			Vector3d e1(v11.X() - v12.X(), v11.Y() - v12.Y(), v11.Z() - v12.Z());

			TopoDS_Edge edge2 = TopoDS::Edge(currentEdges.Last());
			auto v21 = BRep_Tool::Pnt(TopExp::FirstVertex(edge2));
			auto v22 = BRep_Tool::Pnt(TopExp::LastVertex(edge2));
			Vector3d e2(v21.X() - v22.X(), v21.Y() - v22.Y(), v21.Z() - v22.Z());

			Vector3d crossProduct = e1.cross(e2);
			
			double currentWeight = sqrt(crossProduct.dot(crossProduct)) / (e1.dot(e1)*e2.dot(e2));

			normalSum += currentWeight * currentNormal * currentNormal.transpose();
		}


		//compute eigen values, vectors

		EigenSolver<Matrix3d> normalTensorSolver(normalSum, true);
		auto eigenvectors = normalTensorSolver.eigenvectors();
		auto eigenValues = normalTensorSolver.eigenvalues();

		//compute vertex classification
		double creaseParameter = 1 / (tan(0.5*creaseAngle)*tan(0.5*creaseAngle)) - 1;
		Vector3d vertexClassification(eigenValues.x().real() - eigenValues.y().real(),
			creaseParameter * (eigenValues.y().real() - eigenValues.z().real()),
			creaseParameter * eigenValues.z().real());
		
		Vector3d::Index vertexClass;
		vertexClassification.maxCoeff(&vertexClass);

		if (vertexClass>0)
		{
			if (features.back().NumFaces() == 0)
			{
				TopoDS_Face face;
				for (size_t i = 0; i < currentFaces.Size(); i++)
				{
					face = TopoDS::Face(currentFaces.First());
					currentFaces.RemoveFirst();
					if (!features.IsFaceProcessed(face))
						break;
				}
				features.back().AddFace(face);
				features.back().AddVertex(currentVertex);
			} 
			else
			{

			}
			//features.back().AddVertex(currentVertex);
		} 
		else
		{
			int n = currentFaces.Size();
			for (size_t i = 0; i < n; i++)
			{
				auto face = TopoDS::Face(currentFaces.First());
				currentFaces.RemoveFirst();
				features.back().AddFace(face);
				features.back().AddVertex(currentVertex);

				auto faceVertices = faceToVertexMap.FindFromKey(face);
				for (size_t j = 0; j < 3; j++)
				{
					auto v = TopoDS::Vertex(faceVertices.First());
					if (v != currentVertex)
						vertexQueue.push_back(v);
					faceVertices.RemoveFirst();
				}
			}
		}
		//if(feature line)
		//{
		//	if(first face)
		//	{
		//		choose a face
		//		add face to feature
		//		set next vertex as other vertex of chosen face
		//		continue
		//	}
		//	else
		//	{
		//		nothing
		//	}
		//	add vertex to feature
		//}
		//else
		//{
		//	add faces vertices to queue
		//	add faces to feature
		//}
	}
}

bool FeatureExtractionAlgo::ExtractedFeatures::IsFaceProcessed(TopoDS_Face face)
{
	for (size_t i = 0; i < this->size(); i++)
	{
		auto currentFeature = this->at(i);
		if(currentFeature.ContainsFace(face))
			return true;
	}
	return false;
}

bool FeatureExtractionAlgo::ExtractedFeature::ContainsFace(TopoDS_Face face)
{
	for (size_t i = 0; i < faces.size(); i++)
	{
		if (face == faces[i])
			return true;
	}
	return false;
}
