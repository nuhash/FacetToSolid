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
	TopTools_IndexedDataMapOfShapeListOfShape shapeToVertexMap;
	TopExp::MapShapesAndAncestors(model, TopAbs_SHAPE, TopAbs_VERTEX, shapeToVertexMap);
	TopTools_ListOfShape verticesList = shapeToVertexMap.FindFromKey(model);
	TopTools_IndexedMapOfShape vertices;
	TopExp::MapShapes(model, TopAbs_VERTEX, vertices);
	TopTools_IndexedMapOfShape faces;
	TopExp::MapShapes(model, TopAbs_FACE, faces);

	double creaseParameter = 1 / (tan(0.5*creaseAngle)*tan(0.5*creaseAngle)) - 1;

	int numFaces = faces.Size();

	/******************************************************************************/
	/*Check to see if duplicate vertices, need to do decide whether to handle case*/
	/******************************************************************************/
	//TopoDS_Vertex first = TopoDS::Vertex(vertices.FindKey(1));
	//auto faces = vertexToFaceMap.FindFromKey(first);
	//assert(faces.Size() > 1); 
	/******************************************************************************/
	/******************************************************************************/
	/******************************************************************************/
	
	
	int numProcessed = 0;
	ExtractedFeatures features;
	ExtractedFeature firstFeature;
	features.push_back(firstFeature);
	vector<TopoDS_Vertex> vertexQueue;
	vertexQueue.push_back(TopoDS::Vertex(verticesList.First()));
	verticesList.RemoveFirst();
	while (numProcessed<numFaces||!vertexQueue.empty()) 
	{
		
		//get vertex
		TopoDS_Vertex currentVertex = vertexQueue.back();
		vertexQueue.pop_back();
		auto currentFaces = vertexToFaceMap.FindFromKey(currentVertex);
		//auto tempFaces = vertexToFaceMap.FindFromKey(currentVertex);
		//get faces

		//compute normal tensor
		Matrix3d normalSum;
		normalSum << 0, 0, 0,
			0, 0, 0,
			0, 0, 0;
		//TopTools_ListOfShape tempFaces(currentFaces);
		//while(!tempFaces.IsEmpty())
		for (auto ite = currentFaces.begin(); ite != currentFaces.end(); ite++)
		{
			TopoDS_Face currentFace = TopoDS::Face(*ite);

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
				for (auto ite = currentFaces.begin(); ite != currentFaces.end(); ite++)
				{
					face = TopoDS::Face(*ite);
					if (!features.IsFaceProcessed(face))
						break;
				}
				features.back().AddFace(face);
				numProcessed++;
				auto verts = faceToVertexMap.FindFromKey(face);
				for (auto ite = verts.begin(); ite != verts.end(); ite++)
				{
					if (*ite != currentVertex)
					{
						vertexQueue.push_back(TopoDS::Vertex(*ite));
						verticesList.Remove(TopoDS::Vertex(*ite));
					}
				}
			}
			else
			{

			}
			features.back().AddEdgeVertex(currentVertex,(EdgeVertexType)vertexClass);
		} 
		else
		{
			int n = currentFaces.Size();
			for (auto ite = currentFaces.begin(); ite != currentFaces.end(); ite++)
			{
				auto face = TopoDS::Face(*ite);
				features.back().AddFace(face);
				//features.back().AddVertex(currentVertex);
				numProcessed++;
				auto faceVertices = faceToVertexMap.FindFromKey(face);
				for (auto ite2 = faceVertices.begin(); ite2 != faceVertices.end(); ite2++)
				{
					auto v = TopoDS::Vertex(*ite2);
					if (v != currentVertex)
					{
						vertexQueue.push_back(v);
						verticesList.Remove(v);
					}
				}
			}
		}
		features.back().AddVertex(currentVertex);
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

		if (vertexQueue.empty()&& numProcessed<numFaces)
		{
			ExtractedFeature newFeature;
			features.push_back(newFeature);
		}

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

void FeatureExtractionAlgo::ExtractedFeature::AddFace(TopoDS_Face face)
{
	for (size_t i = 0; i < faces.size(); i++)
	{
		if (faces[i] == face)
			return;
	}
	faces.push_back(face);
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
