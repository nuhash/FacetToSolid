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
#include <TopExp_Explorer.hxx>
#include <algorithm>
#include <stdio.h>

using namespace Eigen;
//#include 
FeatureExtractionAlgo::ExtractedFeatures FeatureExtractionAlgo::NormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle /*= 5.0f*/)
{
	TopTools_IndexedDataMapOfShapeListOfShape vertexToFaceMap;
	TopExp::MapShapesAndAncestors(model, TopAbs_VERTEX, TopAbs_FACE, vertexToFaceMap);
	TopTools_IndexedDataMapOfShapeListOfShape faceToVertexMap;
	TopExp::MapShapesAndAncestors(model, TopAbs_FACE, TopAbs_VERTEX, faceToVertexMap);
	TopTools_IndexedDataMapOfShapeListOfShape shapeToVertexMap;
	TopExp::MapShapesAndAncestors(model, TopAbs_SHAPE, TopAbs_VERTEX, shapeToVertexMap);
	//TopTools_ListOfShape verticesList = shapeToVertexMap.FindKey(1);//FromKey(model);
	//TopTools_IndexedMapOfShape vertices;
	//TopExp::MapShapes(model, TopAbs_VERTEX, vertices);
	TopTools_IndexedMapOfShape faces;
	TopExp::MapShapes(model, TopAbs_FACE, faces);

	TopExp_Explorer vertexExp;
	//vector<TopoDS_Vertex> 
	VerticesSet vertices;
	for (vertexExp.Init(model, TopAbs_VERTEX); vertexExp.More(); vertexExp.Next())
	{
		vertices.insert(TopoDS::Vertex(vertexExp.Current()));
	}

	int numFaces = 0;
	for (vertexExp.Init(model, TopAbs_FACE); vertexExp.More(); vertexExp.Next())
	{
		numFaces++;
	}

	double creaseParameter = 5;// 1 / (tan(0.5*creaseAngle)*tan(0.5*creaseAngle)) - 1;

	

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
	vertexQueue.push_back(*(vertices.rbegin()));
	vertices.erase(*(vertices.rbegin()));
	freopen("output.txt", "w", stdout);
	//cout << "write in file";
	while (features.NumFaces()<numFaces||!vertexQueue.empty()) 
	{
		
		//get vertex
		TopoDS_Vertex currentVertex = vertexQueue.back();
		vertexQueue.pop_back();
		auto currentFaces = vertexToFaceMap.FindFromKey(currentVertex);
		//auto tempFaces = vertexToFaceMap.FindFromKey(currentVertex);
		//get faces

		//compute normal tensor
		Matrix3d normalTensorSum;
		normalTensorSum << 0, 0, 0,
			0, 0, 0,
			0, 0, 0;
		Vector3d normalSum(0, 0, 0);
		//TopTools_ListOfShape tempFaces(currentFaces);
		//while(!tempFaces.IsEmpty())
		//currentFaces.Size()
		if (true)
		{
			cout << "Surface check\n";
		}
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
			auto inter = currentNormal * currentNormal.transpose();
			if (true)
			{
				cout << normal.X() << ";" << normal.Y() << ";" << normal.Z() << ";" << currentWeight << "; \n";
				
				cout << inter << "\n\n";
			}

			
			normalTensorSum += currentWeight * inter;
			normalSum += currentWeight * currentNormal;
			ite++;
		}


		//compute eigen values, vectors

		EigenSolver<Matrix3d> normalTensorSolver(normalTensorSum, true);
		auto eigenvectors = normalTensorSolver.eigenvectors();
		auto eigenValuesComplex = normalTensorSolver.eigenvalues();
		Vector3d eigenValues(eigenValuesComplex.x().real(), eigenValuesComplex.y().real(), eigenValuesComplex.z().real());
		std::sort(eigenValues.data(), eigenValues.data() + eigenValues.size());
		eigenValues.reverseInPlace();
		//compute vertex classification

		Vector3d vertexClassification(eigenValues.x() - eigenValues.y(),
			creaseParameter * (eigenValues.y() - eigenValues.z()),
			creaseParameter * eigenValues.z());
		
		Vector3d::Index vertexClass;
		vertexClassification.maxCoeff(&vertexClass);
		if (true)
		{
			cout << normalTensorSum << "\n";
			cout << normalSum.x() << ";" << normalSum.y() << ";" << normalSum.z() << ";\n";
			cout << eigenValues << "\n";
			cout << eigenvectors << "\n";
			cout << vertexClassification.x() << ";" << vertexClassification.y() << ";" << vertexClassification.z() << "\n\n";
			cout << "End surface check\n\n";
		}
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
				TopExp_Explorer faceToVertices;
				int i = 0;
				for (faceToVertices.Init(face, TopAbs_VERTEX); faceToVertices.More()&&i<2; faceToVertices.Next())
				{
					auto v = TopoDS::Vertex(faceToVertices.Current());
					if (!currentVertex.IsSame(v)&&!IsVertexInQueue(vertexQueue, v))
					{
						vertexQueue.push_back(v);
						//verticesList.Remove(TopoDS::Vertex(*ite));
						vertices.erase(v);
						i++;
					}
				}
				//for (auto ite = verts.begin(); ite != verts.end(); ite++)
				//{
				//	if (*ite != currentVertex)
				//	{
				//		vertexQueue.push_back(TopoDS::Vertex(*ite));
				//		//verticesList.Remove(TopoDS::Vertex(*ite));
				//		vertices.erase(std::remove(vertices.begin(), vertices.end(), *ite), vertices.end());
				//	}
				//}
			}
			else
			{

			}
			features.back().AddEdgeVertex(currentVertex,(EdgeVertexType)vertexClass);
			//vertices.insert(currentVertex);
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
				TopExp_Explorer faceToVertices;
				int i = 0;
				for (faceToVertices.Init(face, TopAbs_VERTEX); faceToVertices.More()&&i<2; faceToVertices.Next())
				{
					auto v = TopoDS::Vertex(faceToVertices.Current());
					auto cv = BRep_Tool::Pnt(v);
					cout << cv.X() << ";" << cv.Y() << ";" << cv.Z() << ";\n";
					if (!features.back().ContainsVertex(v)&&!IsVertexInQueue(vertexQueue,v))
					{
						vertexQueue.push_back(v);
						//verticesList.Remove(TopoDS::Vertex(*ite));
						vertices.erase(v);
						i++;
					}
					//faceToVertices.More()
				}
				cout << "\n";
				ite++;
			}
			cout << "------------------------------------------------------\n";
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

		if (vertexQueue.empty()&& features.NumFaces()<numFaces)
		{
			ExtractedFeature newFeature;
			features.push_back(newFeature);
			vertexQueue.push_back(*(vertices.rbegin()));
			vertices.erase(*(vertices.rbegin()));
		}

	}
	//features.ProcessEdges(model);
	return features;
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

bool FeatureExtractionAlgo::ExtractedFeatures::IsVertexProcessed(TopoDS_Vertex vertex)
{
	for (size_t i = 0; i < this->size(); i++)
	{
		auto currentFeature = this->at(i);
		if (currentFeature.ContainsVertex(vertex))
			return true;
	}
	return false;
}

int FeatureExtractionAlgo::ExtractedFeatures::NumFaces()
{
	int result = 0;
	for (size_t i = 0; i < this->size(); i++)
	{
		result += this->at(i).NumFaces();
	}
	return result;
}

void FeatureExtractionAlgo::ExtractedFeatures::ProcessEdges(TopoDS_Shape shape)
{
	for (size_t i = 0; i < size(); i++)
	{
		at(i).ProcessEdges(shape);
	}
}

void FeatureExtractionAlgo::ExtractedFeature::AddFace(TopoDS_Face face)
{
	/*for (size_t i = 0; i < faces.size(); i++)
	{
		if (face.IsSame(faces[i]))
			return;
	}*/
	if(!ContainsFace(face))
	faces.push_back(face);
}

bool FeatureExtractionAlgo::ExtractedFeature::ContainsFace(TopoDS_Face face)
{
	for (size_t i = 0; i < faces.size(); i++)
	{
		if (face.IsSame(faces[i]))
			return true;
	}
	return false;
}

bool FeatureExtractionAlgo::ExtractedFeature::ContainsVertex(TopoDS_Vertex vertex)
{
	for (size_t i = 0; i < vertices.size(); i++)
	{
		if (vertex.IsSame(vertices[i]))
			return true;
	}
	return false;
}

TopoDS_Vertex FeatureExtractionAlgo::ExtractedFeature::GetNearbyEdgeVertex(TopoDS_Shape shape, TopoDS_Vertex vertex)
{
	return vertex;
}

void FeatureExtractionAlgo::ExtractedFeature::ProcessEdges(TopoDS_Shape shape)
{
	TopTools_IndexedDataMapOfShapeListOfShape vertexToEdgeMap;
	TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, vertexToEdgeMap);

	TopoDS_Vertex currentVertex;
	EdgeVertexType currentType;
	vector<ExtractedFeatureEdge> edgeQueue;
	if(!CreateNewEdge(edgeQueue, vertexToEdgeMap, currentVertex, currentType))
		return;
	
	auto workingEdge = edgeQueue.back();
	edgeQueue.pop_back();
	while (edgeVertices.size()>0)
	{

		currentVertex = workingEdge.back();
		auto currentEdges = vertexToEdgeMap.FindFromKey(currentVertex);
		TopoDS_Vertex first, second;
		bool processed = false;
		for (auto ite = currentEdges.begin(); ite != currentEdges.end(); ite++)
		{
			TopExp::Vertices(TopoDS::Edge(*ite), first, second);
			TopoDS_Vertex v;

			if (!currentVertex.IsSame(first))
			{
				v = first;
			} 
			else
//			if(!currentVertex.IsSame(second))
			{
				v = second;
			}

			if (workingEdge.Type() == FINITE && IsVertexEdgeProcessed(v)) //Check if vertex is a corner
			{
				workingEdge.AddVertex(v);
				processed = true;
			}

			if (workingEdge.Type()==CONTINUOUS && v.IsSame(workingEdge.front()))
			{
				//Continuous edge has looped on itself
				processed = true;
				break;
			}

			if (v.IsSame(workingEdge.back()) || !IsVertexEdge(v) )
			{
				continue;
			}

			workingEdge.AddVertex(v);
			int pos = FindEdgeVertex(v);
			auto type = edgeVerticesTypes[pos];
			currentVertex = v;
			currentEdges = vertexToEdgeMap.FindFromKey(currentVertex);
			ite = currentEdges.begin();
			if (type==CORNER)
			{
				//Edge has ended
				//Create new edge
				processed = true;
				break;
			}
			edgeVertices.erase(edgeVertices.begin() + pos);
			edgeVerticesTypes.erase(edgeVerticesTypes.begin() + pos);
		}

		if (processed)
		{
			//
			extractedEdges.push_back(workingEdge);
			workingEdge = edgeQueue.back();
			edgeQueue.pop_back();
		}

		//if (!processed)
		//{
		//	dead edge
		//	check if valid edges remain
		//}

	}
}

int FeatureExtractionAlgo::ExtractedFeature::FindCornerVertex()
{
	int i;
	for (i = 0; i < edgeVerticesTypes.size(); i++)
	{
		if (edgeVerticesTypes[i]==EdgeVertexType::CORNER)
		{
			return i;
		}
	}
	return -1;
}

bool FeatureExtractionAlgo::ExtractedFeature::CreateNewEdge(vector<ExtractedFeatureEdge>& queue, TopTools_IndexedDataMapOfShapeListOfShape v2e, TopoDS_Vertex &vertex, EdgeVertexType &type)
{
	if (edgeVertices.size() == 0)
		return false;
	//extractedEdges.push_back(ExtractedFeatureEdge());
	
	int cornerPos = FindCornerVertex();
	if (cornerPos == -1) //Continuous edge/s
	{
		ExtractedFeatureEdge newEdge;
		newEdge.Type(CONTINUOUS);
		newEdge.AddVertex(edgeVertices.back());
		vertex = edgeVertices.back();
		edgeVertices.pop_back();
		type = edgeVerticesTypes.back();
		edgeVerticesTypes.pop_back();
		queue.push_back(newEdge);
	}
	else
	{
		auto currentVertex = edgeVertices.at(cornerPos);
		vertex = currentVertex;
		type = edgeVerticesTypes.at(cornerPos);
		edgeVertices.erase(edgeVertices.begin() + cornerPos);
		edgeVerticesTypes.erase(edgeVerticesTypes.begin() + cornerPos);

		TopoDS_Vertex first, second;

		auto edges = v2e.FindFromKey(currentVertex);
		for (auto ite = edges.begin(); ite != edges.end(); ite++)
		{
			TopExp::Vertices(TopoDS::Edge(*ite), first, second);
			TopoDS_Vertex v;
			if (!currentVertex.IsSame(first))
			{
				v = first;
			}
			else
//			if(!currentVertex.IsSame(second))
			{
				v = second;
			}
			if (IsVertexEdge(v))
			{
				ExtractedFeatureEdge newEdge;
				newEdge.Type(FINITE);
				newEdge.AddVertex(currentVertex);
				newEdge.AddVertex(v);
				queue.push_back(newEdge);
				int pos = FindEdgeVertex(v);
				edgeVertices.erase(edgeVertices.begin() + pos);
				edgeVerticesTypes.erase(edgeVerticesTypes.begin() + pos);
			}	
		}
	}
	return true;
}

int FeatureExtractionAlgo::ExtractedFeature::FindEdgeVertex(TopoDS_Vertex vertex)
{
	int i;
	for (i = 0; i < edgeVerticesTypes.size(); i++)
	{
		if (vertex.IsSame(edgeVertices[i]))
		{
			return i;
		}
	}
	return -1;
}

bool FeatureExtractionAlgo::ExtractedFeature::IsVertexEdge(TopoDS_Vertex vertex)
{
	for (size_t i = 0; i < edgeVertices.size(); i++)
	{
		if (vertex.IsSame(edgeVertices[i]))
		{
			return true;
		}
	}
	return false;
}

bool FeatureExtractionAlgo::ExtractedFeature::IsVertexEdgeProcessed(TopoDS_Vertex vertex)
{
	for (size_t i = 0; i < extractedEdges.size(); i++)
	{
		for (size_t j = 0; j < extractedEdges[i].size(); j++)
		{
			if (vertex.IsSame(extractedEdges[i][j]))
				return true;
		}
	}
	return false;
}
