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
#include <stack>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Builder.hxx>
#include <queue>

using namespace Eigen;

namespace FeatureExtractionAlgo {

	Vector3d operator|(const TopoDS_Vertex v, ConversionHelpers c)
	{
		return c(v);
	}

	//#include 
	ExtractedFeatures NormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle /*= 5.0f*/)
	{
		TopTools_IndexedDataMapOfShapeListOfShape vertexToFaceMap;
		TopExp::MapShapesAndAncestors(model, TopAbs_VERTEX, TopAbs_FACE, vertexToFaceMap);
		TopTools_IndexedDataMapOfShapeListOfShape faceToVertexMap;
		TopExp::MapShapesAndAncestors(model, TopAbs_FACE, TopAbs_VERTEX, faceToVertexMap);
		TopTools_IndexedDataMapOfShapeListOfShape shapeToVertexMap;
		TopExp::MapShapesAndAncestors(model, TopAbs_SHAPE, TopAbs_VERTEX, shapeToVertexMap);

		TopTools_IndexedMapOfShape faces;
		TopExp::MapShapes(model, TopAbs_FACE, faces);

		TopExp_Explorer vertexExp;

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

		int numProcessed = 0;
		ExtractedFeatures features;
		ExtractedFeature firstFeature;
		features.push_back(firstFeature);
		vector<TopoDS_Vertex> vertexQueue;
		vertexQueue.push_back(*(vertices.rbegin()));
		vertices.erase(*(vertices.rbegin()));
		freopen("output.txt", "w", stdout);
		//cout << "write in file";
		while (features.NumFaces() < numFaces || !vertexQueue.empty())
		{
			TopoDS_Vertex currentVertex = vertexQueue.back();
			vertexQueue.pop_back();
			TopTools_ListOfShape cF = vertexToFaceMap.FindFromKey(currentVertex);
			FacesSet currentFaces;
			for (auto f : cF)
			{
				currentFaces.insert(TopoDS::Face(f));
			}

			Matrix3d normalTensorSum;
			normalTensorSum << 0, 0, 0,
				0, 0, 0,
				0, 0, 0;
			Vector3d normalSum(0, 0, 0);
			CalcNormalTensor(currentFaces, currentVertex, normalTensorSum, normalSum);

			EigenSolver<Matrix3d> normalTensorSolver(normalTensorSum, true);
			Matrix3d eigenVectors = normalTensorSolver.eigenvectors().real();
			Vector3d eigenValues = normalTensorSolver.eigenvalues().real();

			std::sort(eigenValues.data(), eigenValues.data() + eigenValues.size());
			eigenValues.reverseInPlace();
			//compute vertex classification

			Vector3d vertexClassification(eigenValues.x() - eigenValues.y(),
				creaseParameter * (eigenValues.y() - eigenValues.z()),
				creaseParameter * eigenValues.z());

			Vector3d::Index vertexClass;
			vertexClassification.maxCoeff(&vertexClass);

			if (vertexClass > 0)
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
					for (faceToVertices.Init(face, TopAbs_VERTEX); faceToVertices.More() && i < 2; faceToVertices.Next())
					{
						auto v = TopoDS::Vertex(faceToVertices.Current());
						if (!currentVertex.IsSame(v) && !IsVertexInQueue(vertexQueue, v))
						{
							vertexQueue.push_back(v);

							vertices.erase(v);
							i++;
						}
					}

				}
				else
				{

				}
				features.back().AddEdgeVertex(currentVertex, (EdgeVertexType)vertexClass);

			}
			else
			{
				int n = currentFaces.size();
				for (auto face : currentFaces)
				{
					features.back().AddFace(face);

					numProcessed++;
					TopExp_Explorer faceToVertices;
					int i = 0;
					for (faceToVertices.Init(face, TopAbs_VERTEX); faceToVertices.More() && i < 2; faceToVertices.Next())
					{
						auto v = TopoDS::Vertex(faceToVertices.Current());
						auto cv = BRep_Tool::Pnt(v);

						if (!features.back().ContainsVertex(v) && !IsVertexInQueue(vertexQueue, v))
						{
							vertexQueue.push_back(v);

							vertices.erase(v);
							i++;
						}

					}
				}
			}
			features.back().AddVertex(currentVertex);

			if (vertexQueue.empty() && features.NumFaces() < numFaces)
			{
				ExtractedFeature newFeature;
				features.push_back(newFeature);
				vertexQueue.push_back(*(vertices.rbegin()));
				vertices.erase(*(vertices.rbegin()));
			}

		}
		features.ProcessEdges();
		return features;
	}

	ExtractedFeatures HybridEdgewiseNormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle /*= 5.0f*/)
	{
		freopen("output.txt", "w", stdout);
		TopTools_IndexedDataMapOfShapeListOfShape faceToVertexMap;
		TopExp::MapShapesAndAncestors(model, TopAbs_FACE, TopAbs_VERTEX, faceToVertexMap);
		TopTools_IndexedDataMapOfShapeListOfShape faceToEdgeMap;
		TopExp::MapShapesAndAncestors(model, TopAbs_FACE, TopAbs_EDGE, faceToEdgeMap);
		TopTools_IndexedDataMapOfShapeListOfShape vertexToFaceMap;
		TopExp::MapShapesAndAncestors(model, TopAbs_VERTEX, TopAbs_FACE, vertexToFaceMap);
		TopTools_IndexedDataMapOfShapeListOfShape edgeToFaceMap;
		TopExp::MapShapesAndAncestors(model, TopAbs_EDGE, TopAbs_FACE, edgeToFaceMap);
		double creaseParameter = 5;// 1 / (tan(0.5*creaseAngle)*tan(0.5*creaseAngle)) - 1;
		TopExp_Explorer topExp;
		FacesMap<int> faces;

		for (topExp.Init(model, TopAbs_FACE); topExp.More(); topExp.Next())
		{
			faces.insert({ TopoDS::Face(topExp.Current()), 0 });
		}

		map<TopoDS_Edge, bool, Shape_Compare> edgeMap;

		double cosCreaseAngle = cos(creaseAngle*M_PI / 180);

		for (topExp.Init(model, TopAbs_EDGE); topExp.More(); topExp.Next())
		{
			auto edge = TopoDS::Edge(topExp.Current());

			auto faces = edgeToFaceMap.FindFromKey(edge);

			if (faces.Size() != 2)
			{
				throw; //Shouldn't be possible
			}

			auto n1 = FaceNormal(TopoDS::Face(faces.First()));
			auto n2 = FaceNormal(TopoDS::Face(faces.Last()));
			n1.normalize();
			n2.normalize();
			auto dotProduct = n1.dot(n2);

			bool val = dotProduct < cosCreaseAngle;

			edgeMap.insert({ edge,val });
		}

		VerticesTypeMap vertMap;

		for (topExp.Init(model, TopAbs_VERTEX); topExp.More(); topExp.Next())
		{
			auto v = TopoDS::Vertex(topExp.Current());
			if (vertMap.count(v) == 1)
			{
				continue;
			}
			TopTools_ListOfShape cF = vertexToFaceMap.FindFromKey(v);
			FacesSet currentFaces;
			for (auto f : cF)
			{
				currentFaces.insert(TopoDS::Face(f));
			}
			Matrix3d normalTensorSum;
			normalTensorSum << 0, 0, 0,
				0, 0, 0,
				0, 0, 0;
			Vector3d normalSum(0, 0, 0);
			auto x = converter(v);
			CalcNormalTensor(currentFaces, v, normalTensorSum, normalSum);

			EigenSolver<Matrix3d> normalTensorSolver(normalTensorSum, true);
			Matrix3d eigenVectors = normalTensorSolver.eigenvectors().real();
			Vector3d eigenValues = normalTensorSolver.eigenvalues().real();
			vector<pair<double, Vector3d>> eigenResults;
			for (size_t i = 0; i < 3; i++)
			{
				eigenResults.push_back({ eigenValues[i],eigenVectors.col(i) });
			}

			std::sort(eigenResults.begin(), eigenResults.end(), [](pair<double, Vector3d> &left, pair<double, Vector3d> &right) {
				return left.first > right.first;
			});
			Vector3d vertexClassification(eigenResults[0].first - eigenResults[1].first,
				creaseParameter * (eigenResults[1].first - eigenResults[2].first),
				creaseParameter * eigenResults[2].first);
			Vector3d::Index vertexClass;
			vertexClassification.maxCoeff(&vertexClass);
			vertMap[v] = (EdgeVertexType)vertexClass;
			
		}


		int numFaces = faces.size();
		int numProcessed = 0;

		ExtractedFeatures result;
		//result.push_back(ExtractedFeature());

		stack<TopoDS_Face> faceQueue;
		//faceQueue.push(faces.begin()->first);
		//faces[faceQueue.top()] = 1;
		while (result.NumFaces() < numFaces)
		{
			if (faceQueue.empty())
			{
				result.push_back(ExtractedFeature());
				for (auto kp : faces)
				{
					if (kp.second == 0)
					{
						faceQueue.push(kp.first);
						faces[kp.first] = 1;
						break;
					}
				}
			}

			auto currentFace = faceQueue.top();
			faceQueue.pop();


			//auto cV = faceToVertexMap.FindFromKey(currentFace);
			VerticesSet currentVertices;
			for (topExp.Init(currentFace, TopAbs_VERTEX); topExp.More(); topExp.Next())
			{
				currentVertices.insert(TopoDS::Vertex(topExp.Current()));
			}

			result.ExpandFeature(currentFace, currentVertices.begin(), currentVertices.end());
			faces[currentFace] = 2;
			vector<vector<TopoDS_Vertex>> classificationResults = { vector<TopoDS_Vertex>(), vector<TopoDS_Vertex>(), vector<TopoDS_Vertex>() };

			for (auto v : currentVertices)
			{
				classificationResults[vertMap[v]].push_back(v);
			}
				TopTools_IndexedDataMapOfShapeListOfShape vertexToEdgeMap;
				TopExp::MapShapesAndAncestors(currentFace, TopAbs_VERTEX, TopAbs_EDGE, vertexToEdgeMap);
			if (classificationResults[1].size() > 1 || classificationResults[2].size() > 1)
			{
#pragma region Crease-crease and corner-corner edge expansion			
				for (size_t i = 1; i <= 2; i++)
				{
					if (classificationResults[i].size() < 2)
					{
						continue;
					}
					int limit;
					if (classificationResults[i].size() == 2)
					{
						limit = 1;
					}
					else
					{
						limit = 3;
					}
					for (size_t j = 0; j < limit; j++)
					{
						auto v = classificationResults[i][j];
						auto v2 = classificationResults[i][(j + 1) % 3];
						for (topExp.Init(currentFace, TopAbs_EDGE); topExp.More(); topExp.Next())
						{
							auto e = TopoDS::Edge(topExp.Current());
							TopoDS_Vertex first;
							TopoDS_Vertex second;
							TopExp::Vertices(e, first, second);
							if (((v.IsSame(first) && v2.IsSame(second)) || (v2.IsSame(first) && v.IsSame(second))) && !edgeMap[e])
							{
								auto edgeFaces = edgeToFaceMap.FindFromKey(e);
								for (auto f : edgeFaces)
								{
									auto face = TopoDS::Face(f);
									if (faces[face] == 0)
									{
										faceQueue.push(face);
										faces[face] = 1;
										break;
									}
								}
								break;
							}
						}
					}
				}
#pragma endregion
			}
#pragma region Crease-corner edge expansion
				for (auto v : classificationResults[1])
				{
					auto currentEdges = vertexToEdgeMap.FindFromKey(v);
					for (auto v2 : classificationResults[2])
					{
						for (topExp.Init(currentFace, TopAbs_EDGE); topExp.More(); topExp.Next())
						{
							auto e = TopoDS::Edge(topExp.Current());
							TopoDS_Vertex first;
							TopoDS_Vertex second;
							TopExp::Vertices(e, first, second);
							if (((v.IsSame(first) && v2.IsSame(second)) || (v2.IsSame(first) && v.IsSame(second))) && !edgeMap[e])
							{
								auto edgeFaces = edgeToFaceMap.FindFromKey(e);
								for (auto f : edgeFaces)
								{
									auto face = TopoDS::Face(f);
									if (faces[face] == 0)
									{
										faceQueue.push(face);
										faces[face] = 1;
										break;
									}
								}
								break;
							}
						}
					}
				}
#pragma endregion
			
			for (size_t i = 1; i < 3; i++)
			{
				result.AddEdgeVertices(classificationResults[i].begin(), classificationResults[i].end(), (EdgeVertexType)i);
			}


#pragma region Planar vertex expansion
			for (auto v : classificationResults[0])
			{
				TopTools_ListOfShape currentFaces = vertexToFaceMap.FindFromKey(v);
				for (auto x : currentFaces)
				{
					auto f = TopoDS::Face(x);
					if (faces[f] == 0)
					{
						faceQueue.push(f);
						faces[f] = 1;
					}
				}
			}
#pragma endregion
			
		}
		result.ProcessEdges();
		return result;
	}

	FeatureExtractionAlgo::ExtractedFeatures EdgewiseMethod(TopoDS_Shape model, float creaseAngle /*= 5.0f*/)
	{
		TopTools_IndexedDataMapOfShapeListOfShape vertexToEdgeMap;
		TopExp::MapShapesAndAncestors(model, TopAbs_VERTEX,TopAbs_EDGE, vertexToEdgeMap);
		TopTools_IndexedDataMapOfShapeListOfShape edgeToFaceMap;
		TopExp::MapShapesAndAncestors(model, TopAbs_EDGE, TopAbs_FACE, edgeToFaceMap);
		TopExp_Explorer topExp;

		map<TopoDS_Edge, bool, Shape_Compare> edgeMap;

		double cosCreaseAngle = cos(creaseAngle*M_PI/180);

		for (topExp.Init(model, TopAbs_EDGE); topExp.More();topExp.Next())
		{
			auto edge = TopoDS::Edge(topExp.Current());
			
			auto faces = edgeToFaceMap.FindFromKey(edge);

			if (faces.Size() != 2)
			{
				throw; //Shouldn't be possible
			}

			auto n1 = FaceNormal(TopoDS::Face(faces.First()));
			auto n2 = FaceNormal(TopoDS::Face(faces.Last()));
			n1.normalize();
			n2.normalize();
			auto dotProduct = n1.dot(n2);

			bool val = dotProduct < cosCreaseAngle;

			edgeMap.insert({ edge,val });
		}

		VerticesTypeMap vertMap;

		for (topExp.Init(model, TopAbs_VERTEX); topExp.More(); topExp.Next())
		{
			auto v = TopoDS::Vertex(topExp.Current());
			auto cE = vertexToEdgeMap.FindFromKey(v);
			EdgesSet currentEdges;
			for (auto e : cE)
			{
				currentEdges.insert(TopoDS::Edge(e));
			}

			int numCreases = 0;
			for (auto e : currentEdges)
			{
				if (edgeMap[e])
				{
					numCreases++;
				}
			}

			EdgeVertexType cat;
			switch (numCreases)
			{
			case 0:
				cat = PLANAR;
				break;
			case 2:
				cat = CREASE;
				break;
			default:
				cat = CORNER;
				break;
			}

			vertMap[v] = cat;
		}

		FacesMap<int> faces;
		for (topExp.Init(model, TopAbs_FACE); topExp.More(); topExp.Next())
		{
			faces.insert({ TopoDS::Face(topExp.Current()), 0 });
		}

		int numFaces = faces.size();

		ExtractedFeatures result;
		stack<TopoDS_Face> faceQueue;
		while (result.NumFaces() < numFaces)
		{
			if (faceQueue.empty())
			{
				result.push_back(ExtractedFeature());
				for (auto kp : faces)
				{
					if (kp.second == 0)
					{
						faceQueue.push(kp.first);
						faces[kp.first] = 1;
						break;
					}
				}
			}

			auto currentFace = faceQueue.top();
			faceQueue.pop();

			for (topExp.Init(currentFace, TopAbs_EDGE); topExp.More(); topExp.Next())
			{
				auto currentEdge = TopoDS::Edge(topExp.Current());
				if (!edgeMap[currentEdge])
				{
					auto edgeFaces = edgeToFaceMap.FindFromKey(currentEdge);
					for (auto f:edgeFaces)
					{
						if (!currentFace.IsSame(f) && faces[TopoDS::Face(f)]==0)
						{
							faceQueue.push(TopoDS::Face(f));
							faces[TopoDS::Face(f)] = 1;
						}
					}
				}
			}

			VerticesSet currentVertices;
			for (topExp.Init(currentFace, TopAbs_VERTEX); topExp.More(); topExp.Next())
			{
				currentVertices.insert(TopoDS::Vertex(topExp.Current()));
			}
			result.ExpandFeature(currentFace, currentVertices.begin(), currentVertices.end());
			faces[currentFace] = 2;

			for (auto v:currentVertices)
			{
				if (vertMap[v] != PLANAR)
				{
					result.AddEdgeVertex(v, vertMap[v]);
				}
			}
		}
		result.ProcessEdges();
		return result;
	}

	double FaceNormalWeightN(TopoDS_Face currentFace, TopoDS_Vertex currentVertex)
	{
		return 1;
	}

	double FaceNormalWeightMWA(TopoDS_Face currentFace, TopoDS_Vertex currentVertex)
	{
		TopTools_IndexedDataMapOfShapeListOfShape vertexToEdgeMap;
		TopExp::MapShapesAndAncestors(currentFace, TopAbs_VERTEX, TopAbs_EDGE, vertexToEdgeMap);
		auto currentEdges = vertexToEdgeMap.FindFromKey(currentVertex);
		TopoDS_Edge edge1 = TopoDS::Edge(currentEdges.First());
		auto v11 = BRep_Tool::Pnt(TopExp::FirstVertex(edge1));
		auto v12 = BRep_Tool::Pnt(TopExp::LastVertex(edge1));
		Vector3d e1(v12.X() - v11.X(), v12.Y() - v11.Y(), v12.Z() - v11.Z());

		TopoDS_Edge edge2 = TopoDS::Edge(currentEdges.Last());
		auto v21 = BRep_Tool::Pnt(TopExp::FirstVertex(edge2));
		auto v22 = BRep_Tool::Pnt(TopExp::LastVertex(edge2));
		Vector3d e2(v22.X() - v21.X(), v22.Y() - v21.Y(), v22.Z() - v21.Z());

		if (!TopExp::FirstVertex(edge1).IsSame(currentVertex))
			e1 *= -1;
		if (!TopExp::FirstVertex(edge2).IsSame(currentVertex))
			e2 *= -1;
		//Vector3d crossProduct = e1.cross(e2);

		e1.normalize();
		e2.normalize();

		double angle = acos(e1.dot(e2));

		double currentWeight = angle;// / (e1.dot(e1)*e2.dot(e2));
		return currentWeight;
	}

	double FaceNormalWeightMWAAT(TopoDS_Face currentFace, TopoDS_Vertex currentVertex)
	{
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

		double currentWeight = crossProduct.norm();// / (e1.dot(e1)*e2.dot(e2));
		return currentWeight;
	}

	double FaceNormalWeightMWSLER(TopoDS_Face currentFace, TopoDS_Vertex currentVertex)
	{
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

		double currentWeight = crossProduct.norm() / (e1.dot(e1)*e2.dot(e2));
		return currentWeight;
	}

	Vector3d FaceNormal(TopoDS_Face face)
	{
		TopoDS_Face currentFace = face;

		Standard_Real umin, umax, vmin, vmax;
		BRepTools::UVBounds(currentFace, umin, umax, vmin, vmax);

		auto currentSurface = BRep_Tool::Surface(currentFace);

		GeomLProp_SLProps props(currentSurface, umin, vmin, 1, 0.01);
		gp_Dir normal = props.Normal();

		return Vector3d(normal.X(), normal.Y(), normal.Z());
	}

	template<typename List>
	void CalcNormalTensor(List &currentFaces, TopoDS_Vertex v, Matrix3d &normalTensorSum, Vector3d &normalSum)
	{
		for (auto f: currentFaces)
		{
			auto currentNormal = FaceNormal(f);
			auto currentWeight = FaceNormalWeightMWA(f, v);
			normalTensorSum += currentWeight * currentNormal * currentNormal.transpose();
			normalSum += currentWeight * currentNormal;
		}
		normalSum.normalize();
	}

	bool IsVertexInQueue(vector<TopoDS_Vertex> list, TopoDS_Vertex vertex)
	{
		for (size_t i = 0; i < list.size(); i++)
		{
			if (vertex.IsSame(list[i]))
			{
				return true;
			}
		}
		return false;
	}

	int FindVertex(vector<TopoDS_Vertex> list, TopoDS_Vertex vertex)
	{
		int i;
		for (i = 0; i < list.size(); i++)
		{
			if (vertex.IsSame(list[i]))
			{
				return i;
			}
		}
		return -1;
	}

	bool ExtractedFeatures::IsFaceProcessed(TopoDS_Face face)
	{
		for (size_t i = 0; i < this->size(); i++)
		{
			auto currentFeature = this->at(i);
			if (currentFeature.ContainsFace(face))
				return true;
		}
		return false;
	}

	bool ExtractedFeatures::IsVertexProcessed(TopoDS_Vertex vertex)
	{
		for (size_t i = 0; i < this->size(); i++)
		{
			auto currentFeature = this->at(i);
			if (currentFeature.ContainsVertex(vertex))
				return true;
		}
		return false;
	}

	int ExtractedFeatures::NumFaces()
	{
		int result = 0;
		for (size_t i = 0; i < this->size(); i++)
		{
			result += this->at(i).NumFaces();
		}
		return result;
	}

	void ExtractedFeatures::ProcessEdges()
{
		for (size_t i = 0; i < size(); i++)
		{
			at(i).ProcessEdges();
		}
	}




	bool ExtractedFeature::ContainsFace(TopoDS_Face face)
	{
		for (size_t i = 0; i < faces.size(); i++)
		{
			if (face.IsSame(faces[i]))
				return true;
		}
		return false;
	}

	bool ExtractedFeature::ContainsVertex(TopoDS_Vertex vertex)
	{
		for (auto v:vertices)
		{
			if (vertex.IsSame(v.first))
				return true;
		}
		return false;
	}

	TopoDS_Vertex ExtractedFeature::GetNearbyEdgeVertex(TopoDS_Shape shape, TopoDS_Vertex vertex)
	{
		return vertex;
	}

	void ExtractedFeature::ProcessEdges()
	{
		if (edgeVertices.size()==0)
		{
			return;
		}

		TopoDS_Compound shape;
		TopoDS_Builder builder;
		builder.MakeCompound(shape);
		for (size_t i = 0; i < NumFaces(); i++)
		{
			builder.Add(shape, GetFace(i));
		}
		TopTools_IndexedDataMapOfShapeListOfShape vertexToEdgeMap;
		TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, vertexToEdgeMap);
		TopTools_IndexedDataMapOfShapeListOfShape edgeToFaceMap;
		TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeToFaceMap);

		map<TopoDS_Vertex, pair<EdgeVertexType, int>,Shape_Compare> edgeVertexMap;
		map<int, ExtractedFeatureEdge> edgeMap;
		queue<int> edgeQueue;
		int cornerCount = 0;

		for (auto e:edgeVertices)
		{
			edgeVertexMap.insert({ e.first,{e.second,-1} });
			if (e.second==CORNER)
			{
				cornerCount++;
			}
		}

		int numProcessed = 0;
		
		while (numProcessed<edgeVertices.size()||!edgeQueue.empty())
		{
#pragma region Queue populate
			if (edgeQueue.empty())
			{
				bool posFirstCorner = false;// = FindCornerVertex();
				EdgeType type;
				TopoDS_Vertex vertex;
				for (auto p : edgeVertexMap)
				{
					if (p.second.first == CORNER && p.second.second ==-1)
					{
						posFirstCorner = true;
						vertex = p.first;
						type = FINITE;
						break;
					}
				}
				
				
				if (!posFirstCorner)
				{
					//vertex = edgeVertices[0];
					for (auto e : edgeVertexMap)
					{
						if (e.second.second==-1)
						{
							vertex = e.first;
							break;
						}
					}
					type = CONTINUOUS;
				}
				//else
				//{
				//	vertex = edgeVertices[posFirstCorner];
				//	type = FINITE;
				//}
				
				//verticesToProcess
				vector<TopoDS_Vertex> verticesToProcess = { vertex };
				//while there are vertices to process
				while (verticesToProcess.size()>0)
				{
					auto vertex = verticesToProcess.back();
					verticesToProcess.pop_back();
					auto edges = vertexToEdgeMap.FindFromKey(vertex);
					numProcessed++;
					for (auto e : edges)
					{
						auto edge = TopoDS::Edge(e);
						auto faces = edgeToFaceMap.FindFromKey(edge);
						if (faces.Size() == 1)
						{
							//if corner add to vertices to process

							CreateEdge(vertex, edge, type, edgeVertexMap, edgeMap, edgeQueue, numProcessed, verticesToProcess);
						}
					}
				}
				if (edgeQueue.empty())
				{
					continue;
				}
			}
#pragma endregion
			auto queueNum = edgeQueue.front();
			if (edgeMap.count(queueNum)==0)
			{
				edgeQueue.pop();
				continue;
			}
			auto &currentEdge = edgeMap[queueNum];
			auto currentVertex = currentEdge.back();
			auto edges = vertexToEdgeMap.FindFromKey(currentVertex);
			for (auto e:edges)
			{
				auto edge = TopoDS::Edge(e);
				auto faces = edgeToFaceMap.FindFromKey(edge);
				if (faces.Size() == 1)
				{
					TopoDS_Vertex first, second, v;
					TopExp::Vertices(edge, first, second);
					if (currentVertex.IsSame(first))
					{
						v = second;
					}
					else
					{
						v = first;
					}
					auto prevVertex = currentEdge[currentEdge.size() - 2];
					if (v.IsSame(prevVertex))//If the edge is going backwards along the extracted edge
					{
						continue;
					}
					if (currentEdge.Type() == CONTINUOUS && v.IsSame(currentEdge.front()))//If the edge completes a loop
					{
						edgeQueue.pop();
						break;
					}
					if (edgeVertexMap[v].second!=-1 && edgeVertexMap[v].first == CREASE) //Next vertex is already in an extracted edge
					{
						int edgePos = edgeVertexMap[v].second;
						auto oldEdge = edgeMap[edgePos];
						for (auto ite = oldEdge.rbegin(); ite != oldEdge.rend(); ite++)
						{
							auto ov = *ite;
							currentEdge.AddVertex(ov);
							edgeVertexMap[ov].second = queueNum;
						}
						edgeMap.erase(edgePos);
						edgeQueue.pop();
						break;
					}
					if (edgeVertexMap[v].first==CORNER)
					{
						vector<TopoDS_Vertex> verticesToProcess = { v };
						while (verticesToProcess.size()>0)
						{
							auto v = verticesToProcess.back();
							verticesToProcess.pop_back();
							auto edges = vertexToEdgeMap.FindFromKey(v);
							//numProcessed++;
							for (auto e : edges)
							{
								auto edge = TopoDS::Edge(e);
								auto faces = edgeToFaceMap.FindFromKey(edge);
								if (faces.Size() == 1)
								{
									CreateEdge(v, edge, FINITE, edgeVertexMap, edgeMap, edgeQueue, numProcessed, verticesToProcess);
								}
							}
						}
						edgeQueue.pop();
					}
					currentEdge.AddVertex(v);
					edgeVertexMap[v].second = queueNum; 
					numProcessed++;
					break;
				}
			}
		}
		for (auto e : edgeMap)
		{
			extractedEdges.push_back(e.second);
		}
		ProcessEdgeGroups();
	}

	void ExtractedFeature::CreateEdge(TopoDS_Vertex vertex, TopoDS_Edge edge, EdgeType type, map<TopoDS_Vertex, pair<EdgeVertexType, int>, Shape_Compare> &edgeVertexMap, map<int, ExtractedFeatureEdge> &edgeMap, queue<int> &edgeQueue, int &numProcessed, vector<TopoDS_Vertex> &verticesToProcess) {
		
		if (edgeVertexMap[vertex].second != -1 && type == CONTINUOUS)
			return;
		TopoDS_Vertex first, second, v;
		TopExp::Vertices(edge, first, second);
		if (vertex.IsSame(first))
		{
			v = second;
		}
		else
		{
			v = first;
		}
		
		if (edgeVertexMap[v].second == -1 || FindVertex(verticesToProcess,v)>-1)
		{
			ExtractedFeatureEdge newEdge;
			newEdge.Type(type);
			newEdge.AddVertex(vertex);
			newEdge.AddVertex(v);
			edgeVertexMap[vertex].second = edgeMap.size();
			edgeVertexMap[v].second = edgeMap.size();
			if (edgeVertexMap[v].first!=CORNER)
			{
				edgeQueue.push(edgeMap.size());
			}
			else
			{
				verticesToProcess.push_back(v);
			}
			edgeMap.insert({ edgeMap.size(),newEdge });
			numProcessed++;
		}
	}

	void ExtractedFeature::ProcessEdgeGroups()
	{
		int numEdgeGroups = 0;
		int edgesProcessed = 0;
		
		auto edgeList = vector<ExtractedFeatureEdge>(extractedEdges);
		for (auto ite = edgeList.begin(); ite != end(edgeList);)
		{
			auto edge = *ite;
			if (edge.Type() == CONTINUOUS)
			{
				numEdgeGroups++;
				edgesProcessed++;
				EdgeGroup newEdge;
				newEdge.push_back(edge);
				edgeGroups.push_back(newEdge);
				ite = edgeList.erase(ite);
			}
			else
				++ite;
		}

		while (edgesProcessed < extractedEdges.size())
		{
			auto edge = edgeList.back();
			edgeList.pop_back();
			EdgeGroup newEdge;
			newEdge.push_back(edge);
			edgeGroups.push_back({ newEdge });
			edgesProcessed++;
			numEdgeGroups++;
			TopoDS_Vertex startVertex, endVertex;
			startVertex = edge.front();
			endVertex = edge.back();
			for (auto ite = edgeList.begin(); ite != end(edgeList);)
			{
				auto currentEdge = *ite;
				if (endVertex.IsSame(currentEdge.front()) || endVertex.IsSame(currentEdge.back()))
				{
					if (endVertex.IsSame(currentEdge.front()))
					{
						endVertex = currentEdge.back();
					}
					else
						endVertex = currentEdge.front();
					edgeGroups.back().push_back(currentEdge);
					ite = edgeList.erase(ite);
					ite = edgeList.begin();
					edgesProcessed++;
					if (startVertex.IsSame(currentEdge.back())|| startVertex.IsSame(currentEdge.front()))
					{
						break;
					}
				}
				else
					++ite;
			}
		}

		return;
	}

	void ExtractedFeature::GetOuterEdgeGroup(int &index)
	{
		auto verts = GetVertices();
		MatrixXd V(verts.size(), 3);
		for (size_t i = 0; i < verts.size(); i++)
		{
			V.row(i) = converter(verts[i]);
		}
		Vector3d centroid = V.colwise().mean();
		V.rowwise() -= centroid.transpose();
		VectorXd dist2 = V.rowwise().squaredNorm();//V*V.transpose();
		
		int maxDist2;
		dist2.maxCoeff(&maxDist2);

		auto maxVertex = verts[maxDist2];

		edgeGroups.FindEdgeGroup(maxVertex,index);
	}

	const vector<TopoDS_Vertex> ExtractedFeature::GetVertices()
	{
		vector<TopoDS_Vertex> result;
		result.reserve(vertices.size());
		for (auto v:vertices)
		{
			result.push_back(v.first);
		}
		return result;
	}



	vector<ExtractedFeatureEdge>& EdgeGroups::FindEdgeGroup(TopoDS_Vertex v, int &index)
	{
		index = 0;
		for (auto eg : *this)
		{
			for (auto e:eg)
			{
				for (auto vert:e)
				{
					if (v.IsSame(vert))
					{
						return eg;
					}
				}
			}
			index++;
		}
	}

}