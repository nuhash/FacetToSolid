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
		while (features.NumFaces() < numFaces || !vertexQueue.empty())
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

				Vector3d currentNormal = FaceNormal(currentFace);

				double currentWeight = FaceNormalWeightMWSLER(currentFace, currentVertex);

				auto inter = currentNormal * currentNormal.transpose();
				if (true)
				{
					cout << currentNormal.x() << ";" << currentNormal.y() << ";" << currentNormal.z() << ";" << currentWeight << "; \n";

					cout << inter << "\n\n";
				}


				normalTensorSum += currentWeight * inter;
				normalSum += currentWeight * currentNormal;
				ite++;
			}


			//compute eigen values, vectors

			EigenSolver<Matrix3d> normalTensorSolver(normalTensorSum, true);
			Matrix3d eigenVectors = normalTensorSolver.eigenvectors().real();
			Vector3d eigenValues = normalTensorSolver.eigenvalues().real();
			vector<pair<double, Vector3d>> eigenResults;
			for (size_t i = 0; i < 3; i++)
			{
				eigenResults.push_back({ eigenValues[i],eigenVectors.col(i) });
			}
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
				cout << eigenVectors << "\n";
				cout << vertexClassification.x() << ";" << vertexClassification.y() << ";" << vertexClassification.z() << "\n\n";
				cout << "End surface check\n\n";
			}
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
				features.back().AddEdgeVertex(currentVertex, (EdgeVertexType)vertexClass);
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
					for (faceToVertices.Init(face, TopAbs_VERTEX); faceToVertices.More() && i < 2; faceToVertices.Next())
					{
						auto v = TopoDS::Vertex(faceToVertices.Current());
						auto cv = BRep_Tool::Pnt(v);
						cout << cv.X() << ";" << cv.Y() << ";" << cv.Z() << ";\n";
						if (!features.back().ContainsVertex(v) && !IsVertexInQueue(vertexQueue, v))
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

			if (vertexQueue.empty() && features.NumFaces() < numFaces)
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

	ExtractedFeatures EdgewiseNormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle /*= 5.0f*/)
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
		FacesMap faces;

		for (topExp.Init(model, TopAbs_FACE); topExp.More(); topExp.Next())
		{
			faces.insert({ TopoDS::Face(topExp.Current()), 0 });
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


			auto cV = faceToVertexMap.FindFromKey(currentFace);
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
				//auto v = TopoDS::Vertex(*ite);
				TopTools_ListOfShape cF = vertexToFaceMap.FindFromKey(v);
				FacesSet currentFaces;
				for (auto f: cF)
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
				if (x.z()==10 && (x.x()==0|| x.x() == 10|| x.y() == 0|| x.y() == 10))
				{
					cout << x << "\n\n";
					for (auto g : currentFaces)
					{
						auto f = TopoDS::Face(g);
						cout << FaceNormal(f) << "\n" << FaceNormalWeightMWSLER(f,v) << "\n\n";
					}
					
					cout << normalSum << "\n\n" << normalTensorSum << "\n\n\n";
				}
				

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
				classificationResults[vertexClass].push_back(v);
			}

			if (classificationResults[1].size() > 1 || classificationResults[2].size() > 1)
			{
				TopTools_IndexedDataMapOfShapeListOfShape vertexToEdgeMap;
				TopExp::MapShapesAndAncestors(currentFace, TopAbs_VERTEX, TopAbs_EDGE, vertexToEdgeMap);
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
						auto currentEdges = vertexToEdgeMap.FindFromKey(v);
						for (auto e : currentEdges)
						{
							TopoDS_Vertex first;
							TopoDS_Vertex second;
							TopExp::Vertices(TopoDS::Edge(e), first, second);
							if (v2.IsSame(first) || v2.IsSame(second))
							{
								auto edgeFaces = edgeToFaceMap.FindFromKey(e);
								auto face1 = TopoDS::Face(edgeFaces.First());
								auto face2 = TopoDS::Face(edgeFaces.Last());
								auto n1 = FaceNormal(face1);
								auto n2 = FaceNormal(face2);
								n1.normalize();
								n2.normalize();
								if (n1.dot(n2) > cos(creaseAngle*M_PI / 180))
								{
									//expand edge
									if (faces[face1] == 0)
									{
										faceQueue.push(face1);
										faces[face1] = 1;
									}
									else if (faces[face2] == 0)
									{
										faceQueue.push(face2);
										faces[face2] = 1;
									}
								}
								break;
							}
						}
					}
				}
#pragma endregion

#pragma region Crease-corner edge expansion
				for (auto v : classificationResults[1])
				{
					auto currentEdges = vertexToEdgeMap.FindFromKey(v);
					for (auto v2 : classificationResults[2])
					{
						for (auto e : currentEdges)
						{
							TopoDS_Vertex first;
							TopoDS_Vertex second;
							TopExp::Vertices(TopoDS::Edge(e), first, second);
							if (v2.IsSame(first) || v2.IsSame(second))
							{
								auto edgeFaces = edgeToFaceMap.FindFromKey(e);
								auto face1 = TopoDS::Face(edgeFaces.First());
								auto face2 = TopoDS::Face(edgeFaces.Last());
								auto n1 = FaceNormal(face1);
								auto n2 = FaceNormal(face2);
								n1.normalize();
								n2.normalize();
								if (n1.dot(n2) > cos(creaseAngle*M_PI / 180))
								{
									//expand edge
									if (faces[face1] == 0)
									{
										faceQueue.push(face1);
										faces[face1] = 1;
									}
									else if (faces[face2] == 0)
									{
										faceQueue.push(face2);
										faces[face2] = 1;
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

			}

			if (classificationResults[0].size()>0)
			{
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
		Vector3d e1(v11.X() - v12.X(), v11.Y() - v12.Y(), v11.Z() - v12.Z());

		TopoDS_Edge edge2 = TopoDS::Edge(currentEdges.Last());
		auto v21 = BRep_Tool::Pnt(TopExp::FirstVertex(edge2));
		auto v22 = BRep_Tool::Pnt(TopExp::LastVertex(edge2));
		Vector3d e2(v21.X() - v22.X(), v21.Y() - v22.Y(), v21.Z() - v22.Z());

		Vector3d crossProduct = e1.cross(e2);

		double angle = asin(crossProduct.norm() / (e1.norm()*e2.norm()));

		double currentWeight = crossProduct.norm();// / (e1.dot(e1)*e2.dot(e2));
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
		for (auto ite2 = currentFaces.begin(); ite2 != currentFaces.end(); ite2++)
		{
			auto f = TopoDS::Face(*ite2);
			auto currentNormal = FaceNormal(f);
			auto currentWeight = FaceNormalWeightMWSLER(f, v);
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
		for (size_t i = 0; i < vertices.size(); i++)
		{
			if (vertex.IsSame(vertices[i]))
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
		stack<int> edgeQueue;
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
		
		while (numProcessed<edgeVertices.size())
		{
#pragma region Queue populate
			if (edgeQueue.empty())
			{
				bool posFirstCorner = false;// = FindCornerVertex();
				EdgeType type;TopoDS_Vertex vertex;
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
			auto queueNum = edgeQueue.top();
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
					if (edgeVertexMap[v].second!=-1) //Next vertex is already in an extracted edge
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

	//int ExtractedFeature::FindCornerVertex()
	//{
	//	int i;
	//	for (i = 0; i < edgeVerticesTypes.size(); i++)
	//	{
	//		if (edgeVerticesTypes[i] == EdgeVertexType::CORNER)
	//		{
	//			return i;
	//		}
	//	}
	//	return -1; //No corner found, continuous edge/s
	//}

	void ExtractedFeature::CreateEdge(TopoDS_Vertex vertex, TopoDS_Edge edge, EdgeType type, map<TopoDS_Vertex, pair<EdgeVertexType, int>, Shape_Compare> &edgeVertexMap, map<int, ExtractedFeatureEdge> &edgeMap, stack<int> &edgeQueue, int &numProcessed, vector<TopoDS_Vertex> &verticesToProcess) {
		
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
				edgeGroups.push_back({ edge });
				ite = edgeList.erase(ite);
			}
			else
				++ite;
		}

		while (edgesProcessed < extractedEdges.size())
		{
			auto edge = edgeList.back();
			edgeList.pop_back();
			edgeGroups.push_back({ edge });
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

	//int ExtractedFeature::FindEdgeVertex(TopoDS_Vertex vertex)
	//{
	//	int i;
	//	for (i = 0; i < edgeVerticesTypes.size(); i++)
	//	{
	//		if (vertex.IsSame(edgeVertices[i]))
	//		{
	//			return i;
	//		}
	//	}
	//	return -1;
	//}

	//bool ExtractedFeature::IsVertexEdge(TopoDS_Vertex vertex)
	//{
	//	for (size_t i = 0; i < edgeVertices.size(); i++)
	//	{
	//		if (vertex.IsSame(edgeVertices[i]))
	//		{
	//			return true;
	//		}
	//	}
	//	return false;
	//}

	//bool ExtractedFeature::IsVertexEdgeProcessed(TopoDS_Vertex vertex)
	//{
	//	for (size_t i = 0; i < extractedEdges.size(); i++)
	//	{
	//		for (size_t j = 0; j < extractedEdges[i].size(); j++)
	//		{
	//			if (vertex.IsSame(extractedEdges[i][j]))
	//				return true;
	//		}
	//	}
	//	return false;
	//}
}