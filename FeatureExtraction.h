#pragma once
#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <vector>
#include <tuple>

using namespace std;
class FeatureExtractionAlgo
{
public:
	
	enum EdgeVertexType
	{
		CREASE = 1,
		CORNER = 2
	};
	class ExtractedFeatureEdge
	{
	public:
	protected:
	private:
		vector<TopoDS_Vertex> edgeVertices;
	};

	class ExtractedFeature
	{
	public:
		void AddFace(TopoDS_Face face);
		void AddVertex(TopoDS_Vertex vertex) { vertices.push_back(vertex); }
		void AddEdgeVertex(TopoDS_Vertex vertex, EdgeVertexType type) { this->AddVertex(vertex); edgeVertices.push_back(vertex); edgeVerticesTypes.push_back(type); }
		int NumFaces() { return faces.size(); }
		bool ContainsFace(TopoDS_Face face);
		bool ContainsVertex(TopoDS_Vertex vertex);
		TopoDS_Face GetFace(int n) { return faces[n]; };
	protected:
	private:
		vector<TopoDS_Face> faces;
		vector<TopoDS_Vertex> vertices;
		vector<TopoDS_Vertex> edgeVertices;
		vector<EdgeVertexType> edgeVerticesTypes;
	};

	class ExtractedFeatures : public vector<ExtractedFeature>
	{
	public:
		bool IsFaceProcessed(TopoDS_Face face);
		bool IsVertexProcessed(TopoDS_Vertex vertex);
		int NumFaces();;
	protected:
	private:
	};

	static ExtractedFeatures NormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle = 5.0f);



protected:

private:
	static bool IsVertexInQueue(vector<TopoDS_Vertex> list, TopoDS_Vertex vertex)
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
};