#pragma once
#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <vector>
#include <tuple>
#include <set>

using namespace std;
class FeatureExtractionAlgo
{
public:
	
	struct Shape_Compare {
		bool operator() (const TopoDS_Shape& lhs, const TopoDS_Shape& rhs) const {
			return lhs.HashCode(1<<30)<rhs.HashCode(1<<30);
		}
	};

	class VerticesSet : public set<TopoDS_Vertex, Shape_Compare>
	{

	};

	enum EdgeVertexType
	{
		CREASE = 1,
		CORNER = 2
	};
	enum EdgeType
	{
		CONTINUOUS,
		FINITE
	};
	class ExtractedFeatureEdge : public vector<TopoDS_Vertex>
	{
	public:
		FeatureExtractionAlgo::EdgeType Type() const { return type; }
		void AddVertex(TopoDS_Vertex vertex) { push_back(vertex); }
		void Type(FeatureExtractionAlgo::EdgeType val) { type = val; }
	protected:
	private:
		EdgeType type;
		vector<TopoDS_Vertex> edgeVertices;
	};

	class ExtractedFeature
	{
	public:
		void AddFace(TopoDS_Face face);
		void AddVertex(TopoDS_Vertex vertex) { vertices.push_back(vertex); }
		void AddEdgeVertex(TopoDS_Vertex vertex, EdgeVertexType type) { edgeVertices.push_back(vertex); edgeVerticesTypes.push_back(type); }
		int NumFaces() { return faces.size(); }
		bool ContainsFace(TopoDS_Face face);
		bool ContainsVertex(TopoDS_Vertex vertex);
		TopoDS_Face GetFace(int n) { return faces[n]; };
		TopoDS_Vertex GetNearbyEdgeVertex(TopoDS_Shape shape, TopoDS_Vertex vertex);
		void ProcessEdges(TopoDS_Shape shape);
		int FindCornerVertex();
		bool CreateNewEdge(vector<ExtractedFeatureEdge>& queue, TopTools_IndexedDataMapOfShapeListOfShape v2e, TopoDS_Vertex &vertex, EdgeVertexType &type);
		int FindEdgeVertex(TopoDS_Vertex vertex);
		bool IsVertexEdge(TopoDS_Vertex vertex);
		bool IsVertexEdgeProcessed(TopoDS_Vertex vertex);
	protected:
	private:
		vector<TopoDS_Face> faces;
		vector<TopoDS_Vertex> vertices;
		vector<TopoDS_Vertex> edgeVertices;
		vector<EdgeVertexType> edgeVerticesTypes;
		vector<ExtractedFeatureEdge> extractedEdges;
	};

	class ExtractedFeatures : public vector<ExtractedFeature>
	{
	public:
		bool IsFaceProcessed(TopoDS_Face face);
		bool IsVertexProcessed(TopoDS_Vertex vertex);
		int NumFaces();
		void ProcessEdges(TopoDS_Shape shape);
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

	static int FindVertex(vector<TopoDS_Vertex> list, TopoDS_Vertex vertex)
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
};