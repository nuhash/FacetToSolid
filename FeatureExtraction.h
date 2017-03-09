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
	static void NormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle = 5.0f);

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
		void AddFace(TopoDS_Face face) { faces.push_back(face); }
		void AddVertex(TopoDS_Vertex vertex) { vertices.push_back(vertex); }
		int NumFaces() { return faces.size(); }
		bool ContainsFace(TopoDS_Face face);
	protected:
	private:
		vector<TopoDS_Face> faces;
		vector<TopoDS_Vertex> vertices;
		vector<TopoDS_Vertex> edgeVertices;
	};

	class ExtractedFeatures : public vector<ExtractedFeature>
	{
	public:
		bool IsFaceProcessed(TopoDS_Face face);
	protected:
	private:
	};

	enum EdgeVertexType
	{
		CREASE,
		CORNER
	};



protected:

private:
};