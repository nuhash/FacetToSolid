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
#include <map>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>

#include <GeomLProp_SLProps.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopExp.hxx>

using namespace std;
using namespace Eigen;
namespace FeatureExtractionAlgo
{
//public:
	struct ConversionHelpers {
		Vector3d operator()(const TopoDS_Vertex& v) const
		{
			auto p = BRep_Tool::Pnt(v);
			return Vector3d(p.X(), p.Y(), p.Z());
		}


		
	};

	Vector3d operator|(const TopoDS_Vertex v, ConversionHelpers c);
	const ConversionHelpers converter;
	struct Shape_Compare {
		bool operator() (const TopoDS_Shape& lhs, const TopoDS_Shape& rhs) const {
			return lhs.HashCode(1<<30)<rhs.HashCode(1<<30);
		}
	};

	class VerticesSet : public set<TopoDS_Vertex, Shape_Compare>
	{

	};

	class FacesMap : public std::map<TopoDS_Face, int, Shape_Compare>
	{

	};

	class FacesSet : public std::set<TopoDS_Face, Shape_Compare> {};

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
		void AddFace(TopoDS_Face face)
		{
			if (!ContainsFace(face))
				faces.push_back(face);
		}
		void AddVertex(TopoDS_Vertex vertex) { vertices.push_back(vertex); }
		void AddEdgeVertex(TopoDS_Vertex vertex, EdgeVertexType type) { edgeVertices.push_back(vertex); edgeVerticesTypes.push_back(type); }
		int NumFaces() { return faces.size(); }
		bool ContainsFace(TopoDS_Face face);
		bool ContainsVertex(TopoDS_Vertex vertex);
		TopoDS_Face GetFace(int n) { return faces[n]; };
		TopoDS_Vertex GetNearbyEdgeVertex(TopoDS_Shape shape, TopoDS_Vertex vertex);
		void ProcessEdges();
		int FindCornerVertex();
		bool CreateNewEdge(vector<ExtractedFeatureEdge>& queue, TopTools_IndexedDataMapOfShapeListOfShape v2e, TopTools_IndexedDataMapOfShapeListOfShape e2f, TopoDS_Vertex &vertex, EdgeVertexType &type);
		int FindEdgeVertex(TopoDS_Vertex vertex);
		bool IsVertexEdge(TopoDS_Vertex vertex);
		bool IsVertexEdgeProcessed(TopoDS_Vertex vertex);
		const vector<TopoDS_Face> GetFaces() { return faces; }
		const vector<TopoDS_Vertex> GetVertices() { return vertices; }
		int NumEdges() { return extractedEdges.size(); }
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
		template<typename Iter>
		void ExpandFeature(TopoDS_Face face, Iter begin, Iter end)
		{
			this->back().AddFace(face);
			while (begin!=end)
			{
				this->back().AddVertex(*begin);
				begin++;
			}
		}
		template<typename Iter>
		void AddEdgeVertices(Iter begin, Iter end, EdgeVertexType type)
		{
			while (begin!=end)
			{
				this->back().AddEdgeVertex(*begin, type);
				begin++;
			}
		}
	protected:
	private:
	};

	//static double NormalWeight(TopoDS_Face face, TopoDS_Vertex vertex);
	ExtractedFeatures NormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle = 5.0f);
	ExtractedFeatures EdgewiseNormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle = 5.0f);

	double FaceNormalWeightMWSLER(TopoDS_Face currentFace, TopoDS_Vertex currentVertex);
	
	Vector3d FaceNormal(TopoDS_Face face);

	template<typename List>
	void CalcNormalTensor(List &currentFaces, TopoDS_Vertex v, Matrix3d &normalTensorSum, Vector3d &normalSum);

	bool IsVertexInQueue(vector<TopoDS_Vertex> list, TopoDS_Vertex vertex);

	int FindVertex(vector<TopoDS_Vertex> list, TopoDS_Vertex vertex);

};