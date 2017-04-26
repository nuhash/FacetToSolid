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
#include <stack>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>

#include <GeomLProp_SLProps.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopExp.hxx>
#include <queue>
#include <unordered_map>
#include "Utilities.h"

//#include "FeatureCategorisation.h"

using namespace std;
using namespace Eigen;
using namespace Utilities;
namespace FeatureExtractionAlgo
{
//public:

	enum EdgeVertexType
	{
		NONE = -1,
		PLANAR = 0,
		CREASE = 1,
		CORNER = 2
	};
	
	struct Shape_Compare {
		bool operator() (const TopoDS_Shape& lhs, const TopoDS_Shape& rhs) const {
			return lhs.HashCode(1<<30)<rhs.HashCode(1<<30);
		}
	};

	struct Shape_Hash {
		size_t operator() (const TopoDS_Shape& shape) const {
			return shape.HashCode(1 << 30);
		}
	};

	class VerticesSet : public set<TopoDS_Vertex, Shape_Compare>
	{

	};

	class VerticesTypeMap : public map<TopoDS_Vertex, EdgeVertexType, Shape_Compare>
	{

	};

	template<typename ValType>
	class VerticesMap : public map<TopoDS_Vertex, ValType, Shape_Compare>
	{

	};

	template <typename ValType>
	class FacesMap : public std::map<TopoDS_Face, ValType, Shape_Compare>
	{

	};

	class FacesSet : public std::set<TopoDS_Face, Shape_Compare> {};

	class EdgesSet :public set<TopoDS_Edge, Shape_Compare>
	{

	};
	enum EdgeType
	{
		CONTINUOUS,
		FINITE
	};

	enum EdgeVertexStatus
	{
		UNPROCESSED = 1,
		QUEUED = 2,
		PROCESSED = 4
	};
	class ExtractedFeatureEdge : public vector<TopoDS_Vertex>
	{
	public:
		FeatureExtractionAlgo::EdgeType Type() const { return type; }
		void AddVertex(TopoDS_Vertex vertex) { push_back(vertex); }
		void Type(FeatureExtractionAlgo::EdgeType val) { type = val; }
		const vector<TopoDS_Vertex> EdgeVertices() const { return edgeVertices; }
		size_t Hash()
		{
			size_t result;
			if (Type() == FINITE)
			{
				TopoDS_Vertex front;
				TopoDS_Vertex front2;
				TopoDS_Vertex back2;
				TopoDS_Vertex back;
				if (Shape_Compare().operator()(this->front(), this->back()))
				{
					front = this->front();
					back = this->back();
				}
				else
				{
					front = this->back();
					back = this->front();
				}
				if (Shape_Compare().operator()((*(this->begin()++)), (*(this->rbegin()++))))
				{
					front2 = (*(this->begin()++));
					back2 = (*(this->rbegin()++));
				}
				else
				{
					front2 = (*(this->rbegin()++));
					back2 = (*(this->begin()++));
				}
				size_t first = front.HashCode(0xFFFF);
				size_t second = front2.HashCode(0xFFFF);
				size_t secondLast = back2.HashCode(0xFFFF);
				size_t last = back.HashCode(0x7FFF);
				result = first | second << 16 | secondLast << 32 | last << 48;
			}
			else
			{
				size_t maxHash = 0;
				size_t minHash = 0xFFFFFFFFFFFFFFFF;
				for (auto v : *this)
				{
					auto currentHash = v.HashCode(0x7FFFFFFF);
					if (currentHash > maxHash)
					{
						maxHash = currentHash;
					}

					if (currentHash < minHash)
					{
						minHash = currentHash;
					}
				}
				result = maxHash | minHash << 32;
			}
			return result;
		}
	protected:
	private:
		EdgeType type;
		vector<TopoDS_Vertex> edgeVertices;
	};
	struct EdgeHash
	{
		size_t operator() (const ExtractedFeatureEdge &edge) const {
			size_t result;
			if (edge.Type()==FINITE)
			{
				TopoDS_Vertex front;
				TopoDS_Vertex front2;
				TopoDS_Vertex back2;
				TopoDS_Vertex back;
				if (Shape_Compare().operator()(edge.front(),edge.back()))
				{
					front = edge.front();
					back = edge.back();
				}
				else
				{
					front = edge.back();
					back = edge.front();
				}
				if (Shape_Compare().operator()((*(edge.begin()++)), (*(edge.rbegin()++))))
				{
					front2 = (*(edge.begin()++));
					back2 = (*(edge.rbegin()++));
				}
				else
				{
					front2 = (*(edge.rbegin()++));
					back2 = (*(edge.begin()++));
				}
				size_t first = front.HashCode(0xFFFF);
				size_t second = front2.HashCode(0xFFFF);
				size_t secondLast = back2.HashCode(0xFFFF);
				size_t last = back.HashCode(0x7FFF);
				result = first | second << 16 | secondLast<< 32 | last << 48;
			}
			else
			{
				size_t maxHash = 0;
				size_t minHash = 0xFFFFFFFFFFFFFFFF;
				for (auto v:edge)
				{
					auto currentHash = v.HashCode(0x7FFFFFFF);
					if (currentHash>maxHash)
					{
						maxHash = currentHash;
					}

					if (currentHash<minHash)
					{
						minHash = currentHash;
					}
				}
				result = maxHash | minHash << 32;
			}
			return result;
		}
	};

	class EdgeGroup : public vector<ExtractedFeatureEdge>
	{
	public:
	protected:
	private:
	};

	class EdgeGroups : public vector<EdgeGroup> 
	{
	public:
		vector<ExtractedFeatureEdge>& FindEdgeGroup(TopoDS_Vertex v, int &index);
	};

	class ExtractedFeature
	{
	public:
		void AddFace(TopoDS_Face face)
		{
			if (!ContainsFace(face))
				faces.push_back(face);
		}
		void AddVertex(TopoDS_Vertex vertex, EdgeVertexType type = PLANAR) { vertices.insert({ vertex, type }); }
		void AddEdgeVertex(TopoDS_Vertex vertex, EdgeVertexType type) { vertices[vertex] = type; edgeVertices.insert({ vertex,type }); }
		int NumFaces() { return faces.size(); }
		bool ContainsFace(TopoDS_Face face);
		bool ContainsVertex(TopoDS_Vertex vertex);
		TopoDS_Face GetFace(int n) { return faces[n]; };
		TopoDS_Vertex GetNearbyEdgeVertex(TopoDS_Shape shape, TopoDS_Vertex vertex);
		void ProcessEdges();
		void CreateEdge(TopoDS_Vertex vertex, TopoDS_Edge edge, EdgeType type, map<TopoDS_Vertex, pair<EdgeVertexType, int>, Shape_Compare> &edgeVertexMap, map<int, ExtractedFeatureEdge> &edgeMap, queue<int> &edgeQueue, int &numProcessed, vector<TopoDS_Vertex> &verticesToProcess);
		void ProcessEdgeGroups();
		void GetOuterEdgeGroup(int &index);
		const vector<TopoDS_Face> GetFaces() { return faces; }
		const vector<TopoDS_Vertex> GetVertices();
		int NumEdges() { return extractedEdges.size(); }
		int NumEdgeGroups() { return edgeGroups.size(); }
		const EdgeGroups GetEdgeGroups() const { return edgeGroups; }
		const vector<ExtractedFeatureEdge> GetEdges() const { return extractedEdges; }
		//FeatureCategorisation::CategorisationType Type() const { return type; }
	
	protected:
	private:
		vector<TopoDS_Face> faces;
		VerticesTypeMap vertices;
		map<TopoDS_Vertex,EdgeVertexType, Shape_Compare> edgeVertices;
		vector<EdgeVertexType> edgeVerticesTypes;
		vector<ExtractedFeatureEdge> extractedEdges;
		EdgeGroups edgeGroups;

	};

	class ExtractedFeatures : public vector<ExtractedFeature>
	{
	public:
		bool IsFaceProcessed(TopoDS_Face face);
		bool IsVertexProcessed(TopoDS_Vertex vertex);
		int NumFaces();
		void ProcessEdges();
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
		void AddEdgeVertex(TopoDS_Vertex vertex, EdgeVertexType type)
		{
			this->back().AddEdgeVertex(vertex, type);
		}

	protected:
	private:
	};

	//static double NormalWeight(TopoDS_Face face, TopoDS_Vertex vertex);
	ExtractedFeatures NormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle = 5.0f);
	ExtractedFeatures HybridEdgewiseNormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle = 5.0f);
	ExtractedFeatures EdgewiseMethod(TopoDS_Shape model, float creaseAngle = 5.0f);
	double FaceNormalWeightMWSLER(TopoDS_Face currentFace, TopoDS_Vertex currentVertex);
	
	Vector3d FaceNormal(TopoDS_Face face);

	template<typename List>
	void CalcNormalTensor(List &currentFaces, TopoDS_Vertex v, Matrix3d &normalTensorSum, Vector3d &normalSum);

	bool IsVertexInQueue(vector<TopoDS_Vertex> list, TopoDS_Vertex vertex);

	int FindVertex(vector<TopoDS_Vertex> list, TopoDS_Vertex vertex);

};