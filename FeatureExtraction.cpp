#include "FeatureExtraction.h"
#include <TopExp.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS.hxx>
#include <assert.h>
void FeatureExtractionAlgo::NormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle /*= 5.0f*/)
{
	TopTools_IndexedDataMapOfShapeListOfShape vertexToFaceMap;
	TopExp::MapShapesAndAncestors(model, TopAbs_VERTEX, TopAbs_FACE, vertexToFaceMap);
	TopTools_IndexedMapOfShape vertices;
	TopExp::MapShapes(model, TopAbs_VERTEX, vertices);
	TopoDS_Vertex first = TopoDS::Vertex(vertices.FindKey(1));
	auto faces = vertexToFaceMap.FindFromKey(first);
	assert(faces.Size() > 1);
}
