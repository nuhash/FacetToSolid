#pragma once
#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
class FeatureExtractionAlgo
{
public:
	static void NormalTensorFrameworkMethod(TopoDS_Shape model, float creaseAngle = 5.0f);
protected:
private:
};