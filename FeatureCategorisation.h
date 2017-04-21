#pragma once
#include "FeatureExtraction.h"
#include "Utilities.h"
#include "FeatureExtraction.h"
using namespace Utilities;
using namespace FeatureExtractionAlgo;
namespace FeatureCategorisation
{
//public:
	enum FeatureCategorisationType
	{
		PLANAR=0,
		SPHERICAL=1,
		TUBULAR=2,
		SWEPT=3,
		COMPLEX=4 //Used for all features that cannot be categorised
	};

	enum EdgeCategorisationType
	{
		LINEAR = 0,
		CIRCULAR = 1,
		FREE = 2
	};
	//static const vector<std::string> CategoryNames = { "Planar","Spherical","Prismoidal","Swept","Complex" };


	FeatureCategorisationType CategoriseFeatures(FeatureExtractionAlgo::ExtractedFeature feature, double creaseAngle = 5);

	class EdgeCategoryMap : public unordered_map<ExtractedFeatureEdge, EdgeCategorisationType, EdgeHash> {};
	
	void CategoriseEdges(const ExtractedFeatures &features, EdgeCategoryMap &edgeCategoryMap);
//protected:
	//bool PlanarCheck(const vector<TopoDS_Face> faces, double creaseAngle);
	//bool SphericalCheck(const vector<TopoDS_Vertex> vertices, double allowableError = 0.1);
	//bool TubularCheck(const vector<TopoDS_Vertex> vertices, int numEdgeGroups, double allowableError = 0.1);
//private:
};