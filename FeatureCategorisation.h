#pragma once
#include "FeatureExtraction.h"
#include "Utilities.h"
#include "FeatureExtraction.h"
#include <memory>
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

	struct CategorisationData
	{
		//EdgeCategorisationType type;
	};

	struct EdgeCategorisationData : CategorisationData
	{
		EdgeCategorisationType type;
	};

	struct SurfaceCategorisationData
	{
		FeatureCategorisationType type;
	};

	struct LinearEdgeData :EdgeCategorisationData
	{
		Vector3d dir;
		Vector3d pos;
	};
	struct CircularEdgeData : EdgeCategorisationData
	{
		Vector3d position;
		double radius;
		Vector3d normal;
	};
	struct PlanarSurfaceData :SurfaceCategorisationData
	{
		Vector3d normal;
		Vector3d centroid;
	};

	struct SphericalSurfaceData : SurfaceCategorisationData
	{
		Vector3d position;
		double radius;
	};



	FeatureCategorisationType CategoriseFeature(FeatureExtractionAlgo::ExtractedFeature feature, shared_ptr<SurfaceCategorisationData> &data, double creaseAngle = 5);
	void CategoriseFeatures(FeatureExtractionAlgo::ExtractedFeatures features, vector<shared_ptr<SurfaceCategorisationData>> &data, double creaseAngle = 5);
	class EdgeCategoryMap : public unordered_map<ExtractedFeatureEdge, shared_ptr<EdgeCategorisationData>, EdgeHash> {};
	
	void CategoriseEdges(const ExtractedFeatures &features, EdgeCategoryMap &edgeCategoryMap);
//protected:
	//bool PlanarCheck(const vector<TopoDS_Face> faces, double creaseAngle);
	//bool SphericalCheck(const vector<TopoDS_Vertex> vertices, double allowableError = 0.1);
	//bool TubularCheck(const vector<TopoDS_Vertex> vertices, int numEdgeGroups, double allowableError = 0.1);
//private:
};