#pragma once
#include "FeatureExtraction.h"

namespace FeatureCategorisation
{
//public:
	enum CategorisationType
	{
		PLANAR=0,
		SPHERICAL=1,
		TUBULAR=2,
		SWEPT=3,
		COMPLEX=4 //Used for all features that cannot be categorised
	};
	//static const vector<std::string> CategoryNames = { "Planar","Spherical","Prismoidal","Swept","Complex" };
	CategorisationType Categorise(FeatureExtractionAlgo::ExtractedFeature feature, double creaseAngle = 5);
//protected:
	//bool PlanarCheck(const vector<TopoDS_Face> faces, double creaseAngle);
	//bool SphericalCheck(const vector<TopoDS_Vertex> vertices, double allowableError = 0.1);
	//bool TubularCheck(const vector<TopoDS_Vertex> vertices, int numEdgeGroups, double allowableError = 0.1);
//private:
};