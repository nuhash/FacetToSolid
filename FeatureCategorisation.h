#pragma once
#include "FeatureExtraction.h"

class FeatureCategorisation
{
public:
	enum CategorisationType
	{
		PLANAR,
		SPHERICAL,
		PRISMOIDAL,
		SWEPT,
		COMPLEX //Used for all features that cannot be categorised
	};
	static CategorisationType Categorise(FeatureExtractionAlgo::ExtractedFeature feature, double creaseAngle = 5);
protected:
	static bool PlanarCheck(const vector<TopoDS_Face> faces, double creaseAngle);
private:
};