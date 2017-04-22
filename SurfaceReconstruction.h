#pragma once

#include "FeatureExtraction.h"
#include "FeatureCategorisation.h"
using namespace FeatureExtractionAlgo;
using namespace FeatureCategorisation;
;
namespace SurfaceReconstructionAlgo
{

	class ReconstructedSurface
	{
	public:
	protected:
	private:
	};

	class ReconstructedObject : public vector<ReconstructedSurface>
	{
	public:
	protected:
	private:

	};

	ReconstructedSurface ReconstructSurface(ExtractedFeature feature, FeatureCategorisationType type, EdgeCategoryMap &edgeCategoryMap);
	class ReconstructedEdgeMap : public unordered_map<ExtractedFeatureEdge, TopoDS_Edge, EdgeHash> {};

	class SurfaceReconstructor
	{
	public:

		FeatureCategorisation::EdgeCategoryMap& EdgeCategoryMap() { return edgeCategoryMap; }
		
		void Process(vector<pair<ExtractedFeature, FeatureCategorisationType>> features);
		
	protected:
		void ReconstructPlanarSurface(ExtractedFeature feature);
		void ReconstructLinearEdge(ExtractedFeatureEdge edge);
		void ReconstructEdges(vector<ExtractedFeatureEdge> edges);
	private:
		FeatureCategorisation::EdgeCategoryMap edgeCategoryMap;
		ReconstructedObject object;
		ReconstructedEdgeMap reconstructedEdgeMap;
	};
};