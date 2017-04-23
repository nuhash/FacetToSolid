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
		SurfaceReconstructor(ExtractedFeatures &_features, vector<shared_ptr<SurfaceCategorisationData>> &_featureData, EdgeCategoryMap _edgeCategoryMap) :features(_features), edgeCategoryMap(_edgeCategoryMap), featureData(_featureData)
		{

		}
		
		
		void Process();
		
	protected:
		void ReconstructPlanarSurface(ExtractedFeature feature, const shared_ptr<SurfaceCategorisationData> data);
		void ReconstructLinearEdge(ExtractedFeatureEdge edge, const shared_ptr<EdgeCategorisationData> data);
		void ReconstructCircularEdge(ExtractedFeatureEdge edge, const shared_ptr<EdgeCategorisationData> data);
		void ReconstructEdges(vector<ExtractedFeatureEdge> edges);
	private:
		EdgeCategoryMap &edgeCategoryMap;
		vector<shared_ptr<SurfaceCategorisationData>> &featureData;
		ExtractedFeatures &features;
		ReconstructedObject object;
		ReconstructedEdgeMap reconstructedEdgeMap;
	};
};