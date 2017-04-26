#pragma once

#include "FeatureExtraction.h"
#include "FeatureCategorisation.h"
#include <TopoDS_Wire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Builder.hxx>
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
	class ReconstructedEdgeMap : public unordered_map<size_t, TopoDS_Edge> {};// , EdgeHash > {};


	class SurfaceReconstructor
	{
	public:
		SurfaceReconstructor(ExtractedFeatures &_features, vector<shared_ptr<SurfaceCategorisationData>> &_featureData, EdgeCategoryMap &_edgeCategoryMap) :features(_features), edgeCategoryMap(_edgeCategoryMap), featureData(_featureData)
		{
			builder.MakeCompound(tempShape);
		}
		
		
		void Process();
		TopoDS_Shape GetShape() {
			return sew.SewedShape(); 
		}
	protected:
		void ReconstructPlanarSurface(ExtractedFeature feature, const shared_ptr<SurfaceCategorisationData> data, vector<TopoDS_Wire> &reconstructedEdges);
		void ReconstructCylindricalSurface(ExtractedFeature feature, const shared_ptr<SurfaceCategorisationData> data, vector<TopoDS_Wire> &reconstructedEdges);
		void ReconstructLinearEdge(ExtractedFeatureEdge edge, const shared_ptr<EdgeCategorisationData> data);
		void ReconstructCircularEdge(ExtractedFeatureEdge edge, const shared_ptr<EdgeCategorisationData> data);
		void ReconstructEdges();
	private:
		EdgeCategoryMap &edgeCategoryMap;
		vector<shared_ptr<SurfaceCategorisationData>> &featureData;
		ExtractedFeatures &features;
		ReconstructedObject object;
		ReconstructedEdgeMap reconstructedEdgeMap;
		vector<vector<TopoDS_Wire>> reconstructedEdgeGroups;
		BRepBuilderAPI_Sewing sew;
		TopoDS_Compound tempShape;
		TopoDS_Builder builder;
	};
};