#include "SurfaceReconstruction.h"
#include <gp_Ax1.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
namespace SurfaceReconstructionAlgo {

	void SurfaceReconstructor::ReconstructLinearEdge(ExtractedFeatureEdge edge, const shared_ptr<EdgeCategorisationData> data)
	{

		Vector3d normal = static_pointer_cast<LinearEdgeData>(data)->dir;
		Vector3d pos = static_pointer_cast<LinearEdgeData>(data)->pos;

		gp_Ax1 axis(gp_Pnt(pos.x(),pos.y(),pos.z()), gp_Dir(normal.x(), normal.y(), normal.z()));
		Handle(Geom_Line) line = new Geom_Line(axis);
		BRepBuilderAPI_MakeEdge mE(line, edge.front(), edge.back());

		reconstructedEdgeMap.insert({ edge, TopoDS::Edge(mE.Shape()) });
	}

	void SurfaceReconstructor::ReconstructCircularEdge(ExtractedFeatureEdge edge, const shared_ptr<EdgeCategorisationData> data)
	{
		auto circularData = static_pointer_cast<CircularEdgeData>(data);
		Vector3d normal = circularData->normal;
		Vector3d pos = circularData->position;
		double radius = circularData->radius;

		gp_Ax2 axis(converter.operator()<gp_Pnt>(pos), converter.operator()<gp_Dir>(normal));

		gp_Circ circ(axis, radius);

		Handle(Geom_Circle) circle = new Geom_Circle(circ);
		BRepBuilderAPI_MakeEdge mE;
		if (edge.Type() == CONTINUOUS)
		{
			mE = BRepBuilderAPI_MakeEdge(circle);
		} 
		else
		{
			mE = BRepBuilderAPI_MakeEdge(circle, edge.front(), edge.back());
		}

		reconstructedEdgeMap.insert({ edge,TopoDS::Edge(mE.Shape()) });		
	}

	void ReconstructPlane()
	{

	}

	ReconstructedSurface ReconstructSurface(ExtractedFeature feature, FeatureCategorisationType type, EdgeCategoryMap &edgeCategoryMap)
	{
		return ReconstructedSurface();
	}

	void SurfaceReconstructor::Process()
{
	for (size_t i = 0; i < features.size();i++)
		{
			auto feature = features[i];
			auto data = featureData[i];
			switch (data->type)
			{
			default:
				break;
			case FeatureCategorisation::PLANAR:
				break;
			case FeatureCategorisation::SPHERICAL:
				break;
			case FeatureCategorisation::TUBULAR:
				break;
			case FeatureCategorisation::SWEPT:
				break;
			case FeatureCategorisation::COMPLEX:
				break;
			}
		}
	}

	void SurfaceReconstructor::ReconstructPlanarSurface(ExtractedFeature feature, const shared_ptr<SurfaceCategorisationData> data)
	{
		shared_ptr<PlanarSurfaceData> planeData = static_pointer_cast<PlanarSurfaceData>(data);
		gp_Pln pln(converter.operator () < gp_Pnt > (planeData->centroid), converter.operator() < gp_Dir > (planeData->normal));
		BRepBuilderAPI_MakeFace mF(pln);
		int outerEdgeGroupIndex;
		auto outerEdgeGroup = feature.GetOuterEdgeGroup(outerEdgeGroupIndex);
		//find outer edgeGroup
		//add outer edgeGroup
		//add others
	}

	void SurfaceReconstructor::ReconstructEdges(vector<ExtractedFeatureEdge> edges)
	{
		for (auto edge:edges)
		{
			if (reconstructedEdgeMap.count(edge)==1)
			{
				continue;
			}
			auto data = edgeCategoryMap[edge];
			
			switch (data->type)
			{
			default:
				break;
			case FeatureCategorisation::LINEAR:
				ReconstructLinearEdge(edge, data);
				break;
			case FeatureCategorisation::CIRCULAR:
				ReconstructCircularEdge(edge, data);
				break;
			case FeatureCategorisation::FREE:
				throw;
				break;
			}
		}
	}

}

