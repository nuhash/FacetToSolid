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
#include <BRepBuilderAPI_MakeWire.hxx>
namespace SurfaceReconstructionAlgo {

	void SurfaceReconstructor::ReconstructLinearEdge(ExtractedFeatureEdge edge, const shared_ptr<EdgeCategorisationData> data)
	{

		Vector3d normal = static_pointer_cast<LinearEdgeData>(data)->dir;
		Vector3d pos = static_pointer_cast<LinearEdgeData>(data)->pos;

		gp_Ax1 axis(gp_Pnt(pos.x(),pos.y(),pos.z()), gp_Dir(normal.x(), normal.y(), normal.z()));
		Handle(Geom_Line) line = new Geom_Line(axis);
		BRepBuilderAPI_MakeEdge mE(line, edge.front(), edge.back());

		reconstructedEdgeMap.insert({ edge.Hash(), TopoDS::Edge(mE.Shape()) });
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

		reconstructedEdgeMap.insert({ edge.Hash(),TopoDS::Edge(mE.Shape()) });		
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
		ReconstructEdges();
		for (size_t i = 0; i < features.size();i++)
		{
			auto feature = features[i];
			auto data = featureData[i];
			auto &reconstructedEdges = reconstructedEdgeGroups[i];
			switch (data->type)
			{
			default:
				break;
			case FeatureCategorisation::PLANAR:
				ReconstructPlanarSurface(feature, data, reconstructedEdges);
				break;
			case FeatureCategorisation::SPHERICAL:
				throw;
				break;
			case FeatureCategorisation::TUBULAR:
				throw;
				break;
			case FeatureCategorisation::SWEPT:
				throw;
				break;
			case FeatureCategorisation::COMPLEX:
				throw;
				break;
			}
		}
		sew.Perform();
	}

	void SurfaceReconstructor::ReconstructPlanarSurface(ExtractedFeature feature, const shared_ptr<SurfaceCategorisationData> data, vector<TopoDS_Wire> &reconstructedEdges)
	{
		auto surfData = static_pointer_cast<PlanarSurfaceData>(data);
		auto centroid = surfData->centroid;
		auto normal = surfData->normal;

		gp_Pnt point(centroid.x(), centroid.y(), centroid.z());
		gp_Dir dir(normal.x(), normal.y(), normal.z());
		gp_Pln pln(point, dir);
		int outerEdgeGroupIndex;
		feature.GetOuterEdgeGroup(outerEdgeGroupIndex);
		BRepBuilderAPI_MakeFace mF(reconstructedEdges[outerEdgeGroupIndex], false);

		//mF.Add(reconstructedEdges[outerEdgeGroupIndex]);

		for (size_t i = 0; i < reconstructedEdges.size(); i++)
		{
			if (i==outerEdgeGroupIndex)
			{
				continue;
			}
			mF.Add(reconstructedEdges[i]);
		}

		sew.Add(mF.Shape());
		builder.Add(tempShape,mF.Shape());
		//find outer edgeGroup
		//add outer edgeGroup
		//add others
	}

	void SurfaceReconstructor::ReconstructEdges()
{

		for (auto &feature:features)
		{
			auto edgeGroups = feature.GetEdgeGroups();
			reconstructedEdgeGroups.push_back({});
			for (auto &edgeGroup : edgeGroups)
			{
				BRepBuilderAPI_MakeWire mW;
				for (auto &edge:edgeGroup)
				{
					
					if (reconstructedEdgeMap.count(edge.Hash())==0)
					{
						cout << edge.Hash() << endl;
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
					mW.Add(reconstructedEdgeMap[edge.Hash()]);
				}
				reconstructedEdgeGroups.back().push_back(TopoDS::Wire(mW.Shape()));
			}
		}
	}

}

