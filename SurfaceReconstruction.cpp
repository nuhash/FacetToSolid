#include "SurfaceReconstruction.h"
#include <gp_Ax1.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <Geom_Line.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
namespace SurfaceReconstructionAlgo {

	void SurfaceReconstructor::ReconstructLinearEdge(ExtractedFeatureEdge edge)
	{
		MatrixXd V(edge.size(), 3);
		for (size_t i = 0; i < edge.size(); i++)
		{
			V.row(i) = converter(edge[i]);
		}
		V.rowwise() -= V.colwise().mean();
		BDCSVD<MatrixXd> svd(V, ComputeThinU | ComputeThinV);
		Vector3d normal = svd.matrixV().leftCols<1>();
		normal.normalize();

		gp_Ax1 axis(gp_Pnt(V.col(0).mean(), V.col(1).mean(), V.col(2).mean()), gp_Dir(normal.x(), normal.y(), normal.z()));
		Handle(Geom_Line) line = new Geom_Line(axis);
		BRepBuilderAPI_MakeEdge mE(line, edge.front(), edge.back());

		reconstructedEdgeMap.insert({ edge, TopoDS::Edge(mE.Shape()) });
	}

	void ReconstructPlane()
	{

	}

	ReconstructedSurface ReconstructSurface(ExtractedFeature feature, FeatureCategorisationType type, EdgeCategoryMap &edgeCategoryMap)
	{
		return ReconstructedSurface();
	}

	void SurfaceReconstructor::Process(vector<pair<ExtractedFeature,FeatureCategorisationType>> features)
	{
		for (auto feature:features)
		{
			switch (feature.second)
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

	void SurfaceReconstructor::ReconstructPlanarSurface(ExtractedFeature feature)
	{

	}

	void SurfaceReconstructor::ReconstructEdges(vector<ExtractedFeatureEdge> edges)
	{
		for (auto edge:edges)
		{
			if (reconstructedEdgeMap.count(edge)==1)
			{
				continue;
			}
			EdgeCategorisationType cat = edgeCategoryMap[edge];
			switch (cat)
			{
			default:
				break;
			case FeatureCategorisation::LINEAR:
				ReconstructLinearEdge(edge);
				break;
			case FeatureCategorisation::CIRCULAR:
				break;
			case FeatureCategorisation::FREE:
				break;
			}
		}
	}

}

