#include "FeatureCategorisation.h"
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GeomLProp_SLProps.hxx>

using namespace Eigen;

FeatureCategorisation::CategorisationType FeatureCategorisation::Categorise(FeatureExtractionAlgo::ExtractedFeature feature, double creaseAngle /*= 5*/)
{
	auto faces = feature.GetFaces();
	if (PlanarCheck(faces, creaseAngle))
	{
		return PLANAR;
	}

	return COMPLEX;
}

bool FeatureCategorisation::PlanarCheck(const vector<TopoDS_Face> faces, double creaseAngle)
{
	double limit = cos(creaseAngle);
	double sum = 0;
	Vector3d normalAvg(0,0,0);
	for (size_t i = 0; i < faces.size(); i++)
	{
		auto currentFace = faces[i];

		Standard_Real umin, umax, vmin, vmax;
		BRepTools::UVBounds(currentFace, umin, umax, vmin, vmax);

		auto currentSurface = BRep_Tool::Surface(currentFace);

		GeomLProp_SLProps props(currentSurface, umin, vmin, 1, 0.01);
		gp_Dir normal = props.Normal();
		Vector3d currentNormal(normal.X(), normal.Y(), normal.Z());

		if (i != 0)
		{
			auto dotProduct = normalAvg.dot(currentNormal);
			if (dotProduct < limit)
				return false;
		}
			

		normalAvg = (i*normalAvg) / (i + 1) + currentNormal / (i + 1);
	}
	return true;
}
