#pragma once

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <GeomLProp_SLProps.hxx>
using namespace Eigen;
namespace Utilities {
	struct ConversionHelpers {
		Vector3d operator()(const TopoDS_Vertex& v) const
		{
			auto p = BRep_Tool::Pnt(v);
			return Vector3d(p.X(), p.Y(), p.Z());
		}

		Vector3d operator()(const TopoDS_Face& face) const
		{
			Standard_Real umin, umax, vmin, vmax;
			BRepTools::UVBounds(face, umin, umax, vmin, vmax);

			auto currentSurface = BRep_Tool::Surface(face);

			GeomLProp_SLProps props(currentSurface, umin, vmin, 1, 0.01);
			gp_Dir normal = props.Normal();

			return Vector3d(normal.X(), normal.Y(), normal.Z());
		}

	};
	const ConversionHelpers converter;
}