#pragma once

#include <wx/wxprec.h>
#include <V3d_View.hxx>
#include <Graphic3d_GraphicDriver.hxx>
#include <Aspect_Handle.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <WNT_Window.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <BRep_Tool.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>
#include <GCE2d_MakeSegment.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Dir.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>
#include <TopTools_ListOfShape.hxx>
#include <wx/event.h>
#include <wx/window.h>
using namespace std;
class OCCView
{
public:
	OCCView(wxWindow* parentWindow)
	{
		window = parentWindow;
		init();
	}

	wxWindow* window;

	void init(void);

	void drawShape(TopoDS_Shape &shape, bool update=true);

	void UpdateSelected(vector<TopoDS_Face> list);
	// some functions to control the camera/view
	void reset(void);

	void redraw(void) {
		mView->Redraw();
	}
	void rotate(int x, int y)
	{
		mView->Rotation(x, y);
	}

	void StartRotation(int x, int y)
	{
		mView->StartRotation(x, y);
	}

	TopoDS_Shape
		MakeBottle(const Standard_Real myWidth, const Standard_Real myHeight,
			const Standard_Real myThickness);
	TopoDS_Shape GetCurrentShape() { return currentShape; }
protected:
	Handle_V3d_Viewer mViewer;
	Handle_AIS_InteractiveContext mContext;
	Handle_V3d_View mView;
	Handle_WNT_Window wind;
	TopoDS_Shape currentShape;
};





