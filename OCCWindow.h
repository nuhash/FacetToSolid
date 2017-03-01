#pragma once
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
#include "occview.h"

class OCCWindow : public wxWindow
{
public:
	OCCWindow(wxWindow* parent, wxWindowID id = wxID_ANY) :
		wxWindow(parent, id)
	{   }

	OCCView* occview;
	bool dragging;

	// this callback function is really important.
	// otherwise wxwidget just paints over the window 	
	void OnPaint(wxPaintEvent& event)
	{
		occview->reset();
	}

	void OnResize(wxSizeEvent& event)
	{
		//occview->redraw();
	}
	void onMouseUp(wxMouseEvent& event)
	{
		ReleaseMouse();
		dragging = false;
	}
	void OnMouseDown(wxMouseEvent& event)
	{
		CaptureMouse();
		mouseX = event.GetX();
		mouseY = event.GetY();
		occview->StartRotation(mouseX, mouseY);
		dragging = true;
	}
	void onMove(wxMouseEvent& event)
	{
		if (dragging)
		{
			//wxPoint mouseOnScreen = wxGetMousePosition();
			//int dx = event.GetX() - mouseX;
			//xVal += dx;
			//int dy = event.GetY() - mouseY;
			//yVal += dy;
			occview->rotate(event.GetX(),
				event.GetY());
		}
	}

private:
	int mouseX;
	int mouseY;
	int xVal = 0;
	int yVal = 0;

	DECLARE_EVENT_TABLE()
};