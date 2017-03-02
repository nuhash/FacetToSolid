#pragma once
#include "wx/wxprec.h"
#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif
#include "OCCWindow.h"
#include "occview.h"

class MyFrame : public wxFrame
{
public:
	// ctor(s)
	MyFrame(const wxString& title);

	// event handlers (these functions should _not_ be virtual)
	void OnQuit(wxCommandEvent& event);
	void OnAbout(wxCommandEvent& event);
	void OnCrudeExport(wxCommandEvent& event);
	void OnOpen(wxCommandEvent& event);
	void Init();
	void OnExtractFeatures(wxCommandEvent& event);
private:
	// any class wishing to process wxWidgets events must use this macro
	OCCWindow *occWindow;
	OCCView* occView;
	wxChoice* extMethodList;
	wxDECLARE_EVENT_TABLE();
};
