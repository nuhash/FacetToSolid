#include "FTSApp.h"
#include "MainFrame.h"

#include <wx/notebook.h>

wxIMPLEMENT_APP(FTSApp);

void FTSApp::OnCrudeExport(wxCommandEvent& WXUNUSED(event))
{

}

// Create a new application object: this macro will allow wxWidgets to create
// the application object during program execution (it's better than using a
// static object for many reasons) and also implements the accessor function
// wxGetApp() which will return the reference of the right type (i.e. MyApp and
// not wxApp)


// ============================================================================
// implementation
// ============================================================================

// ----------------------------------------------------------------------------
// the application class
// ----------------------------------------------------------------------------

// 'Main program' equivalent: the program execution "starts" here
bool FTSApp::OnInit()
{
	_CrtSetDbgFlag(_CRTDBG_CHECK_ALWAYS_DF);
	// call the base class initialization method, currently it only parses a
	// few common command-line options but it could be do more in the future
	if (!wxApp::OnInit())
		return false;

	// create the main application window
	MyFrame *frame = new MyFrame("Facet File to Solid Model Conversion Program");// , 1024, 1200);
	frame->Init();


	// success: wxApp::OnRun() will be called which will enter the main message
	// loop and the application will run. If we returned false here, the
	// application would exit immediately.
	return true;
}