#include "MainFrame.h"
#include <occview.h>
#include "OCCWindow.h"

#include <wx/listctrl.h>
#include <wx/combobox.h>
#include <StlAPI.hxx>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Builder.hxx>
#include "FeatureExtraction.h"
#include <c:/OpenCASCADE7.1.0-vc10-64/opencascade-7.1.0/inc/TopoDS_Compound.hxx>
#include "FeatureCategorisation.h"
// ----------------------------------------------------------------------------
// constants
// ----------------------------------------------------------------------------

// IDs for the controls and the menu commands
enum
{
	// menu items
	FTS_Quit = wxID_EXIT,
	FTS_ModeNotebook = wxID_HIGHEST + 1,
	FTS_ExportCrude = wxID_HIGHEST + 2,
	FTS_Open = wxID_OPEN,
	FTS_ExtractFeatures = wxID_HIGHEST + 3,
	FTS_FeatureSelect = wxID_HIGHEST +4,
	FTS_CategoriseFeatures = wxID_HIGHEST +5,
	FTS_Reconstruct = wxID_HIGHEST+6,
	// it is important for the id corresponding to the "About" command to have
	// this standard value as otherwise it won't be handled properly under Mac
	// (where it is special and put into the "Apple" menu)
	FTS_About = wxID_ABOUT
};

// ----------------------------------------------------------------------------
// event tables and other macros for wxWidgets
// ----------------------------------------------------------------------------

// the event tables connect the wxWidgets events with the functions (event
// handlers) which process them. It can be also done at run-time, but for the
// simple menu events like this the static method is much simpler.
wxBEGIN_EVENT_TABLE(MyFrame, wxFrame)
EVT_MENU(FTS_Quit, MyFrame::OnQuit)
EVT_MENU(FTS_About, MyFrame::OnAbout)
EVT_MENU(FTS_Open, MyFrame::OnOpen)
EVT_BUTTON(FTS_ExportCrude, MyFrame::OnCrudeExport)
EVT_BUTTON(FTS_ExtractFeatures, MyFrame::OnExtractFeatures)
EVT_LISTBOX(FTS_FeatureSelect, MyFrame::OnFeatureSelect)
EVT_BUTTON(FTS_CategoriseFeatures, MyFrame::OnCategorise)
EVT_BUTTON(FTS_Reconstruct, MyFrame::OnReconstruct)
wxEND_EVENT_TABLE()

// ----------------------------------------------------------------------------
// main frame
// ----------------------------------------------------------------------------

// frame constructor
MyFrame::MyFrame(const wxString& title)
	: wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(1200, 1024))
{
	// set the frame icon
	SetIcon(wxICON(sample));

#if wxUSE_MENUS
	// create a menu bar
	wxMenu *fileMenu = new wxMenu;

	// the "About" item should be in the help menu
	wxMenu *helpMenu = new wxMenu;
	helpMenu->Append(FTS_About, "&About\tF1", "Show about dialog");

	fileMenu->Append(FTS_Open, "&Open\tCtrl-O", "Open a file");
	fileMenu->Append(FTS_Quit, "E&xit\tAlt-X", "Quit this program");

	// now append the freshly created menu to the menu bar...
	wxMenuBar *menuBar = new wxMenuBar();
	menuBar->Append(fileMenu, "&File");
	menuBar->Append(helpMenu, "&Help");

	// ... and attach this menu bar to the frame
	SetMenuBar(menuBar);
#else // !wxUSE_MENUS
	// If menus are not available add a button to access the about box
	wxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);
	wxButton* aboutBtn = new wxButton(this, wxID_ANY, "About...");
	aboutBtn->Bind(wxEVT_BUTTON, &MyFrame::OnAbout, this);
	sizer->Add(aboutBtn, wxSizerFlags().Center());
#endif // wxUSE_MENUS/!wxUSE_MENUS

#if wxUSE_STATUSBAR
	// create a status bar just for fun (by default with 1 pane only)
	CreateStatusBar(2);
	SetStatusText("Welcome to wxWidgets!");
#endif // wxUSE_STATUSBAR
}


// event handlers

void MyFrame::OnQuit(wxCommandEvent& WXUNUSED(event))
{
	// true is to force the frame to close
	Close(true);
}

void MyFrame::OnAbout(wxCommandEvent& WXUNUSED(event))
{
	wxMessageBox(wxString::Format
		(
			"Welcome to %s!\n"
			"\n"
			"This is the minimal wxWidgets sample\n"
			"running under %s.",
			wxVERSION_STRING,
			wxGetOsDescription()
			),
		"About wxWidgets minimal sample",
		wxOK | wxICON_INFORMATION,
		this);
}

void MyFrame::OnCrudeExport(wxCommandEvent& event)
{
	wxFileDialog* SaveDialog = new wxFileDialog(
		this, _("Enter file name to save exported STEP file"), wxEmptyString, wxEmptyString, _("STEP files (*.step)|*.STEP"), wxFD_SAVE, wxDefaultPosition);
	if (SaveDialog->ShowModal() == wxID_OK)
	{
		STEPControl_Writer writer;
		writer.Transfer(occView->GetCurrentShape(), STEPControl_AsIs);
		writer.Write(SaveDialog->GetPath());
	}
}

void MyFrame::OnOpen(wxCommandEvent& event)
{
	wxFileDialog* OpenDialog = new wxFileDialog(
		this, _("Choose a file to open"), wxEmptyString, wxEmptyString, _("STL files (*.stl)|*.STL"), wxFD_OPEN, wxDefaultPosition);
	if (OpenDialog->ShowModal() == wxID_OK)
	{
		TopoDS_Shape facetFile;
		StlAPI::Read(facetFile, OpenDialog->GetPath());
		occView->drawShape(facetFile);
	}
}

void MyFrame::OnFeatureSelect(wxCommandEvent& event)
{
	int pos = 0;
	if (myNotebook->GetSelection() == 0)
		pos = featureSel->GetSelection();
	else
		pos = featureSel2->GetSelection();
	TopoDS_Compound tempShape;
	TopoDS_Builder builder;
	builder.MakeCompound(tempShape);
	for (size_t i = 0; i < features.at(pos).NumFaces(); i++)
	{
		builder.Add(tempShape, features.at(pos).GetFace(i));
	}
	occView->drawShape(tempShape,false);
}

void MyFrame::Init()
{
	myNotebook = new wxNotebook(this, FTS_ModeNotebook, wxDefaultPosition, wxSize(300, 500));
	auto featureExtPage = new wxPanel(myNotebook, -1);
	
	auto featureExtSizer = new wxBoxSizer(wxVERTICAL);
	auto methodSelSizer = new wxBoxSizer(wxHORIZONTAL);
	wxArrayString str;
	str.Add("Normal Tensor Framework Method");
	str.Add("Hybrid Edgewise Normal Tensor Framework Method");
	str.Add("Edgewise Method");
	extMethodList = new wxChoice(featureExtPage, wxID_ANY,wxDefaultPosition,wxDefaultSize,str);
	methodSelSizer->Add(extMethodList, 1, wxALIGN_TOP, 2);
	auto extFeaturesButton = new wxButton(featureExtPage, FTS_ExtractFeatures, "Extract Features");
	methodSelSizer->Add(extFeaturesButton, 0, wxALIGN_TOP, 0);
	
	auto featureSelSizer = new wxBoxSizer(wxVERTICAL);
	featureSel = new wxListBox(featureExtPage,FTS_FeatureSelect);
	featureSelSizer->Add(featureSel, 1, wxEXPAND|wxALIGN_TOP, 0);

	auto featureSelButtonsSizer = new wxBoxSizer(wxHORIZONTAL);
	auto addFeatureButton = new wxButton(featureExtPage, wxID_ANY, "Add Feature");
	auto editFeatureButton = new wxButton(featureExtPage, wxID_ANY, "Edit Feature");
	auto removeFeatureButton = new wxButton(featureExtPage, wxID_ANY, "Remove Feature");
	featureSelButtonsSizer->Add(addFeatureButton, 1, wxEXPAND);
	featureSelButtonsSizer->Add(editFeatureButton, 1, wxEXPAND);
	featureSelButtonsSizer->Add(removeFeatureButton, 1, wxEXPAND);

	featureExtSizer->Add(methodSelSizer);
	featureExtSizer->Add(featureSelSizer);
	featureExtSizer->Add(featureSelButtonsSizer);
	featureExtPage->SetSizer(featureExtSizer);
	
	myNotebook->AddPage(featureExtPage, L"Feature Extraction");

	auto surfFitPage = new wxPanel(myNotebook, -1);
	auto surfFitSizer = new wxBoxSizer(wxVERTICAL);
	auto featureSelSizer2 = new wxBoxSizer(wxVERTICAL);
	featureSel2 = new wxListBox(surfFitPage, FTS_FeatureSelect);
	auto categoriseButton = new wxButton(surfFitPage, FTS_CategoriseFeatures, "Categorise Features");
	categoryInfo = new wxListBox(surfFitPage, -1);
	auto changeCategoryButton = new wxButton(surfFitPage, -1, "Change Category");
	auto reconstructButton = new wxButton(surfFitPage, FTS_Reconstruct, "Reconstruct Surfaces");
	featureSelSizer2->Add(featureSel2, 1, wxEXPAND | wxALIGN_TOP, 0);
	featureSelSizer2->Add(categoriseButton, 1, wxEXPAND);
	featureSelSizer2->Add(categoryInfo,1,wxEXPAND);
	featureSelSizer2->Add(changeCategoryButton, 1, wxEXPAND);
	featureSelSizer2->Add(reconstructButton, 1, wxEXPAND);
	surfFitSizer->Add(featureSelSizer2);
	surfFitPage->SetSizer(surfFitSizer);
	myNotebook->AddPage(surfFitPage, L"Surface Fitting");

	auto exportPage = new wxPanel(myNotebook, -1);
	auto cloneButton = new wxButton(exportPage, FTS_ExportCrude, "Export Crude Conversion");
	myNotebook->AddPage(exportPage, L"Export");

	occWindow = new OCCWindow(this);
	occView = new OCCView(occWindow);
	occWindow->occview = occView;

	wxBoxSizer *sizer = new wxBoxSizer(wxHORIZONTAL);
	sizer->Add(myNotebook, 0, wxEXPAND, 0);
	sizer->Add(occWindow, 1, wxEXPAND, 0);
	this->SetSizer(sizer);

	// and show it (the frames, unlike simple controls, are not shown when
	// created initially)
	this->Show(true);
}

void MyFrame::OnExtractFeatures(wxCommandEvent& event)
{
	featureSel->Clear();
	int currentSelection = extMethodList->GetCurrentSelection();

	switch (currentSelection)
	{
	case 0: //Normal tensor framework method
		features = FeatureExtractionAlgo::NormalTensorFrameworkMethod(occView->GetCurrentShape());
		for (size_t i = 0; i < features.size(); i++)
		{
			wxString label = wxString::Format(wxT("Faces:%i;Edges:%i"), features.at(i).NumFaces(), features.at(i).NumEdges());
			featureSel->AppendString(label);
			featureSel2->AppendString(label);
		}
		break;
	case 1:
		features = FeatureExtractionAlgo::HybridEdgewiseNormalTensorFrameworkMethod(occView->GetCurrentShape());
		for (size_t i = 0; i < features.size(); i++)
		{
			wxString label = wxString::Format(wxT("Faces:%i;Edges:%i"), features.at(i).NumFaces(), features.at(i).NumEdges());
			featureSel->AppendString(label);
			featureSel2->AppendString(label);
		}
		break;
	case 2:
		features = FeatureExtractionAlgo::EdgewiseMethod(occView->GetCurrentShape());
		for (size_t i = 0; i < features.size(); i++)
		{
			wxString label = wxString::Format(wxT("Faces:%i;Edges:%i"), features.at(i).NumFaces(), features.at(i).NumEdges());
			featureSel->AppendString(label);
			featureSel2->AppendString(label);
		}
		break;
	default:
		break;
	}


}

void MyFrame::OnCategorise(wxCommandEvent& event)
{
	FeatureCategorisation::CategoriseFeatures(features, featureData);
	FeatureCategorisation::CategoriseEdges(features, edgeCategoryMap);
	for (size_t i = 0; i < features.size(); i++)
	{
		wxString label = wxString::Format(wxT("Faces:%i;Edges:%i;Category:%i"), features.at(i).NumFaces(), features.at(i).NumEdges(),featureData[i]->type);
		categoryInfo->AppendString(label);
	}
	
}

void MyFrame::OnReconstruct(wxCommandEvent& event)
{
	

}

