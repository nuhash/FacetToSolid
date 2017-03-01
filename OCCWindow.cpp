#include "OCCWindow.h"

// here we declare that RIGHT MOUSE button will do something
// in this case, it will rotate the camera/view.
BEGIN_EVENT_TABLE(OCCWindow, wxWindow)
EVT_SIZE(OCCWindow::OnResize)
//  EVT_SHOW(occWindow::OnShow)
EVT_PAINT(OCCWindow::OnPaint)
EVT_RIGHT_DOWN(OCCWindow::OnMouseDown)
EVT_RIGHT_UP(OCCWindow::onMouseUp)
EVT_MOTION(OCCWindow::onMove)
END_EVENT_TABLE()