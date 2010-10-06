#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include "vtkSmartPointer.h"

class vtkRenderer;

class ScannerInteractorStyle;

class Form : public QWidget
{
  Q_OBJECT

public:
  Form(int argc, char** argv, QWidget *parent = 0 );
  int argc;
  char** argv;

public slots:
  void btnScan_clicked();
  void btnPreview_clicked();

  void btnSaveScan_clicked();
  void btnOpenFile_clicked();

  void CreateScannerRepresentation();

private:
  Ui::Form ui;

  void ConnectSlots();

  void Refresh();
  void ResetAndRefresh();

  vtkSmartPointer<vtkRenderer> Renderer;

  vtkSmartPointer<ScannerInteractorStyle> ScannerStyle;

};

#endif
