#include <QApplication>

#include "form.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  Form form(argc,argv);

  form.show();
  return app.exec();
}

