#include <QtGui/QApplication>
#include <IrisCC.hpp>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    IrisCC w;
    w.show();

    return a.exec();
}
