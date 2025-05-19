#include <QApplication>
#include "lib.hpp"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    return app.exec() + foo();
}
