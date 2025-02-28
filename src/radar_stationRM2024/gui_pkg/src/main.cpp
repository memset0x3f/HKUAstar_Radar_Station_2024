#include <QtGui>
#include <QApplication>
#include <QLoggingCategory>
#include "gui_pkg/main_window.hpp"
#include <unistd.h>

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QLoggingCategory::setFilterRules("myapp.debug=true");
    gui_pkg::MainWindow window(argc, argv);
    window.show();
    int result = app.exec();
    std::cout << window.size().width() << '\t' << window.size().height() << std::endl;
    return result;
}