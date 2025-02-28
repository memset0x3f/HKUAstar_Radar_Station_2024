#ifndef gui_pkg_MAIN_WINDOW_HPP_
#define gui_pkg_MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QImage>
#include <QMutex>
#include <QListWidgetItem>
#include <QTime>
#include <QTimer>
#include <QApplication>
#include "ui_MainWindow.h"
#include "qnode.hpp"

#define LOG_LIMIT_TIMER_COUNT 11
#define LOG_TIME 5

namespace gui_pkg {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    // Overload
    void closeEvent(QCloseEvent *event);
    
    void showNoMasterMessage();
    
    void init();

public Q_SLOTS:

    void updateLogCameraCalibrateMainWindow();
    void updateLogCameraCalibrateSecondWindow();
    void displayCameraCalibrateMainWindow(const QImage &img);
    void displayCameraCalibrateSecondWindow(const QImage &img);

private slots:
    void onComboBoxCalibrateCamera_currentIndexChanged(const QString &arg1);
    void onLabelCalibrateCameraMainWindow_mouseLocationChanged();
    void onTabWidgetCurrentChange(int index);
    void onCalibrateButtonClick();
    // void onTimeout();

private:
    Ui::MainWindow ui;
    QNode qnode;
    QImage qImageMainWindow;
    QImage qImageSecondWindow;
    QImage qImageCalibrateMainWindow;
    QImage qImageCalibrateSecondWindow;
    mutable QMutex qImageMutex;
    QTimer *fTimer;
    unsigned int log_limit_timer[LOG_LIMIT_TIMER_COUNT];
};

}

#endif
