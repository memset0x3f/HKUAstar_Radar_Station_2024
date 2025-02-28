#include <QtGui>
#include <QMessageBox>
#include "gui_pkg/main_window.hpp"

namespace gui_pkg {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc, argv) {

    ui.setupUi(this);
    QObject::connect(&qnode, SIGNAL(loggingCameraCalibrateMainWindow()), this, SLOT(updateLogCameraCalibrateMainWindow()));
    QObject::connect(&qnode, SIGNAL(loggingCameraCalibrateSecondWindow()), this, SLOT(updateLogCameraCalibrateSecondWindow()));
    setWindowIcon(QIcon(":/image/images/Icon.ico"));
    // QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(ui.labelCalibrateCameraMainWindow, SIGNAL(mouseMovePoint(QPoint)), this, SLOT(onLabelCalibrateCameraMainWindow_mouseLocationChanged()));
    QObject::connect(ui.comboBoxCalibrateCamera, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onComboBoxCalibrateCamera_currentIndexChanged(const QString&)));
    QObject::connect(ui.tabWidget, SIGNAL(currentChanged(int)), this, SLOT(onTabWidgetCurrentChange(int)));
    QObject::connect(ui.pushButtonCalibrate, SIGNAL(clicked()), this, SLOT(onCalibrateButtonClick()));
    // QObject::connect(&qnode, SIGNAL(loggingGameStateUpdate()), this, SLOT(updateGameState()));
    // QObject::connect(&qnode, SIGNAL(loggingUimapUpdate()), this, SLOT(updateUimap()));
    if(!qnode.init()) {
        showNoMasterMessage();
    }
    init();
}

MainWindow::~MainWindow() {}

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("No ros Master detected!");
    msgBox.exec();
    close();
}

void MainWindow::init() {
    ui.comboBoxCalibrateCamera->clear();
    QStringList stringList;
    stringList << qnode.leftImgRaw << qnode.rightImgRaw;
    ui.comboBoxCalibrateCamera->addItems(stringList);

    ui.tabWidget->setCurrentIndex(0);

    ui.labelCalibrateCameraMainWindow->cameraCalibrating = qnode.cameraCalibrating;
    //ui.labelCalibra

    ui.labelCalibrateCameraMainWindow->leftPoints = qnode.leftPoints;
    ui.labelCalibrateCameraMainWindow->rightPoints = qnode.rightPoints;

    ui.labelCalibrateCameraMainWindow->leftImgRaw = qnode.leftImgRaw;
    ui.labelCalibrateCameraMainWindow->rightImgRaw = qnode.rightImgRaw;

    qImageMutex.lock();
    qImageMutex.unlock();

    qImageMutex.lock();
    qImageMutex.unlock();

    qImageMutex.lock();
    qImageMutex.unlock();

    QFile file_red(":/qss/qss/progressBarRed.qss");
    QFile file_blue(":/qss/qss/progressBarBlue.qss");
    file_red.open(QFile::ReadOnly);
    file_blue.open(QFile::ReadOnly);
    QString style_red = QString::fromLatin1(file_red.readAll());
    QString style_blue = QString::fromLatin1(file_blue.readAll());
    fTimer = new QTimer(this);
    fTimer->stop();
    fTimer->setInterval(333);
    // connect(fTimer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    fTimer->start();

    ui.labelCalibrateCameraMainWindow->calibrateMainWindowLength = qnode.calibrateMainWindowLength;
    ui.labelCalibrateCameraMainWindow->calibrateMainWindowWidth = qnode.calibrateMainWindowWidth;

    //ui.labelLogo->setMaximumSize(QSize(qnode.logoWidth, qnode.logoHeight));
    //ui.labelLogo->setMinimumSize(QSize(qnode.logoWidth, qnode.logoHeight));
    ui.labelCalibrateCameraMainWindow->setMaximumSize(QSize(qnode.calibrateMainWindowWidth, qnode.calibrateMainWindowLength));
    ui.labelCalibrateCameraMainWindow->setMinimumSize(QSize(qnode.calibrateMainWindowWidth, qnode.calibrateMainWindowLength));
    ui.labelCalibrateCameraSecondWindow->setMaximumSize(QSize(qnode.calibrateSecondWindowWidth, qnode.calibrateSecondWindowLength));
    ui.labelCalibrateCameraSecondWindow->setMinimumSize(QSize(qnode.calibrateSecondWindowWidth, qnode.calibrateSecondWindowLength));

    for(size_t i = 0; i < LOG_LIMIT_TIMER_COUNT; i++)
    {
        log_limit_timer[i] = 0;
    }

}

void MainWindow::updateLogCameraCalibrateMainWindow() {
    if(qnode.isCalibrating && qnode.cameraCalibrating == qnode.leftImgRaw) {
        printf("Calibrating Left!");
        qnode.imageCalibrateMainWindow = QImage(qnode.imgLeft.data, qnode.imgLeft.cols, qnode.imgLeft.rows, qnode.imgLeft.step[0], QImage::Format_RGB888);
    } else if(qnode.isCalibrating && qnode.cameraCalibrating == qnode.rightImgRaw) {
        printf("Calibrating Right!");
        qnode.imageCalibrateMainWindow = QImage(qnode.imgRight.data, qnode.imgRight.cols, qnode.imgRight.rows, qnode.imgRight.step[0], QImage::Format_RGB888);
    }
    displayCameraCalibrateMainWindow(qnode.imageCalibrateMainWindow);
}

void MainWindow::updateLogCameraCalibrateSecondWindow() {
    cv::Rect r;
    r.width = qnode.calibrateSecondWindowWidth / qnode.calibrateRate;
    r.height = qnode.calibrateSecondWindowLength / qnode.calibrateRate;
    int halfWidth = qnode.calibrateSecondWindowWidth * 0.5 / qnode.calibrateRate;
    int halfHeight = qnode.calibrateSecondWindowLength * 0.5 / qnode.calibrateRate;

    // constrain max value
    if(ui.labelCalibrateCameraMainWindow->selectedPoint.x() > qnode.calibrateMainWindowWidth) {
        ui.labelCalibrateCameraMainWindow->selectedPoint.setX(qnode.calibrateMainWindowWidth);
    }
    if(ui.labelCalibrateCameraMainWindow->selectedPoint.y() > qnode.calibrateMainWindowLength) {
        ui.labelCalibrateCameraMainWindow->selectedPoint.setY(qnode.calibrateMainWindowLength);
    }

    cv::Mat m;
    if(ui.labelCalibrateCameraMainWindow->selectedPoint.x() - halfWidth < 0) { // on left half
        r.x = 0;
    } else if((ui.labelCalibrateCameraMainWindow->selectedPoint.x() + halfWidth) > 
              (qnode.calibrateMainWindowWidth)) { // on right half
        r.x = qnode.calibrateMainWindowWidth - qnode.calibrateSecondWindowWidth / qnode.calibrateRate;
    } else {
        r.x = ui.labelCalibrateCameraMainWindow->selectedPoint.x() - halfWidth;
    }

    if(ui.labelCalibrateCameraMainWindow->selectedPoint.y() - halfHeight < 0) {
        r.y = 0;
    } else if((ui.labelCalibrateCameraMainWindow->selectedPoint.y() + halfHeight) >
              (qnode.calibrateMainWindowLength)) {
        r.y = qnode.calibrateMainWindowLength - qnode.calibrateSecondWindowLength / qnode.calibrateRate;
    } else {
        r.y = ui.labelCalibrateCameraMainWindow->selectedPoint.y() - halfHeight;
    }

    if(qnode.cameraCalibrating == qnode.leftImgRaw && !qnode.imgLeft.empty()) {
        qnode.imgLeft(r).copyTo(m);
    } else if(qnode.cameraCalibrating == qnode.rightImgRaw && !qnode.imgRight.empty()) {
        qnode.imgRight(r).copyTo(m);
    }
    
    if(!m.empty()) {
        cv::resize(m, m,
                   cv::Size(qnode.calibrateSecondWindowWidth,
                   qnode.calibrateSecondWindowLength));

        cv::line(m,
                 cv::Point(0, (ui.labelCalibrateCameraMainWindow->selectedPoint.y() - r.y) * qnode.calibrateRate),
                 cv::Point(qnode.calibrateSecondWindowWidth,
                 (ui.labelCalibrateCameraMainWindow->selectedPoint.y() - r.y) * qnode.calibrateRate),
                 cv::Scalar(255, 255, 255));

        cv::line(m,
                 cv::Point((ui.labelCalibrateCameraMainWindow->selectedPoint.x() - r.x) * qnode.calibrateRate, 0),
                 cv::Point((ui.labelCalibrateCameraMainWindow->selectedPoint.x() - r.x) * qnode.calibrateRate, qnode.calibrateSecondWindowLength),
                 cv::Scalar(255, 255, 255));
        
        if(qnode.cameraCalibrating == qnode.leftImgRaw) {
            for(int i = 0; i < 4; i++) {
                cv::Point center((ui.labelCalibrateCameraMainWindow->leftPoints[i].x() - r.x) * qnode.calibrateRate,
                                 (ui.labelCalibrateCameraMainWindow->leftPoints[i].y() - r.y) * qnode.calibrateRate);
                cv::circle(m, center, 5 * qnode.calibrateRate, cv::Scalar(255, 255, 255), 2);
                cv::circle(m, center, 2, cv::Scalar(255, 0, 0), 2);
            }

        }
        else if(qnode.cameraCalibrating == qnode.rightImgRaw) {
            for(int i = 0; i < 4; i++) {
                cv::Point center((ui.labelCalibrateCameraMainWindow->rightPoints[i].x() - r.x) * qnode.calibrateRate,
                                 (ui.labelCalibrateCameraMainWindow->rightPoints[i].y() - r.y) * qnode.calibrateRate);
                cv::circle(m, center, 5 * qnode.calibrateRate, cv::Scalar(255, 255, 255), 2);
                cv::circle(m, center, 2, cv::Scalar(255, 0, 0), 2);
            }
        }
        qnode.imageCalibrateSecondWindow = QImage(m.data,m.cols,m.rows,m.step[0],QImage::Format_RGB888);
        displayCameraCalibrateSecondWindow(qnode.imageCalibrateSecondWindow);
        QString qstr((std::string("(") + 
                         std::to_string(ui.labelCalibrateCameraMainWindow->selectedPoint.x() * qnode.rawImageWidth / qnode.calibrateMainWindowWidth) +
                                        std::string(", ") + 
                                        std::to_string(ui.labelCalibrateCameraMainWindow->selectedPoint.y() * 
                                                       qnode.rawImageLength / 
                                                       qnode.calibrateMainWindowLength) + 
                                        std::string(")")).c_str());
        QPalette pal = ui.labelPointLocation->palette();
        if(ui.labelCalibrateCameraMainWindow->isDragging) {
            pal.setColor(QPalette::WindowText, Qt::red);
        } else {
            pal.setColor(QPalette::WindowText, Qt::black);
        }

        ui.labelPointLocation->setPalette(pal);
        ui.labelPointLocation->setText(qstr);
    }
    //displayCameraCalibrateSecondWindow(qnode.imageCalibrateSecondWindow);
}

void MainWindow::displayCameraCalibrateMainWindow(const QImage &img) {
    qImageMutex.lock();
    qImageCalibrateMainWindow = img.copy();
    ui.labelCalibrateCameraMainWindow->setPixmap(QPixmap::fromImage(qImageCalibrateMainWindow));
    ui.labelCalibrateCameraMainWindow->resize(ui.labelCalibrateCameraMainWindow->pixmap()->size());
    qImageMutex.unlock();
}

void MainWindow::displayCameraCalibrateSecondWindow(const QImage &img) {
    qImageMutex.lock();
    qImageCalibrateSecondWindow = img.copy();
    ui.labelCalibrateCameraSecondWindow->setPixmap(QPixmap::fromImage(qImageCalibrateSecondWindow));
    ui.labelCalibrateCameraSecondWindow->resize(ui.labelCalibrateCameraSecondWindow->pixmap()->size());
    qImageMutex.unlock();
}

void MainWindow::closeEvent(QCloseEvent *event) {
    QMainWindow::closeEvent(event);
}

}

void gui_pkg::MainWindow::onComboBoxCalibrateCamera_currentIndexChanged(const QString &arg1) {
    qnode.cameraCalibrating = arg1;
    ui.labelCalibrateCameraMainWindow->cameraCalibrating = arg1;
    emit qnode.loggingCameraCalibrateMainWindow();
    emit qnode.loggingCameraCalibrateSecondWindow();
}

void gui_pkg::MainWindow::onCalibrateButtonClick() {
    qnode.pubCalibration();
    // if (ui.comboBoxCalibrateCamera->currentIndex() == 1) {
    //     ui.comboBoxCalibrateCamera->setCurrentIndex(0);
    // } else {
    //     ui.comboBoxCalibrateCamera->setCurrentIndex(1);
    // }
}

void gui_pkg::MainWindow::onTabWidgetCurrentChange(int index) {
    if (index == 0) {
        qnode.isCalibrating = true;
        emit qnode.loggingCameraCalibrateMainWindow();
        emit qnode.loggingCameraCalibrateSecondWindow();
    } else {
        qnode.isCalibrating = false;
    }
}

void gui_pkg::MainWindow::onLabelCalibrateCameraMainWindow_mouseLocationChanged() {
    qnode.mouseLocation = ui.labelCalibrateCameraMainWindow->selectedPoint;
    emit qnode.loggingCameraCalibrateSecondWindow();
}

