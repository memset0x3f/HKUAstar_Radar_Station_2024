#ifndef GUI_PKG_MOUSEEVENT_H
#define GUI_PKG_MOUSEEVENT_H

#include <QMainWindow>
#include <QObject>
#include <QLabel>
#include <QPoint>
#include <QMouseEvent>
#include <QPainter>
#include <ros/ros.h>

class MouseEvent : public QLabel {
    Q_OBJECT
protected:
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void paintEvent(QPaintEvent* );
private:
    bool isPointSelected;
public:
    explicit MouseEvent(QWidget *parent = nullptr);
    QPoint selectedPoint;
    QString cameraCalibrating;
    QString leftImgRaw;
    QString rightImgRaw;
    QPoint* leftPoints;
    QPoint* rightPoints;
    bool isDragging;
    int calibrateMainWindowWidth;
    int calibrateMainWindowLength;

signals:
    void mouseMovePoint(QPoint point);
    void mouseClick(QPoint point);
    void mouseRelease(QPoint point);
};

#endif