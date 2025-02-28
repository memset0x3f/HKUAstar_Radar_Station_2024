#ifndef GUI_PKG_PAINTER_H
#define GUI_PKG_PAINTER_H

#include <QMainWindow>
#include <QObject>
#include <QLabel>
#include <QPoint>
#include <QPainter>
#include <QPaintEvent>
#include <chrono>
#include "gui_pkg/qnode.hpp"

class Painter : public QLabel {
    Q_OBJECT
protected:
    void paintEvent(QPaintEvent* );
private:
    QPoint placeRB1_te[4];
    QPoint placeRB2_te[7];
    QPoint placeRB3_te[5];
    QPoint placeLeap_te[4];
    QPoint placeHitWindMill_te[4];
    QPoint placeOutpose_te[4];
    QPoint placeRB1_en[4];
    QPoint placeRB2_en[7];
    QPoint placeRB3_en[5];
    QPoint placeLeap_en[4];
    QPoint placeHitWindMill_en[4];
    QPoint placeOutpose_en[4];
    std::vector<RobotPoint> robotPoints;//收到的点
    unsigned short roiWarnState;//每一个roi的警告情况
public:
    explicit Painter(QWidget *parent = nullptr);
    void drawSmallMap(std::vector<RobotPoint>& );
    void drawROI(unsigned short* );
    bool tim;
private slots:
};

#endif