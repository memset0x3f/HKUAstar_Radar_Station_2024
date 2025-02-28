#include "gui_pkg/MouseEvent.h"

void MouseEvent::mouseMoveEvent(QMouseEvent *event) {
    QPoint point = event->pos();
    QLabel::mouseMoveEvent(event);
    selectedPoint = point;
    double minDist = 100000000000;
    static int minI = 4;
    if (isDragging) {
        if(cameraCalibrating == leftImgRaw) {
            if(isPointSelected) {
                leftPoints[minI] = point;
                std::string str("/GUI/left_cam/calibration/point"), str2;
                str += std::to_string(minI + 1);
                str2 = str + "/x";
                ros::param::set(str2, point.x() * 1.0 / calibrateMainWindowWidth);
                str2 = str + "/y";
                ros::param::set(str2, point.y() * 1.0 / calibrateMainWindowLength);
        
            } else {
                for(int i = 0; i < 4; i++) {
                    double dist = (point.x() - leftPoints[i].x()) * (point.x() - leftPoints[i].x()) + 
                                  (point.y() - leftPoints[i].y()) * (point.y() - leftPoints[i].y());
                    if(minDist > dist) {
                        minDist = dist;
                        minI = i;
                    }
                }
                leftPoints[minI] = point;
                std::string str("/GUI/left_cam/calibration/point"), str2;
                str += std::to_string(minI + 1);
                str2 = str + "/x";
                ros::param::set(str2, point.x() * 1.0 / calibrateMainWindowWidth);
                str2 = str + "/y";
                ros::param::set(str2, point.y() * 1.0 / calibrateMainWindowLength);
                isPointSelected = true;

            }
        } else if(cameraCalibrating == rightImgRaw) {
            if(isPointSelected) {
                rightPoints[minI] = point;
                std::string str("/GUI/right_cam/calibration/point"), str2;
                str += std::to_string(minI + 1);
                str2 = str + "/x";
                ros::param::set(str2, point.x() * 1.0 / calibrateMainWindowWidth);
                str2 = str + "/y";
                ros::param::set(str2, point.y() * 1.0 / calibrateMainWindowLength);

            } else {
                for(int i = 0; i < 4; i++) {
                    double dist = (point.x() - rightPoints[i].x()) * (point.x() - rightPoints[i].x()) + 
                                  (point.y() - rightPoints[i].y()) * (point.y() - rightPoints[i].y());
                    if(minDist > dist) {
                        minDist = dist;
                        minI = i;
                    }
                }
                rightPoints[minI] = point;
                std::string str("/GUI/right_cam/calibration/point"), str2;
                str += std::to_string(minI + 1);
                str2 = str + "/x";
                ros::param::set(str2, point.x() * 1.0 / calibrateMainWindowWidth);
                str2 = str + "/y";
                ros::param::set(str2, point.y() * 1.0 / calibrateMainWindowLength);
                isPointSelected = true;
            }
        }
    }
    emit mouseMovePoint(point);
    update();
}

void MouseEvent::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        QPoint point = event -> pos();
        isDragging = true;
        emit mouseClick(point);
    }
}

void MouseEvent::mouseReleaseEvent(QMouseEvent *event) {
    QPoint point = event -> pos();
    isDragging = false;
    isPointSelected = false;
    emit mouseRelease(point);
}

void MouseEvent::paintEvent(QPaintEvent *event) {
    QLabel::paintEvent(event);
    QPainter painter(this);
    QPen pen;
    pen.setColor(Qt::red);
    pen.setWidth(1);
    painter.setPen(pen);
    painter.drawEllipse(selectedPoint, 3, 3);

    pen.setColor(Qt::white);
    pen.setWidth(1);
    painter.setPen(pen);

    QBrush brush;
    brush.setStyle(Qt::SolidPattern);
    brush.setColor(Qt::white);
    painter.setBrush(brush);

    if(cameraCalibrating == leftImgRaw) {
        for (size_t i = 0; i < 4; i++) {
            painter.drawEllipse(leftPoints[i], 3, 3);
            //add label to the point
            painter.drawText(leftPoints[i].x(), leftPoints[i].y(), QString::number(i + 1));
        }
        brush.setStyle(Qt::Dense7Pattern);
        painter.setBrush(brush);
        painter.drawConvexPolygon(leftPoints, 4);
    } else if(cameraCalibrating == rightImgRaw) {
        for (size_t i = 0; i < 4; i++) {
            painter.drawEllipse(rightPoints[i], 3, 3);
            
            painter.drawText(rightPoints[i].x(), rightPoints[i].y(), QString::number(i + 1));
        }
        brush.setStyle(Qt::Dense7Pattern);
        painter.setBrush(brush);
        painter.drawConvexPolygon(rightPoints, 4);
    }

}

MouseEvent::MouseEvent(QWidget *parent) : QLabel{parent} {
    setMouseTracking(true);
    selectedPoint = QPoint(0, 0);
    isDragging = false;
    isPointSelected = false;
}
