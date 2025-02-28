#include "gui_pkg/Painter.h"

void Painter::paintEvent(QPaintEvent *event) {
    QLabel::paintEvent(event);
    QPainter painter(this);
    QPen pen;
    QFont font;
    QBrush brush;
    pen.setColor(Qt::red);
    pen.setWidth(3);
    font.setPointSize(15);
    painter.setFont(font);
    if(tim) {
        brush.setStyle(Qt::Dense7Pattern);
        brush.setColor(Qt::red);
        painter.setBrush(brush);
        //draw roi warn region
        if (roiWarnState & 0x01) {
            painter.drawPolygon(placeLeap_en, 4);
        }
    }
    brush.setStyle(Qt::SolidPattern);
    for (size_t i = 0; i < robotPoints.size(); i++) {
        int id = robotPoints[i].id;
        if (id < 5 || id == 12) {
            pen.setColor(Qt::red);
            brush.setColor(Qt::red);
        } else {
            pen.setColor(Qt::blue);
            brush.setColor(Qt::blue);
        }

        painter.setPen(pen);
        painter.setBrush(brush);
        painter.drawEllipse(robotPoints[i].point, 10, 10);

        if (id != 10 && id != 110) { // not outpost 
            if (id > 100) {
                id -= 100;
            }
            pen.setColor(Qt::white);
            painter.setPen(pen);
            painter.drawText(robotPoints[i].point.x() - 6, robotPoints[i].point.y() + 8, std::to_string(id).c_str());
        }
    }
}

Painter::Painter(QWidget *parent) : QLabel{parent} {
    placeRB1_te[0] = QPoint(0, 316);
    placeRB1_te[1] = QPoint(0, 450);
    placeRB1_te[2] = QPoint(26, 450);
    placeRB1_te[3] = QPoint(26, 316);
    placeRB2_te[0] = QPoint(80, 382); //280 290
    placeRB2_te[1] = QPoint(61, 409); //299 263
    placeRB2_te[2] = QPoint(128, 455); //232 217
    placeRB2_te[3] = QPoint(198, 455); //162 217
    placeRB2_te[4] = QPoint(207, 450); //153 222
    placeRB2_te[5] = QPoint(188, 424); //172 248
    placeRB2_te[6] = QPoint(138, 424); //222 248
    placeRB3_te[0] = QPoint(0, 458); //360 214
    placeRB3_te[1] = QPoint(0, 564); //360 108
    placeRB3_te[2] = QPoint(126, 564); //234 108
    placeRB3_te[3] = QPoint(126, 523); //234 149
    placeRB3_te[4] = QPoint(25, 458); //335 214
    placeOutpose_te[0] = QPoint(254, 356);
    placeOutpose_te[1] = QPoint(254, 446);
    placeOutpose_te[2] = QPoint(331, 446);
    placeOutpose_te[3] = QPoint(331, 356);
    placeLeap_te[0] = QPoint(334, 371);
    placeLeap_te[1] = QPoint(334, 515);
    placeLeap_te[2] = QPoint(360, 515);
    placeLeap_te[3] = QPoint(360, 371);
    placeHitWindMill_te[0] = QPoint(296, 446);
    placeHitWindMill_te[1] = QPoint(296, 487);
    placeHitWindMill_te[2] = QPoint(333, 487);
    placeHitWindMill_te[3] = QPoint(333, 446);

    placeRB1_en[0] = QPoint(334, 266);
    placeRB1_en[1] = QPoint(334, 356);
    placeRB1_en[2] = QPoint(360, 356);
    placeRB1_en[3] = QPoint(360, 266);
    placeRB2_en[0] = QPoint(280, 290);
    placeRB2_en[1] = QPoint(299, 263);
    placeRB2_en[2] = QPoint(232, 217);
    placeRB2_en[3] = QPoint(162, 217);
    placeRB2_en[4] = QPoint(153, 222);
    placeRB2_en[5] = QPoint(172, 248);
    placeRB2_en[6] = QPoint(222, 248);
    placeRB3_en[0] = QPoint(360, 214);
    placeRB3_en[1] = QPoint(360, 108);
    placeRB3_en[2] = QPoint(234, 108);
    placeRB3_en[3] = QPoint(234, 149);
    placeRB3_en[4] = QPoint(335, 214);
    placeOutpose_en[0] = QPoint(29, 226);
    placeOutpose_en[1] = QPoint(29, 316);
    placeOutpose_en[2] = QPoint(106, 316);
    placeOutpose_en[3] = QPoint(106, 226);
    placeLeap_en[0] = QPoint(0, 157);
    placeLeap_en[1] = QPoint(0, 301);
    placeLeap_en[2] = QPoint(28, 301);
    placeLeap_en[3] = QPoint(28, 157);
    placeHitWindMill_en[0] = QPoint(27, 185);
    placeHitWindMill_en[1] = QPoint(27, 227);
    placeHitWindMill_en[2] = QPoint(64, 227);
    placeHitWindMill_en[3] = QPoint(64, 185);

    tim = false;

    roiWarnState = 0x00;
}

void Painter::drawSmallMap(std::vector<RobotPoint>& rp) {
    std::vector<RobotPoint>().swap(robotPoints);
    for (size_t i = 0; i < rp.size(); i++) {
        robotPoints.push_back(rp[i]);
    }
    update();
}

void Painter::drawROI(unsigned short* input) {
    roiWarnState = *input;
}