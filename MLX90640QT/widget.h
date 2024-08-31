#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QTcpSocket>
#include <QTcpServer>
#include <QStandardItemModel>
#include <QGraphicsView>
#include <QImage>
#include <QComboBox>
#define abs(x)      ((x)>0?(x):-(x))
QT_BEGIN_NAMESPACE
namespace Ui {
class Widget;
}
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    void on_pushButton_clicked();
    void newConnection();
    void readData();
    void conversion(float* data);
    void GrayToPseColor(uint8_t converMethod,uint8_t grayValue, uint8_t *colorR,uint8_t *colorG,uint8_t *colorB);
    void bilinearInterpolate(const uint8_t grayData[24][32], uint8_t upsampleGrayData[48][64]);
private:
    Ui::Widget *ui;
    QTcpServer *server;
    QTcpSocket *socket;
    QStandardItemModel *model;
    QImage image;
    QComboBox *comboBox;
    enum PseudoColorMethod {
        GCM_Pseudo1,
        GCM_Pseudo2,
        GCM_Metal1,
        GCM_Metal2,
        GCM_Rainbow1,
        GCM_Rainbow2,
        GCM_Rainbow3,
        GCM_Zhou,
        GCM_Ning,
        GCM_Gray
    };
    PseudoColorMethod converMethod;

};
#endif // WIDGET_H
