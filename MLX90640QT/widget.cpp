#include "widget.h"
#include "ui_widget.h"

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    //创建一个QTcpServer对象，用于监听TCP连接
    server = new QTcpServer(this);
    // QPixmap pixmap=QPixmap("D:/MLX90640/008 Rainy Ashville.png").scaled(this->size());
    // QPalette palette;
    // //设置主窗口背景图片
    // palette.setBrush(QPalette::Window,QBrush(pixmap));

    // 创建模型，用于存储24x32的数据
    model = new QStandardItemModel(24, 32, this);  // 24行32列
    //将模型绑定到tableView控件
    ui->tableView->setModel(model);
    for (int i = 0; i < 32; ++i) {
        ui->tableView->setColumnWidth(i, 40);
    }
    //如下代码设置横向表格头的间隔线，有四个方向的间隔线,不需要间隔线的可以设置为0px
    ui->tableWidget->horizontalHeader()->setStyleSheet(
                                            "QHeaderView::section{"
                                            "border-top:0px solid #E5E5E5;"
                                            "border-left:0px solid #E5E5E5;"
                                            "border-right:0.5px solid #E5E5E5;"
                                            "border-bottom: 0.5px solid #E5E5E5;"
                                            "background-color:white;"
                                            "padding:4px;"
                                            "}"
        );

        //如下代码设置横向表格头的间隔线，有四个方向的间隔线,不需要间隔线的可以设置为0px
    ui->tableWidget->verticalHeader()->setStyleSheet(
                "QHeaderView::section{"
                "border-top:0px solid #E5E5E5;"
                "border-left:0px solid #E5E5E5;"
                "border-right:0.5px solid #E5E5E5;"
                "border-bottom: 0.5px solid #E5E5E5;"
                "background-color:white;"
                "padding:4px;"
                "}"
        );

        //如下代码设置列表左上角第0行第0列的那个格子的边框线
    ui->tableWidget->verticalHeader()->setStyleSheet(
                "QTableCornerButton::section{"
                "border-top:0px solid #E5E5E5;"
                "border-left:0px solid #E5E5E5;"
                "border-right:0.5px solid #E5E5E5;"
                "border-bottom: 0.5px solid #E5E5E5;"
                "background-color:white;"
                "}"
        );
    //设置显示平均值的表格QtableWidget
    ui->tableWidget->setRowCount(1);
    ui->tableWidget->setColumnCount(1);
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->tableWidget->setHorizontalHeaderLabels(QStringList() << "温度平均值");

    // 创建QImage对象，用于显示图像
    image = QImage(32*2, 24*2, QImage::Format_RGB888); // Ensure dimensions match

    //创建并设置一个QGraphicsScene对象，用于在graphicsView中显示图像
    ui->graphicsView->setScene(new QGraphicsScene(this));
    ui->graphicsView->scene()->addPixmap(QPixmap::fromImage(image));

    // 连接newConnection信号到槽函数
    connect(server, &QTcpServer::newConnection, this, &Widget::newConnection);

    comboBox = ui->comboBox;
    comboBox->addItem("Pseudo Color 1", GCM_Pseudo1);
    comboBox->addItem("Pseudo Color 2", GCM_Pseudo2);
    comboBox->addItem("Metal 1", GCM_Metal1);
    comboBox->addItem("Metal 2", GCM_Metal2);
    comboBox->addItem("Rainbow 1", GCM_Rainbow1);
    comboBox->addItem("Rainbow 2", GCM_Rainbow2);
    comboBox->addItem("Rainbow 3", GCM_Rainbow3);
    comboBox->addItem("Zhou", GCM_Zhou);
    comboBox->addItem("Ning", GCM_Ning);
    comboBox->addItem("Gray", GCM_Gray);
    converMethod = static_cast<PseudoColorMethod>(comboBox->currentData().toInt());
    connect(comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) {
        converMethod = static_cast<PseudoColorMethod>(comboBox->itemData(index).toInt());
    });

}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_pushButton_clicked()
{
    // 开始监听连接
    if (!server->listen(QHostAddress::Any, 8888)) {
        qDebug() << "服务器启动失败！";
        return;
    }
    qDebug() << "服务器启动成功，等待连接...";
}

void Widget::newConnection()
{
    // 获取新连接的socket
    socket = server->nextPendingConnection();

    // 连接socket的readyRead信号到readData槽函数
    connect(socket, &QTcpSocket::readyRead, this, &Widget::readData);

    qDebug() << "客户端已连接：" << socket->peerAddress().toString();
}

void Widget::readData()
{
    // 检查是否有足够的数据
    if (socket->bytesAvailable() < sizeof(float) * 768) {
        return;
    }

    // 接收768数组数据
    float data[768];
    socket->read(reinterpret_cast<char*>(data), sizeof(data));
    // 处理数据
    conversion(data);
    qDebug() << "数据已接收并处理。";
}

void Widget::conversion(float* data)
{
    float sum = 0.0;
    for (int i = 0; i < 768; ++i) {
        sum += data[i];
    }
    uint8_t grayData[24][32];
    uint8_t upsampleGrayData[24*2][32*2];
    float average = sum / 768;
    // 在TableWidget中显示平均值
    QTableWidgetItem *item = new QTableWidgetItem(QString::number(average));
    ui->tableWidget->setItem(0, 0, item);

    // 将一维数组转换为24x32的二维灰度数组
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 32; ++j) {
            /* 对浮点数进行归一化后拉伸到255, 10C-40C */
            float value_temp = (data[i * 32 + j]);
            value_temp = ( (value_temp - 10)/(40 - 10) ) * 255;

            // 将浮点数转换为灰度值（这里假设浮点数在0-255之间）
            uint8_t grayValue = static_cast<uint8_t>(value_temp);
            grayData[i][j] = grayValue;
        }
    }

    // 双线性插值
    bilinearInterpolate(grayData, upsampleGrayData);

    // 将二维数组填充QImage
    for (int i = 0; i < 24*2; ++i) {
        for (int j = 0; j < 32*2; ++j) {

            model->setItem(i, j, new QStandardItem(QString::number(upsampleGrayData[i][j])));
            uint8_t colorR, colorG, colorB;
            GrayToPseColor(converMethod, upsampleGrayData[i][j], &colorR, &colorG, &colorB);
            // 设置 QImage 的像素
            image.setPixelColor(32*2-1 - j, i, QColor(colorR, colorG, colorB));

        }
    }

    // 更新QGraphicsView中的图像
    ui->graphicsView->scene()->clear();
    ui->graphicsView->scene()->addPixmap(QPixmap::fromImage(image));
    // 设置缩放比例，例如放大2倍
    ui->graphicsView->resetTransform();  // 重置之前的缩放
    ui->graphicsView->scale(9.0, 12.0); // 将图像放大20倍
}

void Widget::GrayToPseColor(uint8_t converMethod,uint8_t grayValue, uint8_t *colorR, uint8_t *colorG, uint8_t *colorB)
{
    switch(converMethod)
{
    case GCM_Pseudo1:
        *colorR=abs(0-grayValue);
        *colorG=abs(127-grayValue);
        *colorB=abs(255-grayValue);
        break;

    case GCM_Pseudo2:
        if( (grayValue>=0) && (grayValue<=63) )
        {
            *colorR=0;
            *colorG=0;
            *colorB=round(grayValue/64*255);
        }
        else if( (grayValue>=64) && (grayValue<=127) )
        {
            *colorR=0;
            *colorG=round((grayValue-64)/64*255);
            *colorB=round((127-grayValue)/64*255);
        }
        else if( (grayValue>=128) && (grayValue<=191) )
        {
            *colorR=round((grayValue-128)/64*255);
            *colorG=255;
            *colorB=0;
        }
        else if( (grayValue>=192) && (grayValue<=255) )
        {
            *colorR=255;
            *colorG=round((255-grayValue)/64*255);
            *colorB=0;
        }
        break;

    case GCM_Metal1:
        if( (grayValue>=0) && (grayValue<=63) )
        {
            *colorR=0;
            *colorG=0;
            *colorB=round(grayValue/64*255);
        }
        else if( (grayValue>=64) && (grayValue<=95) )
        {
            *colorR=round((grayValue-63)/32*127);
            *colorG=round((grayValue-63)/32*127);
            *colorB=255;
        }
        else if( (grayValue>=96) && (grayValue<=127) )
        {
            *colorR=round((grayValue-95)/32*127)+128;
            *colorG=round((grayValue-95)/32*127)+128;
            *colorB=round((127-grayValue)/32*255);
        }
        else if( (grayValue>=128) && (grayValue<=191) )
        {
            *colorR=255;
            *colorG=255;
            *colorB=0;
        }
        else if( (grayValue>=192) && (grayValue<=255) )
        {
            *colorR=255;
            *colorG=255;
            *colorB=round((grayValue-192)/64*255);
        }
        break;

    case GCM_Metal2:
        *colorR=0;*colorG=0;*colorB=0;
        if( (grayValue>=0) && (grayValue<=16) )
        {
            *colorR=0;
        }
        else if( (grayValue>=17) && (grayValue<=140) )
        {
            *colorR=round((grayValue-16)/(140-16)*255);
        }
        else if( (grayValue>=141) && (grayValue<=255) )
        {
            *colorR=255;
        }

        if( (grayValue>=0) && (grayValue<=101) )
        {
            *colorG=0;
        }
        else if( (grayValue>=102) && (grayValue<=218) )
        {
            *colorG=round((grayValue-101)/(218-101)*255);
        }
        else if( (grayValue>=219) && (grayValue<=255) )
        {
            *colorG=255;
        }

        if( (grayValue>=0) && (grayValue<=91) )
        {
            *colorB=28+round((grayValue-0)/(91-0)*100);
        }
        else if( (grayValue>=92) && (grayValue<=120) )
        {
            *colorB=round((120-grayValue)/(120-91)*128);
        }
        else if( (grayValue>=129) && (grayValue<=214) )
        {
            *colorB=0;
        }
        else if( (grayValue>=215) && (grayValue<=255) )
        {
            *colorB=round((grayValue-214)/(255-214)*255);
        }
        break;

    case GCM_Rainbow1:
        if( (grayValue>=0) && (grayValue<=31) )
        {
            *colorR=0;
            *colorG=0;
            *colorB=round(grayValue/32*255);
        }
        else if( (grayValue>=32) && (grayValue<=63) )
        {
            *colorR=0;
            *colorG=round((grayValue-32)/32*255);
            *colorB=255;
        }
        else if( (grayValue>=64) && (grayValue<=95) )
        {
            *colorR=0;
            *colorG=255;
            *colorB=round((95-grayValue)/32*255);
        }
        else if( (grayValue>=96) && (grayValue<=127) )
        {
            *colorR=round((grayValue-96)/32*255);
            *colorG=255;
            *colorB=0;
        }
        else if( (grayValue>=128) && (grayValue<=191) )
        {
            *colorR=255;
            *colorG=round((191-grayValue)/64*255);
            *colorB=0;
        }
        else if( (grayValue>=192) && (grayValue<=255) )
        {
            *colorR=255;
            *colorG=round((grayValue-192)/64*255);//0
            *colorB=round((grayValue-192)/64*255);
        }
        break;

    case GCM_Rainbow2:
        if( (grayValue>=0) && (grayValue<=63) )
        {
            *colorR=0;
            *colorG=round((grayValue-0)/64*255);
            *colorB=255;
        }
        else if( (grayValue>=64) && (grayValue<=95) )
        {
            *colorR=0;
            *colorG=255;
            *colorB=round((95-grayValue)/32*255);
        }
        else if( (grayValue>=96) && (grayValue<=127) )
        {
            *colorR=round((grayValue-96)/32*255);
            *colorG=255;
            *colorB=0;
        }
        else if( (grayValue>=128) && (grayValue<=191) )
        {
            *colorR=255;
            *colorG=round((191-grayValue)/64*255);
            *colorB=0;
        }
        else if( (grayValue>=192) && (grayValue<=255) )
        {
            *colorR=255;
            *colorG=round((grayValue-192)/64*255);
            *colorB=round((grayValue-192)/64*255);
        }
        break;

    case GCM_Rainbow3:
        if( (grayValue>=0) && (grayValue<=51) )
        {
            *colorR=0;
            *colorG=grayValue*5;
            *colorB=255;
        }
        else if( (grayValue>=52) && (grayValue<=102) )
        {
            *colorR=0;
            *colorG=255;
            *colorB=255-(grayValue-51)*5;
        }
        else if( (grayValue>=103) && (grayValue<=153) )
        {
            *colorR=(grayValue-102)*5;
            *colorG=255;
            *colorB=0;
        }
        else if( (grayValue>=154) && (grayValue<=204) )
        {
            *colorR=255;
            *colorG=round(255-128*(grayValue-153)/51);
            *colorB=0;
        }
        else if( (grayValue>=205) && (grayValue<=255) )
        {
            *colorR=255;
            *colorG=round(127-127*(grayValue-204)/51);
            *colorB=0;
        }
        break;

    case GCM_Zhou:
        if( (grayValue>=0) && (grayValue<=63) )
        {
            *colorR=0;
            *colorG=round((64-grayValue)/64*255);
            *colorB=255;
        }
        else if( (grayValue>=64) && (grayValue<=127) )
        {
            *colorR=0;
            *colorG=round((grayValue-64)/64*255);
            *colorB=round((127-grayValue)/64*255);
        }
        else if( (grayValue>=128) && (grayValue<=191) )
        {
            *colorR=round((grayValue-128)/64*255);
            *colorG=255;
            *colorB=0;
        }
        else if( (grayValue>=192) && (grayValue<=255) )
        {
            *colorR=255;
            *colorG=round((255-grayValue)/64*255);
            *colorB=0;
        }
        break;

    case GCM_Ning:
        if ((grayValue>=0) && (grayValue<=63))
        {
            *colorR=0;
            *colorG=254-4*grayValue;
            *colorB=255;
        }
        else if ((grayValue>=64) && (grayValue<=127))
        {
            *colorR=0;
            *colorG=4*grayValue-254;
            *colorB=510-4*grayValue;
        }
        else if ((grayValue>=128) && (grayValue<=191))
        {
            *colorR=4*grayValue-510;
            *colorG=255;
            *colorB=0;
        }
        else if ((grayValue>=192) && (grayValue<=255))
        {
            *colorR=255;
            *colorG=1022-4*grayValue;
            *colorB=0;
        }
        break;

    case GCM_Gray:
        *colorR=grayValue;
        *colorG=grayValue;
        *colorB=grayValue;
        break;

    default:
        break;
    }
}


#include <algorithm> // for std::clamp

void Widget::bilinearInterpolate(const uint8_t grayData[24][32], uint8_t upsampleGrayData[48][64]) {
    int oldRows = 24, oldCols = 32;
    int newRows = 48, newCols = 64;
    double interpolatedValue;
    for (int i = 0; i < newRows; ++i) {
        for (int j = 0; j < newCols; ++j) {
            // 计算原图像的坐标
            double x = i * (oldRows - 1.0) / (newRows - 1.0);
            double y = j * (oldCols - 1.0) / (newCols - 1.0);

            int x1 = static_cast<int>(x);
            int y1 = static_cast<int>(y);
            int x2 = std::min(x1 + 1, oldRows - 1);
            int y2 = std::min(y1 + 1, oldCols - 1);
            if(i==newRows-1){
                if(j==newCols-1){
                    double Q11 = grayData[x1][y1];
                    interpolatedValue = Q11;
                }else{
                    double Q11 = grayData[x1][y1];
                    double Q12 = grayData[x1][y2];
                    interpolatedValue = Q11+(y-y1)*(Q12-Q11);
                }
            }else if(j==newCols-1){
                double Q11 = grayData[x1][y1];
                double Q21 = grayData[x2][y1];
                interpolatedValue = Q11+(x-x1)*(Q21-Q11);
            }else{
            // 获取四个邻近点的灰度值
            double Q11 = grayData[x1][y1];
            double Q12 = grayData[x1][y2];
            double Q21 = grayData[x2][y1];
            double Q22 = grayData[x2][y2];

            // 计算插值
            double R1 = Q11 + (x - x1) * (Q21 - Q11);
            double R2 = Q12 + (x - x1) * (Q22 - Q12);
            interpolatedValue = R1 + (y - y1) * (R2 - R1);
            }
            // 确保插值结果在0到255之间
            upsampleGrayData[i][j] = static_cast<uint8_t>(std::clamp(interpolatedValue, 0.0, 255.0));

        }
    }
}



