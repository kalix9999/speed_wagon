#include "Client.h"
#include "ui_Client.h"
#include <QPixmap>
#include <QDebug>

Client::Client(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Client)
{
    ui->setupUi(this);

    QPixmap pixmap("C:/Users/1-17/Desktop/5407a0a8d4960.png");   // 또는 파일 경로
    ui->img_label->setPixmap(
        pixmap.scaled(ui->img_label->size(),Qt::KeepAspectRatioByExpanding));

    socket = new QTcpSocket(this);
    connect(socket, &QTcpSocket::readyRead, this, &Client::onReadyRead);
    connectToHost("192.168.0.12");

    this->setFocusPolicy(Qt::StrongFocus);


    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Client::onTimerTick);
}

Client::~Client()
{
    delete ui;
}

//TCP 연결 함수
void Client::connectToHost(QString host)
{
    socket->connectToHost(host, 8080);
    if(socket->waitForConnected(3000)) {
        ui->connect_state->QLabel::setText("서버 연결 성공!");
    } else {
        ui->connect_state->QLabel::setText("연결 실패...");
    }
}

//연결 후 함수
void Client::onReadyRead()
{
    PacketHeader temp;
    buffer.append(socket->readAll());
    while (true)
    {
        if(buffer.size()<sizeof(PacketHeader)) {
            qDebug()<<"아직 헤더 안옴";
            return;
        }
        memcpy(&temp, buffer.constData(), sizeof(PacketHeader));

        header.speed = ntohl(temp.speed);
        header.checksum = ntohl(temp.checksum);
        header.img_size = temp.img_size;
        header.start = ntohl(temp.start);

        if(header.start != 0x12345) {
            qDebug()<<"시작 다름";
            return;
        }

        if(buffer.size() < sizeof(PacketHeader)+header.img_size)
        {
             qDebug()<<"아직 사진 안옴";
            qDebug()<<header.img_size;
            return;
        }
        buffer.remove(0,sizeof(PacketHeader));

        QByteArray img = buffer.mid(0,header.img_size);
        buffer.remove(0, header.img_size);

        uint32_t temp_chk = 0;
        for(int i = 0; i<img.size(); i++){
            temp_chk += static_cast<unsigned char>(img[i]);
        }

        if(temp_chk != header.checksum){
            qDebug()<<"사진 다름";
            return;
        }
        QDateTime t = QDateTime::currentDateTime();
        QString dateStr = t.toString("yyyy-MM-dd-hh-mm-ss");
        QPixmap pixmap;
        if (pixmap.loadFromData(img)) {
            ui->label_image->setPixmap(pixmap.scaled(ui->label_image->size(), Qt::KeepAspectRatio));
            ui->textBrowser->setText(
                "=================\n"
                "속도:" + QString::number(header.speed)+"\n" +
                "시간" + dateStr + "\n"
                );
        }

    }
}



void Client::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Left) {
        isLeftPressed = true;
        if (!timer->isActive()) timer->start(50); // 50ms 간격으로 타이머 시작
        qDebug()<<"left!";
    }
    else if (event->key() == Qt::Key_Right) {
        isRightPressed = true;
        if (!timer->isActive()) timer->start(50);
        qDebug() << "right!";
    }

    QMainWindow::keyPressEvent(event);
}


void Client::keyReleaseEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Left) isLeftPressed = false;
    else if (event->key() == Qt::Key_Right) isRightPressed = false;

    if (!isLeftPressed && !isRightPressed) {
        timer->stop();
    }

    QMainWindow::keyReleaseEvent(event);
}


void Client::onTimerTick()
{
    bool changed = false;

    if (isLeftPressed) {
        current_angle -= 2;
        changed = true;
    }
    if (isRightPressed) {
        current_angle += 2;
        changed = true;
    }

    if (current_angle < 0) current_angle = 0;
    if (current_angle > 180) current_angle = 180;

    if (changed) {
        sendMqttCommand(current_angle);
    }
}


void Client::sendMqttCommand(int angle)
{
    QProcess process;
    QString program = "mosquitto_pub";
    QStringList arguments;

    QString pi_ip = "192.168.0.23";

    arguments << "-h" << pi_ip
              << "-t" << "cam/servo/pan"
              << "-m" << QString::number(angle);


    process.start(program, arguments);
    process.waitForFinished(100);

}
