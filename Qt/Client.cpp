#include "Client.h"
#include "ui_Client.h"
#include <QPixmap>
#include <QDebug>

Client::Client(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Client)
{
    ui->setupUi(this);

    socket = new QTcpSocket(this);
    connect(socket, &QTcpSocket::readyRead, this, &Client::onReadyRead);
    connectToHost("192.168.0.23");

    this->setFocusPolicy(Qt::StrongFocus);


    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Client::onTimerTick);
}

Client::~Client()
{
    delete ui;
}

void Client::connectToHost(QString host)
{
    socket->connectToHost(host, 8080);
    if(socket->waitForConnected(3000)) {
        ui->textBrowser->append("서버 연결 성공!");
    } else {
        ui->textBrowser->append("연결 실패...");
    }
}

void Client::onReadyRead()
{

    buffer.append(socket->readAll());
    while (true)
    {
        if (imageSize == 0)
        {
            if (buffer.size() < 4) return;
            QDataStream stream(buffer);
            stream >> imageSize;
            buffer.remove(0, 4);
        }
        if (buffer.size() < imageSize) return;

        QByteArray imageData = buffer.mid(0, imageSize);
        buffer.remove(0, imageSize);
        imageSize = 0;

        QPixmap pixmap;
        if (pixmap.loadFromData(imageData)) {
            ui->label_image->setPixmap(pixmap.scaled(ui->label_image->size(), Qt::KeepAspectRatio));

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
