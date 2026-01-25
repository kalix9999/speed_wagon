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
    connectToHost("172.20.10.2");

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
    socket->connectToHost(host, 60000);
    if(socket->waitForConnected(3000)) {
        ui->connect_state->setText("서버 연결 성공!");
    } else {
        ui->connect_state->setText("연결 실패...");
    }
}

void Client::onReadyRead()
{
    // 1. 들어온 데이터를 무조건 버퍼 뒤에 이어 붙입니다.
    buffer.append(socket->readAll());
    if (buffer.size() >= 16) {
        qDebug() << "현재 버퍼 앞 16바이트: " << buffer.left(16).toHex();
    }

    // 2. 처리할 수 있는 패킷이 남아있는 동안 계속 돕니다.
    while (true)
    {
        // [상태 A] 아직 헤더를 안 읽은 상태라면? -> 헤더부터 읽자!
        if (!isHeaderReceived)
        {
            // 버퍼에 헤더만큼의 데이터도 안 모였으면? -> 다음을 기약하고 종료(Return)
            if (buffer.size() < sizeof(PacketHeader)){
                qDebug()<<"아직 헤더 안옴\n";
                return;
            }
            // 헤더 데이터 복사
            qDebug() << "Raw Hex:" << buffer.left(16).toHex();
            memcpy(&currentHeader, buffer.constData(), sizeof(PacketHeader));

            // ★ 중요: 읽은 헤더는 버퍼에서 즉시 삭제해야 함! (그래야 맨 앞이 바디가 됨)
            buffer.remove(0, sizeof(PacketHeader));

            // 엔디안 변환
            currentHeader.start    = qFromBigEndian(currentHeader.start);
            currentHeader.img_size = qFromBigEndian(currentHeader.img_size);
            currentHeader.speed    = qFromBigEndian(currentHeader.speed);
            currentHeader.checksum = qFromBigEndian(currentHeader.checksum);

            // "나 헤더 읽었음!" 상태 변경 (기억)
            isHeaderReceived = true;
        }

        // [상태 B] 헤더는 읽었고, 이제 이미지(Body)를 기다리는 중
        if (isHeaderReceived)
        {
            // 버퍼에 이미지 크기만큼 데이터가 아직 안 모였으면? -> 더 기다리자 (Return)
            // (isHeaderReceived가 true로 유지되므로, 다음 호출 때는 위쪽 헤더 읽기를 건너뜁니다)
            if (buffer.size() < currentHeader.img_size){
                qDebug()<<"사진 안옴\n";
                qDebug()<<currentHeader.img_size;
                qDebug()<<"\n";
                return;
            }
            // 이미지 데이터 가져오기 (이미 헤더는 지웠으므로 맨 앞부터가 이미지임)
            QByteArray imageData = buffer.left(currentHeader.img_size);

            // ★ 중요: 읽은 이미지는 버퍼에서 삭제
            buffer.remove(0, currentHeader.img_size);

            // 상태 초기화 (다음 사진 받을 준비)
            isHeaderReceived = false;
            QString speed =QString::number(currentHeader.speed);
            QString now = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
            ui->textBrowser->append("속도: "+ speed + "    시간: " + now + '\n');
            // 이미지 표시
            QPixmap pixmap;
            if (pixmap.loadFromData(imageData)) {
                ui->label_image->setPixmap(
                    pixmap.scaled(ui->label_image->size(), Qt::KeepAspectRatio)
                    );
            } else {
                qDebug() << "이미지 디코딩 실패";
            }
        }
    }
}



void Client::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Left) {
        isLeftPressed = true;
        if (!timer->isActive()) timer->start(50);
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
