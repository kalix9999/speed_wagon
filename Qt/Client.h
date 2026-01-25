#ifndef CLIENT_H
#define CLIENT_H

#include <QMainWindow>
#include <QtNetwork>
#include <QLabel>
#include <QKeyEvent>
#include <QTimer>
#include <QProcess>
#include <QtEndian>

typedef struct{
    uint32_t start;
    uint32_t img_size;
    uint32_t speed;
    uint64_t time;
    uint32_t checksum;
}PacketHeader;

namespace Ui {
class Client;
}

class Client : public QMainWindow
{
    Q_OBJECT

public:
    explicit Client(QWidget *parent = nullptr);
    ~Client();

public slots:
    void connectToHost(QString host);

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;

private slots:
    void onReadyRead();
    void onTimerTick();

private:
    Ui::Client *ui;
    QTcpSocket *socket;
    QByteArray buffer;
    qint32 imageSize = 0;

    QTimer *timer;
    int current_angle = 90;
    bool isLeftPressed = false;
    bool isRightPressed = false;

    void sendMqttCommand(int angle);
};

#endif // CLIENT_H
