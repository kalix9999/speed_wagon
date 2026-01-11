#ifndef CLIENTPAGE_H
#define CLIENTPAGE_H

#include <QWidget>
#include <QtNetwork>
#include <QLabel>
#include <QKeyEvent>
#include <QTimer>
#include <QProcess>

namespace Ui {
class ClientPage;
}

class Client : public QWidget
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
    Ui::ClientPage *ui;
    QTcpSocket *socket;
    QByteArray buffer;
    qint32 imageSize = 0;

    QTimer *timer;
    int current_angle = 90;
    bool isLeftPressed = false;
    bool isRightPressed = false;

    void sendMqttCommand(int angle);
};

#endif // CLIENTPAGE_H
