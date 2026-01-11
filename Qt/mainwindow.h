#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtNetwork>
#include <QKeyEvent>
#include <QTimer>
#include <QProcess>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // 메뉴바 클릭 시 페이지 전환용
    void on_actionShowCamera_triggered();
    void on_actionShowImageTest_triggered();


protected:


private:
    Ui::MainWindow *ui;
};
#endif
