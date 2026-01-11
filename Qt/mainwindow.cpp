#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "ClientPage.h"
#include "testimage.h"
#include <QPixmap>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Designer에서 만든 빈 페이지들을 먼저 제거
    while(ui->stackedWidget->count() > 0) {
        QWidget* widget = ui->stackedWidget->widget(0);
        ui->stackedWidget->removeWidget(widget);
    }

    // 각 페이지 객체 생성
    Client *clientPage = new Client(this);
    TestImage *testImagePage = new TestImage(this);

    ui->stackedWidget->addWidget(clientPage); // Index 0
    ui->stackedWidget->addWidget(testImagePage); // Index 1
}

MainWindow::~MainWindow() { delete ui; }

// 페이지 전환 로직
void MainWindow::on_actionShowCamera_triggered() {
    ui->stackedWidget->setCurrentIndex(0); // 카메라 페이지
}

void MainWindow::on_actionShowImageTest_triggered() {
    ui->stackedWidget->setCurrentIndex(1); // 이미지 테스트 페이지
}
