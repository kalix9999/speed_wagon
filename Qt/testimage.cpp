#include "testimage.h"
#include "ui_testimage.h"

TestImage::TestImage(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::TestImage)
{
    ui->setupUi(this);
}

TestImage::~TestImage()
{
    delete ui;
}
