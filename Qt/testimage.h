#ifndef TESTIMAGE_H
#define TESTIMAGE_H

#include <QWidget>

namespace Ui {
class TestImage;
}

class TestImage : public QWidget
{
    Q_OBJECT

public:
    explicit TestImage(QWidget *parent = nullptr);
    ~TestImage();

private:
    Ui::TestImage *ui;
};

#endif // TESTIMAGE_H
