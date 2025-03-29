#include "custommodal.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>

CustomModal::CustomModal(QWidget *parent) : QDialog(parent) {
    setWindowTitle("Input Values");
    setModal(true);
    setFixedSize(300, 200);

    QVBoxLayout *layout = new QVBoxLayout(this);

    layout->addWidget(new QLabel("Value 1:"));
    value1Edit = new QLineEdit(this);
    layout->addWidget(value1Edit);

    layout->addWidget(new QLabel("Value 2:"));
    value2Edit = new QLineEdit(this);
    layout->addWidget(value2Edit);

    QPushButton *okButton = new QPushButton("OK", this);
    connect(okButton, &QPushButton::clicked, this, &QDialog::accept);
    layout->addWidget(okButton);
}

QString CustomModal::getValue1() const {
    return value1Edit->text();
}

QString CustomModal::getValue2() const {
    return value2Edit->text();
}
