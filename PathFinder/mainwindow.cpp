#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , selectedColor(Qt::red) // Default color
{
    ui->setupUi(this);

    // Switch to second page when Start Simulation is clicked
    connect(ui->pushButton, &QPushButton::clicked, this, [=]() {
        ui->stackedWidget->setCurrentIndex(1); // Move to page_4
    });


}

MainWindow::~MainWindow()
{
    delete ui;
}
