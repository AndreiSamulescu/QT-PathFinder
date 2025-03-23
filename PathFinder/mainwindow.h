#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QColor>
#include "edge.h"
#include "node.h"
#include <QGraphicsScene>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

protected:
    void mousePressEvent(QMouseEvent *event);

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
    QList<QGraphicsEllipseItem*> selectedNodes; // Nodurile selectate
    QList<QGraphicsLineItem*> arrows; // Săgețile între noduri
    QColor selectedColor = Qt::green; // Culoarea verde pentru noduri
    // Store the selected node color
    void setupUi(QMainWindow *MainWindow);
    QList<QPair<QGraphicsLineItem*, QPair<QGraphicsEllipseItem*, QGraphicsEllipseItem*>>> edges;


};
#endif // MAINWINDOW_H
