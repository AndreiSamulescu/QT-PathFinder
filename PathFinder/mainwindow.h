#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QColor>
#include "edge.h"
#include "node.h"
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QMouseEvent>
#include <QGraphicsPolygonItem>
#include <QPen>
#include <QBrush>
#include <QtMath>

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

private slots:
    void importData();  // Slot pentru importul datelor

private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
    QList<QGraphicsEllipseItem*> selectedNodes;
    QList<QGraphicsLineItem*> arrows;
    QColor selectedColor = Qt::green;
    void setupUi(QMainWindow *MainWindow);
    QList<QPair<QGraphicsLineItem*, QPair<QGraphicsEllipseItem*, QGraphicsEllipseItem*>>> edges;
    int nodeCounter = 1;

    void addArrowHead(QGraphicsLineItem* edge, QPointF start, QPointF end);
    bool edgeExists(QGraphicsEllipseItem* node1, QGraphicsEllipseItem* node2);
};

#endif // MAINWINDOW_H
