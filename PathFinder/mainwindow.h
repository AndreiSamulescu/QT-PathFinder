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
#include "custommodal.h"
#include <QMessageBox>

struct ArrowData {
    QPointF startPoint;
    QPointF endPoint;
    double distance;    // Distanța dintre puncte
    double consumption; // Consumul pe acest segment
    QGraphicsItemGroup* visualItem; // Referință la obiectele grafice

    // Constructor pentru inițializare facilă
    ArrowData(QPointF start, QPointF end, double dist, double cons, QGraphicsItemGroup* item)
        : startPoint(start), endPoint(end), distance(dist), consumption(cons), visualItem(item) {}
};

struct ArrowNode{
    int ID;
    QPointF connection;

    ArrowNode(QPointF connection, int ID): connection(connection), ID(ID){}
};

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
    void runAlgorithm();
    void updateCurrentAlgorithm(int index);

private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
    QList<QGraphicsEllipseItem*> selectedNodes;
    QList<QGraphicsLineItem*> arrows;
    QColor selectedColor = Qt::green;
    void setupUi(QMainWindow *MainWindow);
    QList<QPair<QGraphicsLineItem*, QPair<QGraphicsEllipseItem*, QGraphicsEllipseItem*>>> edges;
    int nodeCounter = 1;
    QList<ArrowData> arrowDataList; // Stores all arrow information
    QList<ArrowNode> arrowNodes; // Stores all arrow information
    bool validateNodes(int from, int to);
    void showWarning(const QString &message);
    bool addArrowHead(QGraphicsLineItem* edge, QPointF start, QPointF end);
    bool edgeExists(QGraphicsEllipseItem* node1, QGraphicsEllipseItem* node2);
    void runGenericAlgorithm(int startNode, int endNode);
    void runFordFulkersonAlgorithm();
    void runEdmondsKarpAlgorithm();
    void logDebugMessage(const QString &message);
    void visualizePath(const QList<int> &path);
    void runBFS(int startNode, int endNode);
};

#endif // MAINWINDOW_H
