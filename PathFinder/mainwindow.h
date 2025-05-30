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

struct EdgeAnalysis {
    int from;
    int to;
    int capacity;
    int flow;
    int residual;
    QString type;
};
struct ArrowData {
    QPointF startPoint;
    QPointF endPoint;
    double distance;
    double consumption;
    QGraphicsItemGroup* group;
    int nodeStartID;
    int nodeEndID;

    ArrowData(QPointF s, QPointF e, double d, double c, QGraphicsItemGroup* g, int sID, int eID)
        : startPoint(s), endPoint(e), distance(d), consumption(c), group(g), nodeStartID(sID), nodeEndID(eID) {}
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
    void clearCanvas();


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
    void runFordFulkersonAlgorithm(int startNode, int endNode);
    void showFordFulkersonResults(int startNode, int endNode, int maxFlow,
                                  const QList<QList<int>>& augmentingPaths,
                                  const QList<EdgeAnalysis>& edgeAnalyses);
    void runEdmondsKarpAlgorithm(int startNode, int endNode);
    void showEdmondsKarpResults(int startNode, int endNode, int maxFlow, const QList<QList<int>>& augmentingPaths, const QList<EdgeAnalysis>& edgeAnalyses);
    void runAhujaOrlinScalingAlgorithm(int startNode, int endNode);
    void showAhujaOrlinScalingResults(int startNode, int endNode, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses);
    void runAhujaOrlinShortestPathAlgorithm(int startNode, int endNode);
    void showAhujaOrlinShortestPathResults(int startNode, int endNode, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses);
    void runAhujaOrlinLayeredNetworkAlgorithm(int startNode, int endNode);
    void showAhujaOrlinLayeredNetworkResults(int startNode, int endNode, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses);
    void runGabowScalingAlgorithm(int startNode, int endNode);
    void showGabowScalingResults(int startNode, int endNode, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses);
    void runGenericPreflowAlgorithm(int startNode, int endNode);
    void showGenericPreflowResults(int startNodeId, int endNodeId, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses);
    void runFIFOPreflowAlgorithm(int startNode, int endNode);
    void showFIFOPreflowResults(int startNodeId, int endNodeId, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses);
    void logDebugMessage(const QString &message);
    void visualizePath(const QList<int>& path, const QString& algorithmName, const QVariant& additionalData);
    void runBFS(int startNode, int endNode);
    bool createArrow(QGraphicsLineItem* edge, QPointF start, QPointF end, double distance, double consumption);
    void runRandomizedBFS(int startNode, int endNode, int maxPaths = 5);
    double calculatePathDistance(const QList<int>& path);
    double calculatePathFlow(const QList<int>& path);
    QString getAlgorithmDetails(const QString& algorithmName, const QList<int>& path);
    QMap<QString, QVariant> calculatePathMetrics(const QList<int>& path);
    void highlightPathOnScene(const QList<int>& path);
    void resetSceneColors();
    void highlightResidualGraph(const QVector<QVector<int>>& flow,
                                const QVector<QVector<int>>& capacity,
                                const QVector<QVector<bool>>& originalEdges,
                                int startNode, int endNode);
    void highlightEdge(int from, int to, const QColor& color, int value, const QString& label);
    void highlightNode(int nodeId, const QColor& color);
    void runPriorityPreflowAlgorithm(int startNode, int endNode);
    void runAhujaExcessScalingAlgorithm(int startNode, int endNode);
    void showPriorityPreflowResults(int startNodeId, int endNodeId, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses);
    void showAhujaExcessScalingResults(int startNodeId, int endNodeId, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses);

};

#endif // MAINWINDOW_H
