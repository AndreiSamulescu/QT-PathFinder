#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
#include <QMouseEvent>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QPen>
#include <QBrush>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , selectedColor(Qt::red)
{
    ui->setupUi(this);

    connect(ui->pushButton, &QPushButton::clicked, this, [=]() {
        ui->stackedWidget->setCurrentIndex(1);
    });

    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    scene->setSceneRect(0, 0, ui->graphicsView->width(), ui->graphicsView->height());
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);

    ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView->setDragMode(QGraphicsView::NoDrag);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    QPointF scenePos = ui->graphicsView->mapToScene(event->pos());
    qreal radius = 10.0;
    QRectF circleRect(scenePos.x() - radius, scenePos.y() - radius, radius * 2, radius * 2);

    if (event->button() == Qt::LeftButton) {
        // Verificăm dacă există deja un nod în apropiere (suprapunere)
        QGraphicsEllipseItem *existingNode = nullptr;
        for (QGraphicsItem *item : scene->items(circleRect)) {
            if (item->type() == QGraphicsEllipseItem::Type) {
                existingNode = qgraphicsitem_cast<QGraphicsEllipseItem*>(item);
                break;
            }
        }

        if (existingNode) {
            // Considerăm click pe un nod existent (selectare)
            if (!selectedNodes.contains(existingNode)) {
                selectedNodes.append(existingNode);
                existingNode->setBrush(QBrush(selectedColor)); // marcăm cu culoarea selectată

                if (selectedNodes.size() == 2) {
                    // Creăm un edge între cele două noduri selectate
                    QPointF p1 = selectedNodes[0]->sceneBoundingRect().center();
                    QPointF p2 = selectedNodes[1]->sceneBoundingRect().center();

                    QGraphicsLineItem *edge = scene->addLine(QLineF(p1, p2), QPen(Qt::black));
                    edges.append(qMakePair(edge, QPair<QGraphicsEllipseItem*, QGraphicsEllipseItem*>(selectedNodes[0], selectedNodes[1])));

                    selectedNodes.clear();
                }
            }
        } else {
            // Nu există nod în apropiere ⇒ adaugăm nod nou
            QGraphicsEllipseItem *circle = scene->addEllipse(circleRect, QPen(Qt::black), QBrush(selectedColor));
        }
    }
    else if (event->button() == Qt::RightButton) {
        QGraphicsItem *item = scene->itemAt(scenePos, QTransform());

        if (item && item->type() == QGraphicsEllipseItem::Type) {
            QGraphicsEllipseItem *clickedNode = qgraphicsitem_cast<QGraphicsEllipseItem*>(item);

            // Ștergem toate liniile legate de acest nod
            for (int i = edges.size() - 1; i >= 0; --i) {
                if (edges[i].second.first == clickedNode || edges[i].second.second == clickedNode) {
                    scene->removeItem(edges[i].first);
                    delete edges[i].first;
                    edges.removeAt(i);
                }
            }

            scene->removeItem(clickedNode);
            delete clickedNode;
        }
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
