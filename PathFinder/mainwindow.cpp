#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , selectedColor(Qt::red)
    , nodeCounter(1)
{
    ui->setupUi(this);

    connect(ui->pushButton, &QPushButton::clicked, this, [=]() {
        ui->stackedWidget->setCurrentIndex(1);
    });
    connect(ui->importDataButton, &QPushButton::clicked, this, &MainWindow::importData);
    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    scene->setSceneRect(0, 0, ui->graphicsView->width(), ui->graphicsView->height());
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);

    ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView->setDragMode(QGraphicsView::NoDrag);
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    QPointF scenePos = ui->graphicsView->mapToScene(event->pos());
    qreal radius = 10.0;
    QRectF circleRect(scenePos.x() - radius, scenePos.y() - radius, radius * 2, radius * 2);

    if (event->button() == Qt::LeftButton) {
        QGraphicsEllipseItem *existingNode = nullptr;
        for (QGraphicsItem *item : scene->items(circleRect)) {
            if (item->type() == QGraphicsEllipseItem::Type) {
                existingNode = qgraphicsitem_cast<QGraphicsEllipseItem*>(item);
                break;
            }
        }

        if (existingNode) {
            if (!selectedNodes.contains(existingNode)) {
                selectedNodes.append(existingNode);
                existingNode->setBrush(QBrush(Qt::green));

                if (selectedNodes.size() == 2) {
                    if (!edgeExists(selectedNodes[0], selectedNodes[1])) {
                        QPointF p1 = selectedNodes[0]->sceneBoundingRect().center();
                        QPointF p2 = selectedNodes[1]->sceneBoundingRect().center();

                        QGraphicsLineItem *edge = scene->addLine(QLineF(p1, p2), QPen(Qt::black, 2));
                        addArrowHead(edge, p1, p2);
                        edges.append(qMakePair(edge, QPair<QGraphicsEllipseItem*, QGraphicsEllipseItem*>(selectedNodes[0], selectedNodes[1])));
                    }
                    selectedNodes[0]->setBrush(QBrush(selectedColor));
                    selectedNodes[1]->setBrush(QBrush(selectedColor));
                    selectedNodes.clear();
                }
            }
        } else {
            QGraphicsEllipseItem *circle = scene->addEllipse(circleRect, QPen(Qt::black), QBrush(selectedColor));
            // Creează textul ca item copil al cercului
            QGraphicsTextItem *text = new QGraphicsTextItem(QString::number(nodeCounter), circle);
            text->setFont(QFont("Arial", 10, QFont::Bold));
            text->setDefaultTextColor(Qt::black);
            // Centrare text în interiorul cercului
            QRectF textRect = text->boundingRect();
            text->setPos(circleRect.x() + circleRect.width()/2 - textRect.width()/2,
                         circleRect.y() + circleRect.height()/2 - textRect.height()/2);
            nodeCounter++;
        }
    } else if (event->button() == Qt::RightButton) {
        QGraphicsItem *item = scene->itemAt(scenePos, QTransform());

        if (item && item->type() == QGraphicsEllipseItem::Type) {
            QGraphicsEllipseItem *clickedNode = qgraphicsitem_cast<QGraphicsEllipseItem*>(item);
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

void MainWindow::addArrowHead(QGraphicsLineItem* edge, QPointF start, QPointF end) {
    // Calculăm vectorul unitar de la start la end
    QLineF line(start, end);
    double length = line.length();
    if (length == 0) return; // evităm diviziunea cu 0
    QPointF u = (end - start) / length;

    // Calculăm vectorul perpendicular lui u
    QPointF v(-u.y(), u.x());

    // Definim dimensiunile triunghiului echilateral
    double sideLength = 10.0; // lungimea laturii triunghiului
    double h = (sqrt(3) / 2) * sideLength; // înălțimea triunghiului

    // Vârful triunghiului este la 'end'
    // Calculăm centrul bazei triunghiului, care se găsește la o distanță h înapoi de la vârf, pe direcția spre start
    QPointF baseCenter = end - u * h;

    // Jumătate din lățimea bazei (pentru un triunghi echilateral, baza are lungimea sideLength)
    double halfBase = sideLength / 2.0;

    // Calculăm colțurile bazei
    QPointF baseLeft = baseCenter + v * halfBase;
    QPointF baseRight = baseCenter - v * halfBase;

    // Formăm poligonul triunghiului
    QPolygonF arrowHead;
    arrowHead << end << baseLeft << baseRight;

    QGraphicsPolygonItem *arrow = scene->addPolygon(arrowHead, QPen(Qt::black), QBrush(Qt::black));
    arrow->setZValue(edge->zValue() + 1);
}


bool MainWindow::edgeExists(QGraphicsEllipseItem* node1, QGraphicsEllipseItem* node2) {
    for (const auto &edge : edges) {
        if (edge.second.first == node1 && edge.second.second == node2) {
            return true;
        }
    }
    return false;
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::importData() {
    // Deschide Windows Explorer pentru a selecta fișierul JSON
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open JSON File"), "", tr("JSON Files (*.json)"));
    if (fileName.isEmpty())
        return;

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
        qDebug() << "Nu s-a putut deschide fișierul.";
        return;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isArray()) {
        qDebug() << "Format JSON incorect. Se așteaptă un array.";
        return;
    }

    QJsonArray array = doc.array();
    for (const QJsonValue &value : array) {
        this->nodeCounter++;
        if (!value.isObject())
            continue;
        QJsonObject obj = value.toObject();
        int nodeNumber = obj["node"].toInt();
        qreal x = obj["x"].toDouble();
        qreal y = obj["y"].toDouble();

        qreal radius = 10.0;
        QRectF circleRect(x - radius, y - radius, radius * 2, radius * 2);
        QGraphicsEllipseItem *circle = scene->addEllipse(circleRect, QPen(Qt::black), QBrush(selectedColor));

        // Creează textul ca item copil al cercului
        QGraphicsTextItem *text = new QGraphicsTextItem(QString::number(nodeNumber), circle);
        text->setFont(QFont("Arial", 10, QFont::Bold));
        text->setDefaultTextColor(Qt::black);
        QRectF textRect = text->boundingRect();
        text->setPos(circleRect.x() + circleRect.width()/2 - textRect.width()/2,
                     circleRect.y() + circleRect.height()/2 - textRect.height()/2);
    }
}
