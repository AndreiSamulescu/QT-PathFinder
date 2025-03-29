#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QDebug>
#include <QInputDialog>
#include <QMessageBox>
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
    connect(ui->algorithmComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCurrentAlgorithm(int)));
    connect(ui->runAlgorithmButton, SIGNAL(clicked()), this, SLOT(runAlgorithm()));
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

                        // Creăm linia temporar (dar nu o adăugăm în scenă încă)
                        QGraphicsLineItem *edge = new QGraphicsLineItem(QLineF(p1, p2));
                        edge->setPen(QPen(Qt::black, 2));

                        // Încercăm să adăugăm săgeata (poate eșua dacă userul anulează)
                        bool arrowAdded = addArrowHead(edge, p1, p2);

                        if (arrowAdded) {
                            // Dacă totul e OK, adăugăm linia în scenă și în lista de edges
                            scene->addItem(edge);
                            edges.append(qMakePair(edge, QPair<QGraphicsEllipseItem*, QGraphicsEllipseItem*>(selectedNodes[0], selectedNodes[1])));
                        } else {
                            // Dacă userul a anulat sau a introdus valori invalide, ștergem linia
                            delete edge;
                        }
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

bool MainWindow::addArrowHead(QGraphicsLineItem* edge, QPointF start, QPointF end) {
    // Afișează dialogul modal pentru input
    CustomModal modal(this);
    if (modal.exec() != QDialog::Accepted) {
        return false; // Utilizatorul a anulat
    }

    // Validare și conversie input
    bool ok1, ok2;
    double consumption = modal.getValue1().toDouble(&ok1);
    double distance = modal.getValue2().toDouble(&ok2);

    if (!ok1 || !ok2 || consumption < 0 || distance < 0) {
        QMessageBox::warning(this, "Input invalid", "Introduceți valori numerice pozitive");
        return false;
    }

    // Calcule geometrice pentru săgeată
    QLineF line(start, end);
    if (line.length() == 0) return false;

    // Creare săgeată (triunghi)
    QPointF u = (end - start) / line.length();
    QPointF v(-u.y(), u.x());
    double sideLength = 15.0;
    double h = (sqrt(3) / 2) * sideLength;

    QPolygonF arrowHead;
    arrowHead << end
              << (end - u * h + v * sideLength/2)
              << (end - u * h - v * sideLength/2);

    // Creare elemente grafice
    QGraphicsPolygonItem* arrow = scene->addPolygon(arrowHead, QPen(Qt::black), QBrush(Qt::black));
    arrow->setZValue(edge->zValue() + 1);

    // Creare dreptunghi cu text
    QPointF rectPos = start + (end - start) * 0.7;
    QGraphicsRectItem* rect = scene->addRect(-50, -25, 100, 50, QPen(Qt::red), QBrush(Qt::white));
    rect->setPos(rectPos);

    // Adăugare text centrat
    QGraphicsTextItem* text = scene->addText(QString("C: %1\nD: %2").arg(consumption, 0, 'f', 2).arg(distance, 0, 'f', 2));
    QRectF textRect = text->boundingRect();
    text->setPos(rectPos.x() - textRect.width()/2, rectPos.y() - textRect.height()/2);

    // Grupare elemente
    QGraphicsItemGroup* group = scene->createItemGroup({arrow, rect, text});
    group->setFlag(QGraphicsItem::ItemIsSelectable, true);
    group->setFlag(QGraphicsItem::ItemIsMovable, true);

    // Salvare în listă
    arrowDataList.append(ArrowData(start, end, distance, consumption, group));

    return true;
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

void MainWindow::updateCurrentAlgorithm(int index)
{
    // Deocamdată, doar actualizăm titlul ferestrei cu algoritmul curent
    QString algorithmName = ui->algorithmComboBox->itemText(index);
    setWindowTitle("MainWindow - Current Algorithm: " + algorithmName);
}

void MainWindow::runAlgorithm()
{
    if (nodeCounter <= 2) {
        showWarning("Nu există noduri în listă!");
        return;
    }

    bool ok;
    int from = QInputDialog::getInt(this, "Introducere Noduri", "De la:", 1, 1, nodeCounter - 1, 1, &ok);
    if (!ok) return;

    int to = QInputDialog::getInt(this, "Introducere Noduri", "Până la:", 1, 1, nodeCounter - 1, 1, &ok);
    if (!ok) return;

    if (!validateNodes(from, to)) return;

    QString selectedAlgorithm = ui->algorithmComboBox->currentText();

    if (selectedAlgorithm == "Generic BFS") {
        runGenericAlgorithm();
    }
    else if (selectedAlgorithm == "Ford-Fulkerson") {
        runFordFulkersonAlgorithm();
    }
    else if (selectedAlgorithm == "Edmonds-Karp") {
        runEdmondsKarpAlgorithm();
    }
}

void MainWindow::runGenericAlgorithm()
{
    logDebugMessage("Running Generic Algorithm");
    qDebug() << "Running Generic Algorithm";
    // Implementare pentru algoritmul Generic
}

void MainWindow::runFordFulkersonAlgorithm()
{
    logDebugMessage("Running Ford-Fulkerson Algorithm");
    qDebug() << "Running Ford-Fulkerson Algorithm";
    // Implementare pentru algoritmul Ford-Fulkerson
}

void MainWindow::runEdmondsKarpAlgorithm()
{
    logDebugMessage("Running Edmonds-Karp Algorithm");
    qDebug() << "Running Edmonds-Karp Algorithm";
    // Implementare pentru algoritmul Edmonds-Karp
}
void MainWindow::logDebugMessage(const QString &message)
{
    QFile file("debug.log");
    if (file.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream out(&file);
        out << QDateTime::currentDateTime().toString("[yyyy-MM-dd HH:mm:ss] ") << message << "\n";
        file.close();
    }
}

bool MainWindow::validateNodes(int from, int to)
{
    if (nodeCounter < 1) {
        showWarning("Nu există noduri disponibile!");
        return false;
    }

    if (from < 1 || from >= nodeCounter || to < 1 || to >= nodeCounter) {
        showWarning("Unul dintre nodurile introduse nu există!");
        return false;
    }

    return true;
}

void MainWindow::showWarning(const QString &message)
{
    QMessageBox::warning(this, "Eroare", message);
}
