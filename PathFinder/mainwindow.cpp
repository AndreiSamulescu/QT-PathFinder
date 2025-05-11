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
#include <QQueue>
#include <algorithm>
#include <ctime>
#include <algorithm>
#include <random>
#include <chrono>
#include <QTextEdit>
#include <qboxlayout.h>
#include <QTableWidget>
#include <QHeaderView>
#include <QStack>

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
    connect(ui->clearCanvasButton, &QPushButton::clicked, this, &MainWindow::clearCanvas);

}
bool MainWindow::createArrow(QGraphicsLineItem* edge, QPointF start, QPointF end, double distance, double consumption) {
    // Găsim ID-urile nodurilor sursă și destinație
    int fromId = -1;
    int toId = -1;

    // Caută nodul de start (fromId)
    for (const ArrowNode& node : arrowNodes) {
        if (QLineF(node.connection, start).length() < 5.0) {  // Toleranță mică pentru poziție
            fromId = node.ID;
            break;
        }
    }

    // Caută nodul de final (toId)
    for (const ArrowNode& node : arrowNodes) {
        if (QLineF(node.connection, end).length() < 5.0) {
            toId = node.ID;
            break;
        }
    }

    if (fromId == -1 || toId == -1) {
        qDebug() << "Nu s-au găsit ID-urile nodurilor pentru muchie!";
        return false;
    }

    QLineF line(start, end);
    if (line.length() == 0) return false;

    // Crearea săgeții (triunghi)
    QPointF u = (end - start) / line.length();
    QPointF v(-u.y(), u.x());
    double sideLength = 15.0;
    double h = (sqrt(3) / 2) * sideLength;

    QPolygonF arrowHead;
    arrowHead << end
              << (end - u * h + v * sideLength/2)
              << (end - u * h - v * sideLength/2);

    QGraphicsPolygonItem* arrow = scene->addPolygon(arrowHead, QPen(Qt::black), QBrush(Qt::black));
    arrow->setZValue(edge->zValue() + 1);

    // Crearea dreptunghiului cu text
    QPointF rectPos = start + (end - start) * 0.7;
    QGraphicsRectItem* rect = scene->addRect(-50, -25, 100, 50, QPen(Qt::red), QBrush(Qt::white));
    rect->setPos(rectPos);

    QGraphicsTextItem* text = scene->addText(QString("C: %1\nD: %2").arg(consumption, 0, 'f', 2).arg(distance, 0, 'f', 2));
    QRectF textRect = text->boundingRect();
    text->setPos(rectPos.x() - textRect.width()/2, rectPos.y() - textRect.height()/2);

    // Grupare elemente
    QGraphicsItemGroup* group = scene->createItemGroup({arrow, rect, text});
    group->setFlag(QGraphicsItem::ItemIsSelectable, true);
    group->setFlag(QGraphicsItem::ItemIsMovable, true);

    // Salvare date săgeată cu ID-uri corecte
    arrowDataList.append(ArrowData(start, end, distance, consumption, group, fromId, toId));

    return true;
}

void MainWindow::clearCanvas() {
    // Șterge toate elementele din scenă
    scene->clear();

    // Resetează contoarele și listele
    arrowNodes.clear();
    arrowDataList.clear();
    edges.clear();
    selectedNodes.clear();
    nodeCounter = 1;

    qDebug() << "Canvas cleared";
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
            // Crearea cercului
            QGraphicsEllipseItem *circle = scene->addEllipse(circleRect, QPen(Qt::black), QBrush(selectedColor));

            // Creează textul ca item copil al cercului
            QGraphicsTextItem *text = new QGraphicsTextItem(QString::number(nodeCounter), circle);
            text->setFont(QFont("Arial", 10, QFont::Bold));
            text->setDefaultTextColor(Qt::black);

            // Centrare text în interiorul cercului
            QRectF textRect = text->boundingRect();
            text->setPos(circleRect.x() + circleRect.width() / 2 - textRect.width() / 2,
                         circleRect.y() + circleRect.height() / 2 - textRect.height() / 2);

            // Crearea unui obiect ArrowNode cu ID-ul nodului și poziția acestuia
            ArrowNode newNode(scenePos, nodeCounter);
            arrowNodes.append(newNode);  // Adăugăm ArrowNode în lista arrowNodes

            // Crește contorul pentru următorul nod
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

    // Găsim ID-urile nodurilor care formează această săgeată
    int startNodeID = -1;
    int endNodeID = -1;

    // Căutăm nodul de start în arrowNodes
    for (const ArrowNode& node : arrowNodes) {
        if (QLineF(node.connection, start).length() < 5.0) {  // Dacă punctul de conexiune e apropiat
            startNodeID = node.ID;
            break;
        }
    }

    // Căutăm nodul de final în arrowNodes
    for (const ArrowNode& node : arrowNodes) {
        if (QLineF(node.connection, end).length() < 5.0) {  // Dacă punctul de conexiune e apropiat
            endNodeID = node.ID;
            break;
        }
    }

    // Verificăm dacă am găsit nodurile
    if (startNodeID == -1 || endNodeID == -1) {
        qDebug() << "Nu am găsit nodurile corespunzătoare.";
        return false;
    }

    // Salvăm săgeata în lista de date cu ID-urile nodurilor
    arrowDataList.append(ArrowData(start, end, distance, consumption, group, startNodeID, endNodeID));

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
    if (!doc.isObject()) {
        qDebug() << "Format JSON incorect. Se așteaptă un obiect.";
        return;
    }

    QJsonObject root = doc.object();
    QJsonArray nodesArray = root["nodes"].toArray();
    QJsonArray edgesArray = root["edges"].toArray();

    // Procesează nodurile
    for (const QJsonValue &nodeValue : nodesArray) {
        if (!nodeValue.isObject())
            continue;
        QJsonObject nodeObj = nodeValue.toObject();
        int nodeId = nodeObj["id"].toInt();
        qreal x = nodeObj["x"].toDouble();
        qreal y = nodeObj["y"].toDouble();

        qreal radius = 10.0;
        QRectF circleRect(x - radius, y - radius, radius * 2, radius * 2);
        QGraphicsEllipseItem *circle = scene->addEllipse(circleRect, QPen(Qt::black), QBrush(selectedColor));

        // Adaugă text cu ID-ul nodului
        QGraphicsTextItem *text = new QGraphicsTextItem(QString::number(nodeId), circle);
        text->setFont(QFont("Arial", 10, QFont::Bold));
        text->setDefaultTextColor(Qt::black);
        QRectF textRect = text->boundingRect();
        text->setPos(circleRect.x() + circleRect.width()/2 - textRect.width()/2,
                     circleRect.y() + circleRect.height()/2 - textRect.height()/2);

        // Adaugă ArrowNode în listă
        ArrowNode newNode(QPointF(x, y), nodeId);
        arrowNodes.append(newNode);

        // Actualizează nodeCounter
        if (nodeId >= nodeCounter) {
            nodeCounter = nodeId + 1;
        }
    }

    // Procesează muchiile
    for (const QJsonValue &edgeValue : edgesArray) {
        if (!edgeValue.isObject())
            continue;
        QJsonObject edgeObj = edgeValue.toObject();
        int fromId = edgeObj["from"].toInt();
        int toId = edgeObj["to"].toInt();
        double distance = edgeObj["distance"].toDouble();
        double consumption = edgeObj["consumption"].toDouble();

        // Găsește nodurile sursă și destinație după ID
        QGraphicsEllipseItem *fromNode = nullptr;
        QGraphicsEllipseItem *toNode = nullptr;

        for (QGraphicsItem *item : scene->items()) {
            if (item->type() == QGraphicsEllipseItem::Type) {
                QGraphicsEllipseItem *ellipse = qgraphicsitem_cast<QGraphicsEllipseItem*>(item);
                QGraphicsTextItem *textItem = qgraphicsitem_cast<QGraphicsTextItem*>(ellipse->childItems().first());
                if (textItem) {
                    int id = textItem->toPlainText().toInt();
                    if (id == fromId) {
                        fromNode = ellipse;
                    } else if (id == toId) {
                        toNode = ellipse;
                    }
                }
            }
        }

        if (!fromNode || !toNode) {
            qDebug() << "Nu s-au găsit nodurile pentru muchia de la" << fromId << "la" << toId;
            continue;
        }

        // Creează linia între noduri
        QPointF start = fromNode->sceneBoundingRect().center();
        QPointF end = toNode->sceneBoundingRect().center();
        QGraphicsLineItem *edgeLine = new QGraphicsLineItem(QLineF(start, end));
        edgeLine->setPen(QPen(Qt::black, 2));

        // Creează săgeata cu valorile din JSON
        bool success = createArrow(edgeLine, start, end, distance, consumption);
        if (success) {
            scene->addItem(edgeLine);
            edges.append(qMakePair(edgeLine, QPair<QGraphicsEllipseItem*, QGraphicsEllipseItem*>(fromNode, toNode)));
        } else {
            delete edgeLine;
        }
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
        runGenericAlgorithm(from, to);
    }
    else if (selectedAlgorithm == "Ford-Fulkerson") {
        runFordFulkersonAlgorithm(from, to);
    }
    else if (selectedAlgorithm == "Edmonds-Karp") {
        runEdmondsKarpAlgorithm(from, to);
    }
    else if (selectedAlgorithm == "Ahuja-Orlin Scaling") {
        runAhujaOrlinScalingAlgorithm(from, to);
    }
    else if (selectedAlgorithm == "Ahuja-Orlin Shortest Path") {
        runAhujaOrlinShortestPathAlgorithm(from, to);
    }
    else if (selectedAlgorithm == "Ahuja-Orlin Layered Network") {
        runAhujaOrlinLayeredNetworkAlgorithm(from, to);
    }
    else if (selectedAlgorithm == "Gabow Scaling") {
        runGabowScalingAlgorithm(from, to);
    }
    else if (selectedAlgorithm == "Generic Preflow") {
        runGenericPreflowAlgorithm(from, to);
    }
    else if (selectedAlgorithm == "FIFO Preflow") {
        runFIFOPreflowAlgorithm(from, to);
    }
}

void MainWindow::runFordFulkersonAlgorithm(int startNode, int endNode) {
    logDebugMessage("Running Enhanced Ford-Fulkerson Algorithm");

    // Verificare noduri valide
    int numNodes = arrowNodes.size();
    if (startNode < 1 || endNode < 1 || startNode > numNodes || endNode > numNodes) {
        showWarning("Invalid start or end node");
        return;
    }

    // Inițializare structuri de date
    QVector<QVector<int>> capacity(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<int>> flow(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<bool>> originalEdges(numNodes, QVector<bool>(numNodes, false));

    // Populare matrice de capacități
    for (const ArrowData& arrowData : arrowDataList) {
        int startID = arrowData.nodeStartID - 1;
        int endID = arrowData.nodeEndID - 1;
        capacity[startID][endID] = arrowData.distance;
        originalEdges[startID][endID] = true;
    }

    int source = startNode - 1;
    int sink = endNode - 1;
    int maxFlow = 0;
    QList<QList<int>> augmentingPaths;
    QList<EdgeAnalysis> edgeAnalyses;

    // Generator random
    std::random_device rd;
    std::mt19937 rng(rd());

    while (true) {
        // BFS cu randomizare pentru drum augmentator
        QVector<int> parent(numNodes, -1);
        QVector<bool> visited(numNodes, false);
        QQueue<int> queue;
        queue.enqueue(source);
        visited[source] = true;

        bool pathFound = false;
        while (!queue.isEmpty() && !pathFound) {
            int currentNode = queue.dequeue();

            // Colectare și randomizare vecini
            QList<int> neighbors;
            for (int neighbor = 0; neighbor < numNodes; ++neighbor) {
                if (!visited[neighbor] && (capacity[currentNode][neighbor] - flow[currentNode][neighbor] > 0)) {
                    neighbors.append(neighbor);
                }
            }
            std::shuffle(neighbors.begin(), neighbors.end(), rng);

            for (int neighbor : neighbors) {
                parent[neighbor] = currentNode;
                visited[neighbor] = true;
                queue.enqueue(neighbor);

                if (neighbor == sink) {
                    pathFound = true;
                    break;
                }
            }
        }

        if (!pathFound) break;

        // Reconstruire drum și calcul flow
        QList<int> path;
        int pathFlow = INT_MAX;
        int currentNode = sink;

        while (currentNode != source) {
            path.prepend(currentNode + 1);
            int prevNode = parent[currentNode];
            pathFlow = qMin(pathFlow, capacity[prevNode][currentNode] - flow[prevNode][currentNode]);
            currentNode = prevNode;
        }
        path.prepend(source + 1);
        augmentingPaths.append(path);

        // Actualizare fluxuri
        currentNode = sink;
        while (currentNode != source) {
            int prevNode = parent[currentNode];
            flow[prevNode][currentNode] += pathFlow;
            flow[currentNode][prevNode] -= pathFlow;
            currentNode = prevNode;
        }

        maxFlow += pathFlow;
    }

    // Analiză muchii pentru reziduuri
    for (int i = 0; i < numNodes; ++i) {
        for (int j = 0; j < numNodes; ++j) {
            if (originalEdges[i][j] || flow[j][i] > 0) {
                EdgeAnalysis analysis;
                analysis.from = i + 1;
                analysis.to = j + 1;
                analysis.capacity = originalEdges[i][j] ? capacity[i][j] : 0;
                analysis.flow = flow[i][j];
                analysis.residual = originalEdges[i][j] ? capacity[i][j] - flow[i][j] : flow[j][i];

                if (originalEdges[i][j]) {
                    if (flow[i][j] < capacity[i][j]) {
                        analysis.type = "Non-full forward edge";
                    } else {
                        analysis.type = "Saturated edge";
                    }
                } else {
                    if (flow[j][i] > 0) {
                        analysis.type = "Non-empty backward edge";
                    } else {
                        analysis.type = "Residual only";
                    }
                }

                edgeAnalyses.append(analysis);
            }
        }
    }

    // Afișare rezultate
    showFordFulkersonResults(startNode, endNode, maxFlow, augmentingPaths, edgeAnalyses);
    highlightResidualGraph(flow, capacity, originalEdges, startNode, endNode);
}

void MainWindow::highlightNode(int nodeId, const QColor& color) {
    for (const ArrowNode& node : arrowNodes) {
        if (node.ID == nodeId) {
            QPointF pos = node.connection;

            // Caută iteme la acea poziție
            QList<QGraphicsItem*> itemsAtPos = scene->items(pos);
            for (QGraphicsItem* item : itemsAtPos) {
                QGraphicsEllipseItem* ellipse = qgraphicsitem_cast<QGraphicsEllipseItem*>(item);
                if (ellipse) {
                    ellipse->setBrush(QBrush(color));
                    ellipse->setPen(QPen(Qt::black));
                    break;
                }
            }
            break; // odată ce l-am găsit, ieșim din loop
        }
    }
}

void MainWindow::showFordFulkersonResults(int startNode, int endNode, int maxFlow,
                                          const QList<QList<int>>& augmentingPaths,
                                          const QList<EdgeAnalysis>& edgeAnalyses) {
    QDialog* resultsDialog = new QDialog(this);
    resultsDialog->setWindowTitle("Ford-Fulkerson Detailed Analysis");
    resultsDialog->resize(800, 600);

    QVBoxLayout* mainLayout = new QVBoxLayout(resultsDialog);

    // Tab widget pentru organizare
    QTabWidget* tabWidget = new QTabWidget(resultsDialog);

    // Tab-ul cu rezumat
    QWidget* summaryTab = new QWidget();
    QVBoxLayout* summaryLayout = new QVBoxLayout(summaryTab);

    QTextEdit* summaryText = new QTextEdit(summaryTab);
    summaryText->setReadOnly(true);

    QString summary = QString(
                          "╔══════════════════════════════════╗\n"
                          "║      FORD-FULKERSON RESULTS      ║\n"
                          "╚══════════════════════════════════╝\n\n"
                          "● Source Node: %1\n"
                          "● Sink Node: %2\n"
                          "● Maximum Flow: %3\n"
                          "● Augmenting Paths Found: %4\n\n"
                          "╔══════════════════════════════════╗\n"
                          "║          FLOW CHARACTERISTICS    ║\n"
                          "╚══════════════════════════════════╝\n"
                          "Total Nodes: %5\n"
                          "Total Edges: %6\n"
                          "Execution Time: %7 ms\n"
                          ).arg(startNode).arg(endNode).arg(maxFlow).arg(augmentingPaths.size())
                          .arg(arrowNodes.size()).arg(arrowDataList.size()).arg(QDateTime::currentDateTime().time().msec());

    summaryText->setText(summary);
    summaryLayout->addWidget(summaryText);
    tabWidget->addTab(summaryTab, "Summary");

    // Tab-ul cu analiza muchiilor
    QWidget* edgesTab = new QWidget();
    QVBoxLayout* edgesLayout = new QVBoxLayout(edgesTab);

    QTableWidget* edgesTable = new QTableWidget(edgeAnalyses.size(), 6, edgesTab);
    edgesTable->setHorizontalHeaderLabels({"From", "To", "Capacity", "Flow", "Residual", "Type"});
    edgesTable->verticalHeader()->setVisible(false);

    for (int i = 0; i < edgeAnalyses.size(); ++i) {
        const EdgeAnalysis& ea = edgeAnalyses[i];
        edgesTable->setItem(i, 0, new QTableWidgetItem(QString::number(ea.from)));
        edgesTable->setItem(i, 1, new QTableWidgetItem(QString::number(ea.to)));
        edgesTable->setItem(i, 2, new QTableWidgetItem(QString::number(ea.capacity)));
        edgesTable->setItem(i, 3, new QTableWidgetItem(QString::number(ea.flow)));
        edgesTable->setItem(i, 4, new QTableWidgetItem(QString::number(ea.residual)));
        edgesTable->setItem(i, 5, new QTableWidgetItem(ea.type));
    }

    edgesTable->resizeColumnsToContents();
    edgesLayout->addWidget(edgesTable);
    tabWidget->addTab(edgesTab, "Edge Analysis");

    // Tab-ul cu drumuri augmentatoare
    QWidget* pathsTab = new QWidget();
    QVBoxLayout* pathsLayout = new QVBoxLayout(pathsTab);

    QTextEdit* pathsText = new QTextEdit(pathsTab);
    pathsText->setReadOnly(true);

    QString pathsStr = "Augmenting Paths Found:\n";
    for (int i = 0; i < augmentingPaths.size(); ++i) {
        pathsStr += QString("%1. ").arg(i+1);
        for (int node : augmentingPaths[i]) {
            pathsStr += QString::number(node) + " → ";
        }
        pathsStr.chop(3);
        pathsStr += "\n";
    }

    pathsText->setText(pathsStr);
    pathsLayout->addWidget(pathsText);
    tabWidget->addTab(pathsTab, "Augmenting Paths");

    mainLayout->addWidget(tabWidget);

    // Buton de închidere
    QPushButton* closeButton = new QPushButton("Close", resultsDialog);
    connect(closeButton, &QPushButton::clicked, resultsDialog, &QDialog::accept);
    mainLayout->addWidget(closeButton);

    resultsDialog->exec();
    delete resultsDialog;
}

void MainWindow::highlightResidualGraph(const QVector<QVector<int>>& flow,
                                        const QVector<QVector<int>>& capacity,
                                        const QVector<QVector<bool>>& originalEdges,
                                        int startNode, int endNode) {
    resetSceneColors();

    // Highlight noduri speciale
    highlightNode(startNode, Qt::green);  // Sursa
    highlightNode(endNode, Qt::red);      // Sink

    // Highlight muchii
    for (int i = 0; i < flow.size(); ++i) {
        for (int j = 0; j < flow[i].size(); ++j) {
            if (originalEdges[i][j]) {
                // Muchii originale
                if (flow[i][j] > 0) {
                    if (flow[i][j] < capacity[i][j]) {
                        // Muchie forward non-saturată
                        highlightEdge(i+1, j+1, QColor(65, 105, 225), flow[i][j],
                                      QString("F: %1/%2").arg(flow[i][j]).arg(capacity[i][j]));
                    } else {
                        // Muchie saturată
                        highlightEdge(i+1, j+1, Qt::red, flow[i][j], "SATURATED");
                    }
                }
            } else if (flow[j][i] > 0) {
                // Muchie backward
                highlightEdge(j+1, i+1, QColor(255, 165, 0), flow[j][i],
                              QString("B: %1").arg(flow[j][i]));
            }
        }
    }
}

void MainWindow::highlightEdge(int from, int to, const QColor& color, int value, const QString& label) {
    for (auto& edge : edges) {
        QGraphicsTextItem* textFrom = qgraphicsitem_cast<QGraphicsTextItem*>(edge.second.first->childItems().first());
        QGraphicsTextItem* textTo = qgraphicsitem_cast<QGraphicsTextItem*>(edge.second.second->childItems().first());

        if (textFrom && textTo &&
            textFrom->toPlainText().toInt() == from &&
            textTo->toPlainText().toInt() == to) {

            // Setare stil linie
            QPen pen(color, 3);
            if (label.contains("B:")) {
                pen.setStyle(Qt::DotLine);
            }
            edge.first->setPen(pen);

            // Centru pentru etichetă
            QPointF center = (edge.second.first->sceneBoundingRect().center() +
                              edge.second.second->sceneBoundingRect().center()) / 2;

            // Stil font comun
            QFont font;
            font.setPointSize(12);     // mărime mai mare
            font.setBold(true);        // bold

            // Etichetă flow
            QGraphicsSimpleTextItem* flowText = scene->addSimpleText(label);
            flowText->setFont(font);
            flowText->setPos(center);
            flowText->setBrush(QBrush(color));
            flowText->setZValue(1);

            // Etichetă rezidual
            if (label.contains("/")) {
                QStringList parts = label.split('/');
                int residual = parts[1].toInt() - parts[0].toInt();

                QGraphicsSimpleTextItem* resText = scene->addSimpleText(QString("R: %1").arg(residual));
                font.setPointSize(11); // puțin mai mic la reziduu dacă vrei diferențiere
                resText->setFont(font);
                resText->setPos(center.x(), center.y() + 18); // puțin mai jos
                resText->setBrush(QBrush(Qt::darkGray));
                resText->setZValue(1);
            }

            break;
        }
    }
}




void MainWindow::runEdmondsKarpAlgorithm(int startNode, int endNode) {
    logDebugMessage("Running Edmonds-Karp Algorithm");
    qDebug() << "Running Edmonds-Karp Algorithm";

    // Verificăm dacă nodurile startNode și endNode sunt valide
    int numNodes = arrowNodes.size();
    if (startNode < 1 || endNode < 1 || startNode > numNodes || endNode > numNodes) {
        qDebug() << "Invalid start or end node.";
        return;
    }

    // Conversie: Nodurile 1, 2, 3... sunt mapate la indicii 0, 1, 2...
    int source = startNode - 1;  // Nodul sursă (indexat de la 0)
    int sink = endNode - 1;      // Nodul destinație (indexat de la 0)

    // Matricea de capacități și fluxuri
    QVector<QVector<int>> capacity(numNodes, QVector<int>(numNodes, 0));  // Capacitățile pe muchii
    QVector<QVector<int>> flow(numNodes, QVector<int>(numNodes, 0));      // Fluxurile pe muchii

    // Inițializăm matricea de capacități
    for (const ArrowData &arrowData : arrowDataList) {
        int startID = arrowData.nodeStartID - 1;  // Conversie la indicele din listă
        int endID = arrowData.nodeEndID - 1;      // Conversie la indicele din listă
        int cap = arrowData.distance;              // Capacitatea pe muchie

        capacity[startID][endID] = cap;  // Setăm capacitatea pentru muchie
    }

    // Algoritmul Edmonds-Karp
    int maxFlow = 0;  // Fluxul maxim

    // Vom salva drumurile augmentatoare
    QList<QList<int>> augmentingPaths;

    // Căutăm un drum augmentator folosind BFS
    while (true) {
        QVector<int> parent(numNodes, -1);  // Harta părinților pentru reconstrucția drumului
        QVector<bool> visited(numNodes, false);  // Set pentru vizitarea nodurilor
        QQueue<int> queue;

        queue.enqueue(source);
        visited[source] = true;

        // BFS pentru a găsi un drum augmentator
        bool pathFound = false;
        while (!queue.isEmpty()) {
            int currentNode = queue.dequeue();

            // Verificăm vecinii
            for (int neighbor = 0; neighbor < numNodes; ++neighbor) {
                // Dacă există capacitate reziduală și nu am vizitat deja vecinul
                if (!visited[neighbor] && capacity[currentNode][neighbor] - flow[currentNode][neighbor] > 0) {
                    queue.enqueue(neighbor);
                    visited[neighbor] = true;
                    parent[neighbor] = currentNode;

                    if (neighbor == sink) {
                        pathFound = true;
                        break;
                    }
                }
            }

            if (pathFound) break;
        }

        // Dacă nu am găsit niciun drum augmentator, înseamnă că fluxul maxim a fost găsit
        if (!pathFound) break;

        // Găsim capacitatea minimă pe drumul augmentator
        int pathFlow = INT_MAX;
        int currentNode = sink;
        while (currentNode != source) {
            int prevNode = parent[currentNode];
            pathFlow = std::min(pathFlow, capacity[prevNode][currentNode] - flow[prevNode][currentNode]);
            currentNode = prevNode;
        }

        // Actualizăm fluxurile pe drum
        currentNode = sink;
        while (currentNode != source) {
            int prevNode = parent[currentNode];
            flow[prevNode][currentNode] += pathFlow;
            flow[currentNode][prevNode] -= pathFlow;  // Flux invers (pentru fluxul de retur)
            currentNode = prevNode;
        }

        maxFlow += pathFlow;

        // Adăugăm drumul augmentator în lista de drumuri
        QList<int> augmentingPath;
        currentNode = sink;
        while (currentNode != source) {
            augmentingPath.prepend(currentNode + 1);  // Adăugăm nodul în drum (convertim la 1-based)
            currentNode = parent[currentNode];
        }
        augmentingPath.prepend(source + 1);  // Adăugăm și nodul sursă
        augmentingPaths.append(augmentingPath);
    }

    qDebug() << "Max flow from node " << startNode << " to node " << endNode << ": " << maxFlow;

    // Afișăm rezultatele cu drumurile augmentatoare
    QList<EdgeAnalysis> edgeAnalyses;
    for (int i = 0; i < numNodes; ++i) {
        for (int j = 0; j < numNodes; ++j) {
            if (capacity[i][j] > 0 || flow[i][j] != 0) {
                int residual = capacity[i][j] - flow[i][j];
                QString type;

                if (flow[i][j] > 0 && capacity[i][j] == 0) {
                    type = "Backward";
                } else if (capacity[i][j] > 0 && flow[i][j] == capacity[i][j]) {
                    type = "Saturated";
                } else if (capacity[i][j] > 0 && flow[i][j] < capacity[i][j]) {
                    type = "Forward";
                } else {
                    type = "Neutral";
                }

                edgeAnalyses.append({i + 1, j + 1, capacity[i][j], flow[i][j], residual, type});
            }
        }
    }

    // Afișăm rezultatele
    showFordFulkersonResults(startNode, endNode, maxFlow, augmentingPaths, edgeAnalyses);
}

void MainWindow::showEdmondsKarpResults(int startNode, int endNode, int maxFlow, const QList<QList<int>>& augmentingPaths, const QList<EdgeAnalysis>& edgeAnalyses) {
    // Crearea unui QDialog pentru a afisa rezultatele
    QDialog* resultDialog = new QDialog(this);
    resultDialog->setWindowTitle("Edmonds-Karp Algorithm Results");

    // Layout pentru dialog
    QVBoxLayout* layout = new QVBoxLayout(resultDialog);

    // Etichete pentru fluxul maxim
    QLabel* maxFlowLabel = new QLabel("Max Flow: " + QString::number(maxFlow), resultDialog);
    layout->addWidget(maxFlowLabel);

    // Etichete pentru drumurile augmentatoare
    QLabel* augmentingPathsLabel = new QLabel("Augmenting Paths:", resultDialog);
    layout->addWidget(augmentingPathsLabel);

    // Afișăm drumurile augmentatoare
    QString pathsStr;
    for (int i = 0; i < augmentingPaths.size(); ++i) {
        pathsStr += QString("Path %1: ").arg(i + 1);
        for (int node : augmentingPaths[i]) {
            pathsStr += QString::number(node) + " → ";
        }
        pathsStr.chop(3);  // Îndepărtăm ultima săgeată "→"
        pathsStr += "\n";
    }

    QLabel* pathsDisplay = new QLabel(pathsStr, resultDialog);
    layout->addWidget(pathsDisplay);

    // Etichete pentru analiza muchiilor
    QLabel* edgeAnalysisLabel = new QLabel("Edge Analysis:", resultDialog);
    layout->addWidget(edgeAnalysisLabel);

    // Afișăm analiza muchiilor
    QString edgeAnalysisStr;
    for (const EdgeAnalysis& analysis : edgeAnalyses) {
        edgeAnalysisStr += QString("Edge %1 → %2: Capacity = %3, Flow = %4, Residual = %5, Type = %6\n")
                               .arg(analysis.from)
                               .arg(analysis.to)
                               .arg(analysis.capacity)
                               .arg(analysis.flow)
                               .arg(analysis.residual)
                               .arg(analysis.type);
    }

    QLabel* edgeAnalysisDisplay = new QLabel(edgeAnalysisStr, resultDialog);
    layout->addWidget(edgeAnalysisDisplay);

    // Buton de închidere
    QPushButton* closeButton = new QPushButton("Close", resultDialog);
    connect(closeButton, &QPushButton::clicked, resultDialog, &QDialog::accept);
    layout->addWidget(closeButton);

    resultDialog->setLayout(layout);
    resultDialog->exec();
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

void MainWindow::runGenericAlgorithm(int startNode, int endNode)
{
    runRandomizedBFS(startNode, endNode);
}

void MainWindow::runBFS(int startNode, int endNode)
{
    // Verificăm dacă startNode și endNode sunt valabile
    if (startNode == endNode) {
        qDebug() << "Start node and end node are the same.";
        return;
    }

    // Verificăm dacă există noduri în lista arrowNodes
    if (arrowNodes.isEmpty()) {
        qDebug() << "No nodes available.";
        return;
    }

    // Coada pentru BFS
    QQueue<int> queue; // Stocăm ID-urile nodurilor de explorat
    QMap<int, int> parentMap; // Harta de părinți pentru a reconstrui calea
    QSet<int> visitedNodes; // Set pentru a marca nodurile vizitate

    // Adăugăm startNode în coadă
    queue.enqueue(startNode);
    visitedNodes.insert(startNode);

    // BFS
    while (!queue.isEmpty()) {
        int currentNode = queue.dequeue();

        // Verificăm dacă am ajuns la endNode
        if (currentNode == endNode) {
            qDebug() << "Path found from node " << startNode << " to node " << endNode;

            // Reconstruim calea de la endNode la startNode
            QList<int> path;
            while (parentMap.contains(currentNode)) {
                path.prepend(currentNode);
                currentNode = parentMap[currentNode];
            }
            path.prepend(startNode); // Adăugăm și startNode la începutul căii

            // Afișăm calea
            qDebug() << "Path: " << path;
            return;
        }

        // Găsim vecinii nodului curent (conexiuni orientate)
        for (const ArrowData &arrowData : arrowDataList) {
            // Verificăm dacă nodul curent este începutul unei săgeți și nodul final nu a fost vizitat
            if (arrowData.nodeStartID == currentNode && !visitedNodes.contains(arrowData.nodeEndID)) {
                // Adăugăm vecinul în coadă și îl marcăm ca vizitat
                queue.enqueue(arrowData.nodeEndID);
                visitedNodes.insert(arrowData.nodeEndID);
                parentMap[arrowData.nodeEndID] = currentNode; // Setăm părintele
            }
        }
    }

    // Dacă am ajuns aici, înseamnă că nu am găsit o cale
    qDebug() << "No path found between node " << startNode << " and node " << endNode;
}

void MainWindow::runRandomizedBFS(int startNode, int endNode, int maxPaths) {
    if (startNode == endNode) {
        qDebug() << "Start and end nodes are the same.";
        QList<int> path = {startNode};
        visualizePath(path, "Randomized BFS", QVariant());
        return;
    }

    if (arrowNodes.isEmpty()) {
        qDebug() << "No nodes available.";
        return;
    }

    QList<QList<int>> allPaths;
    QList<int> currentPath;
    currentPath.append(startNode);

    // Folosim o coadă pentru a păstra căile parțiale
    QQueue<QList<int>> pathQueue;
    pathQueue.enqueue(currentPath);

    // Inițializare generator random
    std::random_device rd;
    std::mt19937 rng(rd());

    while (!pathQueue.isEmpty() && allPaths.size() < maxPaths) {
        currentPath = pathQueue.dequeue();
        int lastNode = currentPath.last();

        if (lastNode == endNode) {
            allPaths.append(currentPath);
            continue;
        }

        // Colectăm toți vecinii nevizitați
        QList<int> neighbors;
        for (const ArrowData &arrowData : arrowDataList) {
            if (arrowData.nodeStartID == lastNode && !currentPath.contains(arrowData.nodeEndID)) {
                neighbors.append(arrowData.nodeEndID);
            }
        }

        // Amestecăm vecinii pentru randomizare
        std::shuffle(neighbors.begin(), neighbors.end(), rng);

        // Adăugăm căi noi în coadă
        for (int neighbor : neighbors) {
            QList<int> newPath(currentPath);
            newPath.append(neighbor);
            pathQueue.enqueue(newPath);
        }
    }

    // Procesăm rezultatele
    if (allPaths.isEmpty()) {
        qDebug() << "No paths found between node" << startNode << "and node" << endNode;
        QMessageBox::information(this, "No Path Found",
                                 QString("No path exists from node %1 to node %2").arg(startNode).arg(endNode));
    } else {
        qDebug() << "Found" << allPaths.size() << "random paths:";

        // Afișăm fiecare cale găsită
        for (const QList<int> &path : allPaths) {
            // Construim string pentru debug
            QString pathStr;
            for (int node : path) {
                pathStr += QString::number(node) + " -> ";
            }
            pathStr.chop(4);
            qDebug() << pathStr;

            // Afișăm vizual fiecare cale
            visualizePath(path, "Randomized BFS", QVariant::fromValue(calculatePathMetrics(path)));
        }

        // Highlight la prima cale în interfață grafică
        if (!allPaths.isEmpty()) {
            highlightPathOnScene(allPaths.first());
        }
    }
}

// Funcții helper adiționale:

QMap<QString, QVariant> MainWindow::calculatePathMetrics(const QList<int>& path) {
    QMap<QString, QVariant> metrics;
    double totalDistance = 0;
    double totalConsumption = 0;
    int hops = path.size() - 1;

    for (int i = 0; i < path.size() - 1; i++) {
        for (const ArrowData& arrow : arrowDataList) {
            if (arrow.nodeStartID == path[i] && arrow.nodeEndID == path[i+1]) {
                totalDistance += arrow.distance;
                totalConsumption += arrow.consumption;
                break;
            }
        }
    }

    metrics["total_distance"] = totalDistance;
    metrics["total_consumption"] = totalConsumption;
    metrics["hops"] = hops;
    return metrics;
}

void MainWindow::highlightPathOnScene(const QList<int>& path) {
    // Resetăm toate culorile la cele inițiale
    resetSceneColors();

    // Highlight nodurile din cale
    for (int nodeId : path) {
        for (QGraphicsItem* item : scene->items()) {
            if (item->type() == QGraphicsEllipseItem::Type) {
                QGraphicsEllipseItem* ellipse = qgraphicsitem_cast<QGraphicsEllipseItem*>(item);
                QGraphicsTextItem* textItem = qgraphicsitem_cast<QGraphicsTextItem*>(ellipse->childItems().first());
                if (textItem && textItem->toPlainText().toInt() == nodeId) {
                    ellipse->setBrush(QBrush(Qt::yellow));
                    break;
                }
            }
        }
    }

    // Highlight muchiile din cale
    for (int i = 0; i < path.size() - 1; i++) {
        int from = path[i];
        int to = path[i+1];

        for (auto& edge : edges) {
            QGraphicsTextItem* textFrom = qgraphicsitem_cast<QGraphicsTextItem*>(edge.second.first->childItems().first());
            QGraphicsTextItem* textTo = qgraphicsitem_cast<QGraphicsTextItem*>(edge.second.second->childItems().first());

            if (textFrom && textTo &&
                textFrom->toPlainText().toInt() == from &&
                textTo->toPlainText().toInt() == to) {
                edge.first->setPen(QPen(Qt::red, 3));
                break;
            }
        }
    }
}

void MainWindow::resetSceneColors() {
    // Resetăm nodurile
    for (QGraphicsItem* item : scene->items()) {
        if (item->type() == QGraphicsEllipseItem::Type) {
            qgraphicsitem_cast<QGraphicsEllipseItem*>(item)->setBrush(QBrush(selectedColor));
        }
    }

    // Resetăm muchiile
    for (auto& edge : edges) {
        edge.first->setPen(QPen(Qt::black, 2));
    }
}



void MainWindow::visualizePath(const QList<int>& path, const QString& algorithmName, const QVariant& additionalData) {
    // Creăm un dialog modal
    QDialog* pathDialog = new QDialog(this);
    pathDialog->setWindowTitle("Path Details - " + algorithmName);
    pathDialog->setMinimumSize(400, 300);

    QVBoxLayout* layout = new QVBoxLayout(pathDialog);

    // Adăugăm un QTextEdit pentru detalii
    QTextEdit* detailsText = new QTextEdit(pathDialog);
    detailsText->setReadOnly(true);
    detailsText->setFont(QFont("Courier New", 10));

    // Generăm conținutul în funcție de algoritm
    QString content;
    content += "╔══════════════════════════════╗\n";
    content += "║  PATH FINDING ALGORITHM RESULTS  ║\n";
    content += "╚══════════════════════════════╝\n\n";
    QStringList pathStrList;
    for (int node : path) {
        pathStrList << QString::number(node);
    }
    content += "● Path: " + pathStrList.join(" → ") + "\n";
    content += "● Algorithm: " + algorithmName + "\n";
    content += "● Nodes visited: " + QString::number(path.size()) + "\n";

    if (algorithmName == "Ford-Fulkerson" || algorithmName == "Edmonds-Karp") {
        content += "● Max flow: " + additionalData.toString() + "\n";
        content += "● Path flow: " + QString::number(calculatePathFlow(path)) + "\n";
    }
    else {
        content += "● Path length: " + QString::number(path.size()-1) + " hops\n";
        content += "● Total distance: " + QString::number(calculatePathDistance(path)) + "\n";
    }

    content += "\n════════════════════════════\n";
    content += "TECHNICAL DETAILS:\n";
    content += getAlgorithmDetails(algorithmName, path);

    detailsText->setText(content);
    layout->addWidget(detailsText);

    // Buton de închidere
    QPushButton* closeButton = new QPushButton("Close", pathDialog);
    connect(closeButton, &QPushButton::clicked, pathDialog, &QDialog::accept);
    layout->addWidget(closeButton);

    pathDialog->exec();
    delete pathDialog;
}

// Funcții helper:
double MainWindow::calculatePathDistance(const QList<int>& path) {
    double total = 0;
    for (int i = 0; i < path.size()-1; i++) {
        for (const ArrowData& arrow : arrowDataList) {
            if (arrow.nodeStartID == path[i] && arrow.nodeEndID == path[i+1]) {
                total += arrow.distance;
                break;
            }
        }
    }
    return total;
}

double MainWindow::calculatePathFlow(const QList<int>& path) {
    double minFlow = std::numeric_limits<double>::max();
    for (int i = 0; i < path.size()-1; i++) {
        for (const ArrowData& arrow : arrowDataList) {
            if (arrow.nodeStartID == path[i] && arrow.nodeEndID == path[i+1]) {
                minFlow = qMin(minFlow, arrow.distance);
                break;
            }
        }
    }
    return minFlow;
}

QString MainWindow::getAlgorithmDetails(const QString& algorithmName, const QList<int>& path) {
    QString details;

    if (algorithmName == "Ford-Fulkerson") {
        details = "- Uses BFS to find augmenting paths\n";
        details += "- Time complexity: O(E * max_flow)\n";
        details += "- Residual graph updated after each iteration";
    }
    else if (algorithmName == "Edmonds-Karp") {
        details = "- Specialization of Ford-Fulkerson\n";
        details += "- Always finds shortest augmenting path\n";
        details += "- Time complexity: O(V*E²)\n";
        details += "- More efficient than basic Ford-Fulkerson";
    }
    else { // BFS
        details = "- Explores all neighbors at current depth first\n";
        details += "- Time complexity: O(V + E)\n";
        details += "- Guarantees shortest path in unweighted graphs";
    }

    details += "\n\nPATH ANALYSIS:\n";
    for (int i = 0; i < path.size()-1; i++) {
        for (const ArrowData& arrow : arrowDataList) {
            if (arrow.nodeStartID == path[i] && arrow.nodeEndID == path[i+1]) {
                details += QString("%1 → %2: dist=%3, cons=%4\n")
                               .arg(path[i]).arg(path[i+1])
                               .arg(arrow.distance).arg(arrow.consumption);
                break;
            }
        }
    }

    return details;
}

// Implementation of Ahuja-Orlin Scaling Algorithm
void MainWindow::runAhujaOrlinScalingAlgorithm(int startNode, int endNode) {
    logDebugMessage("Running Ahuja-Orlin Scaling Algorithm");

    int numNodes = arrowNodes.size();
    if (startNode < 1 || endNode < 1 || startNode > numNodes || endNode > numNodes) {
        showWarning("Invalid start or end node");
        return;
    }

    // Initialize data structures
    QVector<QVector<int>> capacity(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<int>> flow(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<bool>> originalEdges(numNodes, QVector<bool>(numNodes, false));

    // Populate capacity matrix
    for (const ArrowData& arrowData : arrowDataList) {
        int startID = arrowData.nodeStartID - 1;
        int endID = arrowData.nodeEndID - 1;
        capacity[startID][endID] = arrowData.distance;
        originalEdges[startID][endID] = true;
    }

    int source = startNode - 1;
    int sink = endNode - 1;
    int maxFlow = 0;

    // Find maximum capacity for scaling
    int maxCapacity = 0;
    for (int i = 0; i < numNodes; ++i) {
        for (int j = 0; j < numNodes; ++j) {
            if (capacity[i][j] > maxCapacity) {
                maxCapacity = capacity[i][j];
            }
        }
    }

    // Scaling phases
    for (int delta = 1; delta <= maxCapacity; delta *= 2) {
        while (true) {
            // BFS with delta scaling
            QVector<int> parent(numNodes, -1);
            QVector<bool> visited(numNodes, false);
            QQueue<int> queue;
            queue.enqueue(source);
            visited[source] = true;

            bool pathFound = false;
            while (!queue.isEmpty() && !pathFound) {
                int currentNode = queue.dequeue();

                for (int neighbor = 0; neighbor < numNodes; ++neighbor) {
                    int residual = capacity[currentNode][neighbor] - flow[currentNode][neighbor];
                    if (!visited[neighbor] && residual >= delta) {
                        parent[neighbor] = currentNode;
                        visited[neighbor] = true;
                        queue.enqueue(neighbor);

                        if (neighbor == sink) {
                            pathFound = true;
                            break;
                        }
                    }
                }
            }

            if (!pathFound) break;

            // Reconstruct path and find minimum residual capacity
            int pathFlow = INT_MAX;
            int currentNode = sink;
            while (currentNode != source) {
                int prevNode = parent[currentNode];
                pathFlow = qMin(pathFlow, capacity[prevNode][currentNode] - flow[prevNode][currentNode]);
                currentNode = prevNode;
            }

            // Update flows
            currentNode = sink;
            while (currentNode != source) {
                int prevNode = parent[currentNode];
                flow[prevNode][currentNode] += pathFlow;
                flow[currentNode][prevNode] -= pathFlow;
                currentNode = prevNode;
            }

            maxFlow += pathFlow;
        }
    }

    // Prepare results for visualization
    QList<EdgeAnalysis> edgeAnalyses;
    for (int i = 0; i < numNodes; ++i) {
        for (int j = 0; j < numNodes; ++j) {
            if (originalEdges[i][j] || flow[i][j] != 0) {
                EdgeAnalysis analysis;
                analysis.from = i + 1;
                analysis.to = j + 1;
                analysis.capacity = originalEdges[i][j] ? capacity[i][j] : 0;
                analysis.flow = flow[i][j];
                analysis.residual = originalEdges[i][j] ? capacity[i][j] - flow[i][j] : flow[j][i];

                if (originalEdges[i][j]) {
                    if (flow[i][j] < capacity[i][j]) {
                        analysis.type = "Non-full forward edge";
                    } else {
                        analysis.type = "Saturated edge";
                    }
                } else {
                    if (flow[j][i] > 0) {
                        analysis.type = "Non-empty backward edge";
                    } else {
                        analysis.type = "Residual only";
                    }
                }

                edgeAnalyses.append(analysis);
            }
        }
    }

    // Show results
    showAhujaOrlinScalingResults(startNode, endNode, maxFlow, edgeAnalyses);
    highlightResidualGraph(flow, capacity, originalEdges, startNode, endNode);
}

// Implementation of Ahuja-Orlin Shortest Path Algorithm
void MainWindow::runAhujaOrlinShortestPathAlgorithm(int startNode, int endNode) {
    logDebugMessage("Running Ahuja-Orlin Shortest Path Algorithm");

    int numNodes = arrowNodes.size();
    if (startNode < 1 || endNode < 1 || startNode > numNodes || endNode > numNodes) {
        showWarning("Invalid start or end node");
        return;
    }

    // Initialize data structures
    QVector<QVector<int>> capacity(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<int>> flow(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<bool>> originalEdges(numNodes, QVector<bool>(numNodes, false));

    // Populate capacity matrix
    for (const ArrowData& arrowData : arrowDataList) {
        int startID = arrowData.nodeStartID - 1;
        int endID = arrowData.nodeEndID - 1;
        capacity[startID][endID] = arrowData.distance;
        originalEdges[startID][endID] = true;
    }

    int source = startNode - 1;
    int sink = endNode - 1;
    int maxFlow = 0;

    while (true) {
        // BFS to find shortest augmenting path
        QVector<int> parent(numNodes, -1);
        QVector<int> distance(numNodes, INT_MAX);
        QQueue<int> queue;

        queue.enqueue(source);
        distance[source] = 0;

        bool pathFound = false;
        while (!queue.isEmpty() && !pathFound) {
            int currentNode = queue.dequeue();

            for (int neighbor = 0; neighbor < numNodes; ++neighbor) {
                if (capacity[currentNode][neighbor] - flow[currentNode][neighbor] > 0 &&
                    distance[neighbor] > distance[currentNode] + 1) {
                    distance[neighbor] = distance[currentNode] + 1;
                    parent[neighbor] = currentNode;
                    queue.enqueue(neighbor);

                    if (neighbor == sink) {
                        pathFound = true;
                        break;
                    }
                }
            }
        }

        if (!pathFound) break;

        // Find minimum residual capacity along the path
        int pathFlow = INT_MAX;
        int currentNode = sink;
        while (currentNode != source) {
            int prevNode = parent[currentNode];
            pathFlow = qMin(pathFlow, capacity[prevNode][currentNode] - flow[prevNode][currentNode]);
            currentNode = prevNode;
        }

        // Update flows
        currentNode = sink;
        while (currentNode != source) {
            int prevNode = parent[currentNode];
            flow[prevNode][currentNode] += pathFlow;
            flow[currentNode][prevNode] -= pathFlow;
            currentNode = prevNode;
        }

        maxFlow += pathFlow;
    }

    // Prepare results for visualization
    QList<EdgeAnalysis> edgeAnalyses;
    for (int i = 0; i < numNodes; ++i) {
        for (int j = 0; j < numNodes; ++j) {
            if (originalEdges[i][j] || flow[i][j] != 0) {
                EdgeAnalysis analysis;
                analysis.from = i + 1;
                analysis.to = j + 1;
                analysis.capacity = originalEdges[i][j] ? capacity[i][j] : 0;
                analysis.flow = flow[i][j];
                analysis.residual = originalEdges[i][j] ? capacity[i][j] - flow[i][j] : flow[j][i];

                if (originalEdges[i][j]) {
                    if (flow[i][j] < capacity[i][j]) {
                        analysis.type = "Non-full forward edge";
                    } else {
                        analysis.type = "Saturated edge";
                    }
                } else {
                    if (flow[j][i] > 0) {
                        analysis.type = "Non-empty backward edge";
                    } else {
                        analysis.type = "Residual only";
                    }
                }

                edgeAnalyses.append(analysis);
            }
        }
    }

    // Show results
    showAhujaOrlinShortestPathResults(startNode, endNode, maxFlow, edgeAnalyses);
    highlightResidualGraph(flow, capacity, originalEdges, startNode, endNode);
}

// Implementation of Ahuja-Orlin Layered Network Algorithm
void MainWindow::runAhujaOrlinLayeredNetworkAlgorithm(int startNode, int endNode) {
    logDebugMessage("Running Ahuja-Orlin Layered Network Algorithm");

    int numNodes = arrowNodes.size();
    if (startNode < 1 || endNode < 1 || startNode > numNodes || endNode > numNodes) {
        showWarning("Invalid start or end node");
        return;
    }

    // Initialize data structures
    QVector<QVector<int>> capacity(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<int>> flow(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<bool>> originalEdges(numNodes, QVector<bool>(numNodes, false));

    // Populate capacity matrix
    for (const ArrowData& arrowData : arrowDataList) {
        int startID = arrowData.nodeStartID - 1;
        int endID = arrowData.nodeEndID - 1;
        capacity[startID][endID] = arrowData.distance;
        originalEdges[startID][endID] = true;
    }

    int source = startNode - 1;
    int sink = endNode - 1;
    int maxFlow = 0;

    while (true) {
        // Build layered network using BFS
        QVector<int> level(numNodes, -1);
        QQueue<int> queue;

        queue.enqueue(source);
        level[source] = 0;

        bool sinkReached = false;
        while (!queue.isEmpty() && !sinkReached) {
            int currentNode = queue.dequeue();

            for (int neighbor = 0; neighbor < numNodes; ++neighbor) {
                if (level[neighbor] == -1 && capacity[currentNode][neighbor] - flow[currentNode][neighbor] > 0) {
                    level[neighbor] = level[currentNode] + 1;
                    queue.enqueue(neighbor);

                    if (neighbor == sink) {
                        sinkReached = true;
                    }
                }
            }
        }

        if (level[sink] == -1) break; // No more augmenting paths

        // Find blocking flow in the layered network
        QVector<int> ptr(numNodes, 0);
        while (true) {
            QStack<int> stack;
            stack.push(source);

            bool pathFound = false;
            while (!stack.isEmpty() && !pathFound) {
                int currentNode = stack.top();

                if (currentNode == sink) {
                    pathFound = true;
                    break;
                }

                while (ptr[currentNode] < numNodes) {
                    int neighbor = ptr[currentNode]++;
                    if (level[neighbor] == level[currentNode] + 1 &&
                        capacity[currentNode][neighbor] - flow[currentNode][neighbor] > 0) {
                        stack.push(neighbor);
                        break;
                    }
                }

                if (ptr[currentNode] == numNodes) {
                    stack.pop();
                    if (!stack.isEmpty()) {
                        ptr[stack.top()] = 0;
                    }
                    break;
                }
            }

            if (!pathFound) break;

            // Find minimum residual capacity in the path
            int pathFlow = INT_MAX;
            for (int i = 0; i < stack.size() - 1; ++i) {
                int u = stack[i];
                int v = stack[i+1];
                pathFlow = qMin(pathFlow, capacity[u][v] - flow[u][v]);
            }

            // Update flows
            for (int i = 0; i < stack.size() - 1; ++i) {
                int u = stack[i];
                int v = stack[i+1];
                flow[u][v] += pathFlow;
                flow[v][u] -= pathFlow;
            }

            maxFlow += pathFlow;
        }
    }

    // Prepare results for visualization
    QList<EdgeAnalysis> edgeAnalyses;
    for (int i = 0; i < numNodes; ++i) {
        for (int j = 0; j < numNodes; ++j) {
            if (originalEdges[i][j] || flow[i][j] != 0) {
                EdgeAnalysis analysis;
                analysis.from = i + 1;
                analysis.to = j + 1;
                analysis.capacity = originalEdges[i][j] ? capacity[i][j] : 0;
                analysis.flow = flow[i][j];
                analysis.residual = originalEdges[i][j] ? capacity[i][j] - flow[i][j] : flow[j][i];

                if (originalEdges[i][j]) {
                    if (flow[i][j] < capacity[i][j]) {
                        analysis.type = "Non-full forward edge";
                    } else {
                        analysis.type = "Saturated edge";
                    }
                } else {
                    if (flow[j][i] > 0) {
                        analysis.type = "Non-empty backward edge";
                    } else {
                        analysis.type = "Residual only";
                    }
                }

                edgeAnalyses.append(analysis);
            }
        }
    }

    // Show results
    showAhujaOrlinLayeredNetworkResults(startNode, endNode, maxFlow, edgeAnalyses);
    highlightResidualGraph(flow, capacity, originalEdges, startNode, endNode);
}

// Implementation of Gabow Scaling Algorithm
void MainWindow::runGabowScalingAlgorithm(int startNode, int endNode) {
    logDebugMessage("Running Gabow Scaling Algorithm");

    int numNodes = arrowNodes.size();
    if (startNode < 1 || endNode < 1 || startNode > numNodes || endNode > numNodes) {
        showWarning("Invalid start or end node");
        return;
    }

    // Initialize data structures
    QVector<QVector<int>> capacity(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<int>> flow(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<bool>> originalEdges(numNodes, QVector<bool>(numNodes, false));

    // Populate capacity matrix
    for (const ArrowData& arrowData : arrowDataList) {
        int startID = arrowData.nodeStartID - 1;
        int endID = arrowData.nodeEndID - 1;
        capacity[startID][endID] = arrowData.distance;
        originalEdges[startID][endID] = true;
    }

    int source = startNode - 1;
    int sink = endNode - 1;
    int maxFlow = 0;

    // Find maximum capacity for scaling
    int maxCapacity = 0;
    for (int i = 0; i < numNodes; ++i) {
        for (int j = 0; j < numNodes; ++j) {
            if (capacity[i][j] > maxCapacity) {
                maxCapacity = capacity[i][j];
            }
        }
    }

    // Scaling phases
    for (int delta = 1; delta <= maxCapacity; delta *= 2) {
        while (true) {
            // BFS with delta scaling
            QVector<int> parent(numNodes, -1);
            QVector<bool> visited(numNodes, false);
            QQueue<int> queue;
            queue.enqueue(source);
            visited[source] = true;

            bool pathFound = false;
            while (!queue.isEmpty() && !pathFound) {
                int currentNode = queue.dequeue();

                for (int neighbor = 0; neighbor < numNodes; ++neighbor) {
                    int residual = capacity[currentNode][neighbor] - flow[currentNode][neighbor];
                    if (!visited[neighbor] && residual >= delta) {
                        parent[neighbor] = currentNode;
                        visited[neighbor] = true;
                        queue.enqueue(neighbor);

                        if (neighbor == sink) {
                            pathFound = true;
                            break;
                        }
                    }
                }
            }

            if (!pathFound) break;

            // Reconstruct path and find minimum residual capacity
            int pathFlow = INT_MAX;
            int currentNode = sink;
            while (currentNode != source) {
                int prevNode = parent[currentNode];
                pathFlow = qMin(pathFlow, capacity[prevNode][currentNode] - flow[prevNode][currentNode]);
                currentNode = prevNode;
            }

            // Update flows
            currentNode = sink;
            while (currentNode != source) {
                int prevNode = parent[currentNode];
                flow[prevNode][currentNode] += pathFlow;
                flow[currentNode][prevNode] -= pathFlow;
                currentNode = prevNode;
            }

            maxFlow += pathFlow;
        }
    }

    // Prepare results for visualization
    QList<EdgeAnalysis> edgeAnalyses;
    for (int i = 0; i < numNodes; ++i) {
        for (int j = 0; j < numNodes; ++j) {
            if (originalEdges[i][j] || flow[i][j] != 0) {
                EdgeAnalysis analysis;
                analysis.from = i + 1;
                analysis.to = j + 1;
                analysis.capacity = originalEdges[i][j] ? capacity[i][j] : 0;
                analysis.flow = flow[i][j];
                analysis.residual = originalEdges[i][j] ? capacity[i][j] - flow[i][j] : flow[j][i];

                if (originalEdges[i][j]) {
                    if (flow[i][j] < capacity[i][j]) {
                        analysis.type = "Non-full forward edge";
                    } else {
                        analysis.type = "Saturated edge";
                    }
                } else {
                    if (flow[j][i] > 0) {
                        analysis.type = "Non-empty backward edge";
                    } else {
                        analysis.type = "Residual only";
                    }
                }

                edgeAnalyses.append(analysis);
            }
        }
    }

    // Show results
    showGabowScalingResults(startNode, endNode, maxFlow, edgeAnalyses);
    highlightResidualGraph(flow, capacity, originalEdges, startNode, endNode);
}

// Implementation of Generic Preflow Algorithm
void MainWindow::runGenericPreflowAlgorithm(int startNode, int endNode) {
    logDebugMessage("Running Generic Preflow Algorithm");

    int numNodes = arrowNodes.size();
    if (startNode < 1 || endNode < 1 || startNode > numNodes || endNode > numNodes) {
        showWarning("Invalid start or end node");
        return;
    }

    // Initialize data structures
    QVector<QVector<int>> capacity(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<int>> flow(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<bool>> originalEdges(numNodes, QVector<bool>(numNodes, false));
    QVector<int> height(numNodes, 0);
    QVector<int> excess(numNodes, 0);

    // Populate capacity matrix
    for (const ArrowData& arrowData : arrowDataList) {
        int startID = arrowData.nodeStartID - 1;
        int endID = arrowData.nodeEndID - 1;
        capacity[startID][endID] = arrowData.distance;
        originalEdges[startID][endID] = true;
    }

    int source = startNode - 1;
    int sink = endNode - 1;

    // Initialize preflow
    height[source] = numNodes;
    for (int v = 0; v < numNodes; ++v) {
        if (capacity[source][v] > 0) {
            flow[source][v] = capacity[source][v];
            flow[v][source] = -flow[source][v];
            excess[v] = flow[source][v];
            excess[source] -= flow[source][v];
        }
    }

    // Main loop
    while (true) {
        bool activeFound = false;
        for (int u = 0; u < numNodes; ++u) {
            if (u == source || u == sink || excess[u] <= 0) continue;

            activeFound = true;
            bool pushed = false;

            // Try to push flow to admissible edges
            for (int v = 0; v < numNodes && !pushed; ++v) {
                if (height[u] == height[v] + 1 && capacity[u][v] - flow[u][v] > 0) {
                    int delta = qMin(excess[u], capacity[u][v] - flow[u][v]);
                    flow[u][v] += delta;
                    flow[v][u] -= delta;
                    excess[u] -= delta;
                    excess[v] += delta;
                    pushed = true;
                }
            }

            // If couldn't push, relabel
            if (!pushed) {
                int minHeight = INT_MAX;
                for (int v = 0; v < numNodes; ++v) {
                    if (capacity[u][v] - flow[u][v] > 0) {
                        minHeight = qMin(minHeight, height[v]);
                    }
                }
                if (minHeight != INT_MAX) {
                    height[u] = minHeight + 1;
                }
            }
        }

        if (!activeFound) break;
    }

    // Calculate max flow
    int maxFlow = 0;
    for (int v = 0; v < numNodes; ++v) {
        maxFlow += flow[source][v];
    }

    // Prepare results for visualization
    QList<EdgeAnalysis> edgeAnalyses;
    for (int i = 0; i < numNodes; ++i) {
        for (int j = 0; j < numNodes; ++j) {
            if (originalEdges[i][j] || flow[i][j] != 0) {
                EdgeAnalysis analysis;
                analysis.from = i + 1;
                analysis.to = j + 1;
                analysis.capacity = originalEdges[i][j] ? capacity[i][j] : 0;
                analysis.flow = flow[i][j];
                analysis.residual = originalEdges[i][j] ? capacity[i][j] - flow[i][j] : flow[j][i];

                if (originalEdges[i][j]) {
                    if (flow[i][j] < capacity[i][j]) {
                        analysis.type = "Non-full forward edge";
                    } else {
                        analysis.type = "Saturated edge";
                    }
                } else {
                    if (flow[j][i] > 0) {
                        analysis.type = "Non-empty backward edge";
                    } else {
                        analysis.type = "Residual only";
                    }
                }

                edgeAnalyses.append(analysis);
            }
        }
    }

    // Show results
    showGenericPreflowResults(startNode, endNode, maxFlow, edgeAnalyses);
    highlightResidualGraph(flow, capacity, originalEdges, startNode, endNode);
}

// Implementation of FIFO Preflow Algorithm
void MainWindow::runFIFOPreflowAlgorithm(int startNode, int endNode) {
    logDebugMessage("Running FIFO Preflow Algorithm");

    int numNodes = arrowNodes.size();
    if (startNode < 1 || endNode < 1 || startNode > numNodes || endNode > numNodes) {
        showWarning("Invalid start or end node");
        return;
    }

    // Initialize data structures
    QVector<QVector<int>> capacity(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<int>> flow(numNodes, QVector<int>(numNodes, 0));
    QVector<QVector<bool>> originalEdges(numNodes, QVector<bool>(numNodes, false));
    QVector<int> height(numNodes, 0);
    QVector<int> excess(numNodes, 0);
    QQueue<int> activeNodes;

    // Populate capacity matrix
    for (const ArrowData& arrowData : arrowDataList) {
        int startID = arrowData.nodeStartID - 1;
        int endID = arrowData.nodeEndID - 1;
        capacity[startID][endID] = arrowData.distance;
        originalEdges[startID][endID] = true;
    }

    int source = startNode - 1;
    int sink = endNode - 1;

    // Initialize preflow
    height[source] = numNodes;
    for (int v = 0; v < numNodes; ++v) {
        if (capacity[source][v] > 0) {
            flow[source][v] = capacity[source][v];
            flow[v][source] = -flow[source][v];
            excess[v] = flow[source][v];
            if (v != sink) {
                activeNodes.enqueue(v);
            }
        }
    }

    // Main loop
    while (!activeNodes.isEmpty()) {
        int u = activeNodes.dequeue();

        // Try to push flow to admissible edges
        for (int v = 0; v < numNodes && excess[u] > 0; ++v) {
            if (height[u] == height[v] + 1 && capacity[u][v] - flow[u][v] > 0) {
                int delta = qMin(excess[u], capacity[u][v] - flow[u][v]);
                flow[u][v] += delta;
                flow[v][u] -= delta;
                excess[u] -= delta;
                excess[v] += delta;

                if (v != source && v != sink && excess[v] == delta) {
                    activeNodes.enqueue(v);
                }
            }
        }

        // If still has excess, relabel and requeue
        if (excess[u] > 0) {
            int minHeight = INT_MAX;
            for (int v = 0; v < numNodes; ++v) {
                if (capacity[u][v] - flow[u][v] > 0) {
                    minHeight = qMin(minHeight, height[v]);
                }
            }
            if (minHeight != INT_MAX) {
                height[u] = minHeight + 1;
                activeNodes.enqueue(u);
            }
        }
    }

    // Calculate max flow
    int maxFlow = 0;
    for (int v = 0; v < numNodes; ++v) {
        maxFlow += flow[source][v];
    }

    // Prepare results for visualization
    QList<EdgeAnalysis> edgeAnalyses;
    for (int i = 0; i < numNodes; ++i) {
        for (int j = 0; j < numNodes; ++j) {
            if (originalEdges[i][j] || flow[i][j] != 0) {
                EdgeAnalysis analysis;
                analysis.from = i + 1;
                analysis.to = j + 1;
                analysis.capacity = originalEdges[i][j] ? capacity[i][j] : 0;
                analysis.flow = flow[i][j];
                analysis.residual = originalEdges[i][j] ? capacity[i][j] - flow[i][j] : flow[j][i];

                if (originalEdges[i][j]) {
                    if (flow[i][j] < capacity[i][j]) {
                        analysis.type = "Non-full forward edge";
                    } else {
                        analysis.type = "Saturated edge";
                    }
                } else {
                    if (flow[j][i] > 0) {
                        analysis.type = "Non-empty backward edge";
                    } else {
                        analysis.type = "Residual only";
                    }
                }

                edgeAnalyses.append(analysis);
            }
        }
    }

    // Show results
    showFIFOPreflowResults(startNode, endNode, maxFlow, edgeAnalyses);
    highlightResidualGraph(flow, capacity, originalEdges, startNode, endNode);
}

void MainWindow::showAhujaOrlinScalingResults(int startNodeId, int endNodeId, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses)
{
    QDialog *resultsDialog = new QDialog(this);
    resultsDialog->setWindowTitle("Ahuja-Orlin Scaling Algorithm Results");
    QVBoxLayout *layout = new QVBoxLayout(resultsDialog);

    QTextEdit *resultsText = new QTextEdit(resultsDialog);
    resultsText->setReadOnly(true);
    QString content = QString("Ahuja-Orlin Scaling Algorithm\n\n");
    content += QString("Start Node: %1\nEnd Node: %2\n").arg(startNodeId).arg(endNodeId);
    content += QString("Maximum Flow: %1\n\n").arg(maxFlow);
    content += "Edge Analysis:\n";
    if (edgeAnalyses.isEmpty()) {
        content += "No edge analysis data available.\n";
    } else {
        QTableWidget *edgeTable = new QTableWidget(edgeAnalyses.size(), 6, resultsDialog);
        edgeTable->setHorizontalHeaderLabels({"From", "To", "Capacity", "Flow", "Residual", "Type"});
        for (int i = 0; i < edgeAnalyses.size(); ++i) {
            const EdgeAnalysis& edge = edgeAnalyses[i];
            edgeTable->setItem(i, 0, new QTableWidgetItem(QString::number(edge.from)));
            edgeTable->setItem(i, 1, new QTableWidgetItem(QString::number(edge.to)));
            edgeTable->setItem(i, 2, new QTableWidgetItem(QString::number(edge.capacity)));
            edgeTable->setItem(i, 3, new QTableWidgetItem(QString::number(edge.flow)));
            edgeTable->setItem(i, 4, new QTableWidgetItem(QString::number(edge.residual)));
            edgeTable->setItem(i, 5, new QTableWidgetItem(edge.type));
        }
        edgeTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        layout->addWidget(edgeTable);
    }
    resultsText->setText(content);
    layout->addWidget(resultsText);

    QPushButton *closeButton = new QPushButton("Close", resultsDialog);
    connect(closeButton, &QPushButton::clicked, resultsDialog, &QDialog::accept);
    layout->addWidget(closeButton);

    resultsDialog->exec();
    delete resultsDialog;
}

void MainWindow::showAhujaOrlinShortestPathResults(int startNodeId, int endNodeId, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses)
{
    QDialog *resultsDialog = new QDialog(this);
    resultsDialog->setWindowTitle("Ahuja-Orlin Shortest Path Algorithm Results");
    QVBoxLayout *layout = new QVBoxLayout(resultsDialog);

    QTextEdit *resultsText = new QTextEdit(resultsDialog);
    resultsText->setReadOnly(true);
    QString content = QString("Ahuja-Orlin Shortest Path Algorithm\n\n");
    content += QString("Start Node: %1\nEnd Node: %2\n").arg(startNodeId).arg(endNodeId);
    content += QString("Maximum Flow: %1\n\n").arg(maxFlow);
    content += "Edge Analysis:\n";
    if (edgeAnalyses.isEmpty()) {
        content += "No edge analysis data available.\n";
    } else {
        QTableWidget *edgeTable = new QTableWidget(edgeAnalyses.size(), 6, resultsDialog);
        edgeTable->setHorizontalHeaderLabels({"From", "To", "Capacity", "Flow", "Residual", "Type"});
        for (int i = 0; i < edgeAnalyses.size(); ++i) {
            const EdgeAnalysis& edge = edgeAnalyses[i];
            edgeTable->setItem(i, 0, new QTableWidgetItem(QString::number(edge.from)));
            edgeTable->setItem(i, 1, new QTableWidgetItem(QString::number(edge.to)));
            edgeTable->setItem(i, 2, new QTableWidgetItem(QString::number(edge.capacity)));
            edgeTable->setItem(i, 3, new QTableWidgetItem(QString::number(edge.flow)));
            edgeTable->setItem(i, 4, new QTableWidgetItem(QString::number(edge.residual)));
            edgeTable->setItem(i, 5, new QTableWidgetItem(edge.type));
        }
        edgeTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        layout->addWidget(edgeTable);
    }
    resultsText->setText(content);
    layout->addWidget(resultsText);

    QPushButton *closeButton = new QPushButton("Close", resultsDialog);
    connect(closeButton, &QPushButton::clicked, resultsDialog, &QDialog::accept);
    layout->addWidget(closeButton);

    resultsDialog->exec();
    delete resultsDialog;
}

void MainWindow::showAhujaOrlinLayeredNetworkResults(int startNodeId, int endNodeId, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses)
{
    QDialog *resultsDialog = new QDialog(this);
    resultsDialog->setWindowTitle("Ahuja-Orlin Layered Network Algorithm Results");
    QVBoxLayout *layout = new QVBoxLayout(resultsDialog);

    QTextEdit *resultsText = new QTextEdit(resultsDialog);
    resultsText->setReadOnly(true);
    QString content = QString("Ahuja-Orlin Layered Network Algorithm\n\n");
    content += QString("Start Node: %1\nEnd Node: %2\n").arg(startNodeId).arg(endNodeId);
    content += QString("Maximum Flow: %1\n\n").arg(maxFlow);
    content += "Edge Analysis:\n";
    if (edgeAnalyses.isEmpty()) {
        content += "No edge analysis data available.\n";
    } else {
        QTableWidget *edgeTable = new QTableWidget(edgeAnalyses.size(), 6, resultsDialog);
        edgeTable->setHorizontalHeaderLabels({"From", "To", "Capacity", "Flow", "Residual", "Type"});
        for (int i = 0; i < edgeAnalyses.size(); ++i) {
            const EdgeAnalysis& edge = edgeAnalyses[i];
            edgeTable->setItem(i, 0, new QTableWidgetItem(QString::number(edge.from)));
            edgeTable->setItem(i, 1, new QTableWidgetItem(QString::number(edge.to)));
            edgeTable->setItem(i, 2, new QTableWidgetItem(QString::number(edge.capacity)));
            edgeTable->setItem(i, 3, new QTableWidgetItem(QString::number(edge.flow)));
            edgeTable->setItem(i, 4, new QTableWidgetItem(QString::number(edge.residual)));
            edgeTable->setItem(i, 5, new QTableWidgetItem(edge.type));
        }
        edgeTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        layout->addWidget(edgeTable);
    }
    resultsText->setText(content);
    layout->addWidget(resultsText);

    QPushButton *closeButton = new QPushButton("Close", resultsDialog);
    connect(closeButton, &QPushButton::clicked, resultsDialog, &QDialog::accept);
    layout->addWidget(closeButton);

    resultsDialog->exec();
    delete resultsDialog;
}

void MainWindow::showGabowScalingResults(int startNodeId, int endNodeId, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses)
{
    QDialog *resultsDialog = new QDialog(this);
    resultsDialog->setWindowTitle("Gabow Scaling Algorithm Results");
    QVBoxLayout *layout = new QVBoxLayout(resultsDialog);

    QTextEdit *resultsText = new QTextEdit(resultsDialog);
    resultsText->setReadOnly(true);
    QString content = QString("Gabow Scaling Algorithm\n\n");
    content += QString("Start Node: %1\nEnd Node: %2\n").arg(startNodeId).arg(endNodeId);
    content += QString("Maximum Flow: %1\n\n").arg(maxFlow);
    content += "Edge Analysis:\n";
    if (edgeAnalyses.isEmpty()) {
        content += "No edge analysis data available.\n";
    } else {
        QTableWidget *edgeTable = new QTableWidget(edgeAnalyses.size(), 6, resultsDialog);
        edgeTable->setHorizontalHeaderLabels({"From", "To", "Capacity", "Flow", "Residual", "Type"});
        for (int i = 0; i < edgeAnalyses.size(); ++i) {
            const EdgeAnalysis& edge = edgeAnalyses[i];
            edgeTable->setItem(i, 0, new QTableWidgetItem(QString::number(edge.from)));
            edgeTable->setItem(i, 1, new QTableWidgetItem(QString::number(edge.to)));
            edgeTable->setItem(i, 2, new QTableWidgetItem(QString::number(edge.capacity)));
            edgeTable->setItem(i, 3, new QTableWidgetItem(QString::number(edge.flow)));
            edgeTable->setItem(i, 4, new QTableWidgetItem(QString::number(edge.residual)));
            edgeTable->setItem(i, 5, new QTableWidgetItem(edge.type));
        }
        edgeTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        layout->addWidget(edgeTable);
    }
    resultsText->setText(content);
    layout->addWidget(resultsText);

    QPushButton *closeButton = new QPushButton("Close", resultsDialog);
    connect(closeButton, &QPushButton::clicked, resultsDialog, &QDialog::accept);
    layout->addWidget(closeButton);

    resultsDialog->exec();
    delete resultsDialog;
}

void MainWindow::showGenericPreflowResults(int startNodeId, int endNodeId, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses)
{
    QDialog *resultsDialog = new QDialog(this);
    resultsDialog->setWindowTitle("Generic Preflow-Push Relabel Algorithm Results");
    QVBoxLayout *layout = new QVBoxLayout(resultsDialog);

    QTextEdit *resultsText = new QTextEdit(resultsDialog);
    resultsText->setReadOnly(true);
    QString content = QString("Generic Preflow-Push Relabel Algorithm\n\n");
    content += QString("Start Node: %1\nEnd Node: %2\n").arg(startNodeId).arg(endNodeId);
    content += QString("Maximum Flow: %1\n\n").arg(maxFlow);
    content += "Edge Analysis:\n";
    if (edgeAnalyses.isEmpty()) {
        content += "No edge analysis data available.\n";
    } else {
        QTableWidget *edgeTable = new QTableWidget(edgeAnalyses.size(), 6, resultsDialog);
        edgeTable->setHorizontalHeaderLabels({"From", "To", "Capacity", "Flow", "Residual", "Type"});
        for (int i = 0; i < edgeAnalyses.size(); ++i) {
            const EdgeAnalysis& edge = edgeAnalyses[i];
            edgeTable->setItem(i, 0, new QTableWidgetItem(QString::number(edge.from)));
            edgeTable->setItem(i, 1, new QTableWidgetItem(QString::number(edge.to)));
            edgeTable->setItem(i, 2, new QTableWidgetItem(QString::number(edge.capacity)));
            edgeTable->setItem(i, 3, new QTableWidgetItem(QString::number(edge.flow)));
            edgeTable->setItem(i, 4, new QTableWidgetItem(QString::number(edge.residual)));
            edgeTable->setItem(i, 5, new QTableWidgetItem(edge.type));
        }
        edgeTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        layout->addWidget(edgeTable);
    }
    resultsText->setText(content);
    layout->addWidget(resultsText);

    QPushButton *closeButton = new QPushButton("Close", resultsDialog);
    connect(closeButton, &QPushButton::clicked, resultsDialog, &QDialog::accept);
    layout->addWidget(closeButton);

    resultsDialog->exec();
    delete resultsDialog;
}

void MainWindow::showFIFOPreflowResults(int startNodeId, int endNodeId, int maxFlow, const QList<EdgeAnalysis>& edgeAnalyses)
{
    QDialog *resultsDialog = new QDialog(this);
    resultsDialog->setWindowTitle("FIFO Preflow-Push Relabel Algorithm Results");
    QVBoxLayout *layout = new QVBoxLayout(resultsDialog);

    QTextEdit *resultsText = new QTextEdit(resultsDialog);
    resultsText->setReadOnly(true);
    QString content = QString("FIFO Preflow-Push Relabel Algorithm\n\n");
    content += QString("Start Node: %1\nEnd Node: %2\n").arg(startNodeId).arg(endNodeId);
    content += QString("Maximum Flow: %1\n\n").arg(maxFlow);
    content += "Edge Analysis:\n";
    if (edgeAnalyses.isEmpty()) {
        content += "No edge analysis data available.\n";
    } else {
        QTableWidget *edgeTable = new QTableWidget(edgeAnalyses.size(), 6, resultsDialog);
        edgeTable->setHorizontalHeaderLabels({"From", "To", "Capacity", "Flow", "Residual", "Type"});
        for (int i = 0; i < edgeAnalyses.size(); ++i) {
            const EdgeAnalysis& edge = edgeAnalyses[i];
            edgeTable->setItem(i, 0, new QTableWidgetItem(QString::number(edge.from)));
            edgeTable->setItem(i, 1, new QTableWidgetItem(QString::number(edge.to)));
            edgeTable->setItem(i, 2, new QTableWidgetItem(QString::number(edge.capacity)));
            edgeTable->setItem(i, 3, new QTableWidgetItem(QString::number(edge.flow)));
            edgeTable->setItem(i, 4, new QTableWidgetItem(QString::number(edge.residual)));
            edgeTable->setItem(i, 5, new QTableWidgetItem(edge.type));
        }
        edgeTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        layout->addWidget(edgeTable);
    }
    resultsText->setText(content);
    layout->addWidget(resultsText);

    QPushButton *closeButton = new QPushButton("Close", resultsDialog);
    connect(closeButton, &QPushButton::clicked, resultsDialog, &QDialog::accept);
    layout->addWidget(closeButton);

    resultsDialog->exec();
    delete resultsDialog;
}
