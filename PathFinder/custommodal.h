#ifndef CUSTOMMODAL_H
#define CUSTOMMODAL_H

#include <QDialog>
#include <QLineEdit>

class CustomModal : public QDialog {
    Q_OBJECT
public:
    explicit CustomModal(QWidget *parent = nullptr);
    QString getValue1() const;
    QString getValue2() const;

private:
    QLineEdit *value1Edit;
    QLineEdit *value2Edit;
};

#endif // CUSTOMMODAL_H
