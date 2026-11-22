#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QByteArray>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_btnApri_clicked();
    void on_btnElabora_clicked();
    void on_btnSalva_clicked();

private:
    Ui::MainWindow *ui;
    QString filePath;
    QByteArray dxfData;
    QByteArray processedDxfData;
    bool fileLoaded = false;
    bool processed = false;
};

#endif // MAINWINDOW_H
