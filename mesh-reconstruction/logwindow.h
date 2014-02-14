#ifndef LOGWINDOW_H
#define LOGWINDOW_H

#include <QPlainTextEdit>
#include <QScrollBar>
#include <QFile>
#include <QTextStream>

class LogWindow : public QPlainTextEdit
{
    Q_OBJECT

public:
    LogWindow(QWidget *parent = 0);
    void appendMessage(const QString& text);
    void saveLogMessage(const QString &saveFile);

private:
    QString logText;
};



#endif // LOGWINDOW_H
