#ifndef LOGWINDOW_H
#define LOGWINDOW_H

#include <QPlainTextEdit>
#include <QScrollBar>

class LogWindow : public QPlainTextEdit
{
    Q_OBJECT

public:
    LogWindow(QWidget *parent = 0);
    void appendMessage(const QString& text);

private:
    QFile m_logFile;
};



#endif // LOGWINDOW_H
