#include <logwindow.h>

LogWindow::LogWindow(QWidget *parent) : QPlainTextEdit(parent)
{
    appendMessage("Welcome young padawan. \n");
}

void LogWindow::appendMessage(const QString& text)
{
    this->appendPlainText(text); // Adds the message to the widget
    this->verticalScrollBar()->setValue(this->verticalScrollBar()->maximum());
    this->setReadOnly(true);
    logText = logText + text + "\n";
}

void LogWindow::saveLogMessage(const QString &saveFile)
{
    QFile data(saveFile);
    if (data.open(QFile::WriteOnly | QFile::Truncate)) {
        QTextStream out(&data);
        out << logText;
    }
}
