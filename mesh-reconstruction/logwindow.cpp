#include <logwindow.h>

LogWindow::LogWindow(QWidget *parent) : QPlainTextEdit(parent)
{
    appendMessage("Welcome young padwan");
}

void LogWindow::appendMessage(const QString& text)
{
    this->appendPlainText(text); // Adds the message to the widget
    this->verticalScrollBar()->setValue(this->verticalScrollBar()->maximum()); // Scrolls to the bottom
    this->setReadOnly(true);
}
