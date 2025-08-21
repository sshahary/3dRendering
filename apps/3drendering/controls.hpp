#pragma once
#include <QWidget>

class SketchApp;

class SketchWidget : public QWidget {
    Q_OBJECT
public:
    explicit SketchWidget(SketchApp* app, QWidget* parent = nullptr);

protected:
    void paintEvent(QPaintEvent*) override;
    void resizeEvent(QResizeEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void keyPressEvent(QKeyEvent*) override;

private:
    SketchApp* app_;
    bool   drag_ = false;
    bool   ortho_ = false;   // local toggle, no getter needed in SketchApp
    QPoint last_;
};
