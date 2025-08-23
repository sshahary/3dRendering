#include "controls.hpp"
#include <QImage>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QInputDialog>
#include <cmath>
#include "../include/sketch_app.hpp"

SketchWidget::SketchWidget(SketchApp* app, QWidget* parent)
    : QWidget(parent), app_(app) {
    setMinimumSize(600, 600);
    setFocusPolicy(Qt::StrongFocus);
}

void SketchWidget::paintEvent(QPaintEvent*) {
    const auto& px = app_->render();
    QImage img(reinterpret_cast<const uchar*>(px.data()),
               width(), height(), width()*4, QImage::Format_ARGB32);
    QPainter p(this);
    p.drawImage(0, 0, img);
}

void SketchWidget::resizeEvent(QResizeEvent*) {
    app_->resize(width(), height());
    update();
}

void SketchWidget::mousePressEvent(QMouseEvent* e) {
    drag_ = true;
    last_ = e->pos();
}

void SketchWidget::mouseMoveEvent(QMouseEvent* e) {
    if (!drag_) return;
    QPoint d = e->pos() - last_;
    last_ = e->pos();
    app_->orbit(float(d.x()) * 0.3f, float(d.y()) * 0.3f);
    update();
}

void SketchWidget::mouseReleaseEvent(QMouseEvent*) {
    drag_ = false;
}

void SketchWidget::wheelEvent(QWheelEvent* e) {
    double steps = 0.0;
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    if (!e->pixelDelta().isNull())
        steps = e->pixelDelta().y() / 120.0;    // trackpads
    else
        steps = e->angleDelta().y() / 120.0;    // mouse wheels
#else
    steps = e->delta() / 120.0;
#endif
    if (steps != 0.0) {
        const float per = 0.97f;
        app_->dolly(std::pow(per, float(-steps)));
        e->accept();
        update();
    } else {
        QWidget::wheelEvent(e);
    }
}

void SketchWidget::keyPressEvent(QKeyEvent* e) {
    if (e->key() == Qt::Key_Space) {
        app_->cycleVertexColors();
        update();
        return;
    }
    if (e->key() == Qt::Key_O) {
        ortho_ = !ortho_;
        app_->setUseOrtho(ortho_);
        update();
        return;
    }
    if (e->key() == Qt::Key_C) {
        bool okX, okY, okZ;
        double x = QInputDialog::getDouble(this, "Camera X", "x:", -4.0, -1e6, 1e6, 2, &okX);
        if (!okX) return;
        double y = QInputDialog::getDouble(this, "Camera Y", "y:", -3.0, -1e6, 1e6, 2, &okY);
        if (!okY) return;
        double z = QInputDialog::getDouble(this, "Camera Z", "z:",  2.0, -1e6, 1e6, 2, &okZ);
        if (!okZ) return;
        app_->setCameraPosition(float(x), float(y), float(z)); // see step 4
        update();
        return;
    }
    QWidget::keyPressEvent(e);
}
