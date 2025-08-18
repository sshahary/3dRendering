
#include <QApplication>
#include <QWidget>
#include <QImage>
#include <QPainter>
#include <QMouseEvent>
#include "mesh_loader.hpp"
#include "sketch_app.hpp"

class SketchWidget : public QWidget {
public:
    explicit SketchWidget(SketchApp* app, QWidget* parent=nullptr): QWidget(parent), app_(app) {
        setMinimumSize(600,600);
    }
protected:
    void paintEvent(QPaintEvent*) override {
        if(!app_) return;
        const auto& px = app_->render();
        QImage img(reinterpret_cast<const uchar*>(px.data()), width(), height(), QImage::Format_ARGB32);
        QPainter p(this); p.fillRect(rect(), Qt::white); p.drawImage(0,0,img);
    }
    void resizeEvent(QResizeEvent*) override { if(app_) app_->resize(width(),height()); }
    void mousePressEvent(QMouseEvent* e) override { dragging_ = true; last_ = e->pos(); }
    void mouseMoveEvent(QMouseEvent* e) override {
        if(!dragging_) return;
        QPoint d = e->pos() - last_;
        last_ = e->pos();
        if(app_) { app_->orbit(d.x()*0.3f, d.y()*0.3f); }
        update();
    }
    void mouseReleaseEvent(QMouseEvent*) override { dragging_ = false; }
    void wheelEvent(QWheelEvent* e) override {
        if(app_) app_->dolly( e->angleDelta().y() > 0 ? 0.9f : 1.1f );
        update();
    }
private:
    SketchApp* app_;
    bool dragging_ = false;
    QPoint last_;
};

int main(int argc, char** argv){
    QApplication a(argc, argv);
    if(argc<2){ qWarning("Usage: sketch-qt <model.obj>"); return 1; }
    MeshLoader loader; if(!loader.loadOBJ(argv[1])){ qWarning("Failed to load OBJ"); return 2; }
    SketchApp app(loader.positions, loader.edges, 1000, 1000);
    SketchWidget w(&app); w.resize(1000,1000); w.show();
    return a.exec();
}
