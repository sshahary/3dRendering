#include <QApplication>
#include <QWidget>
#include <QImage>
#include <QPainter>
#include <QMouseEvent>
#include <algorithm>
#include <string>
#include <thread>  

#include "mesh_loader.hpp"
#include "sketch_app.hpp"

class SketchWidget : public QWidget {
public:
    explicit SketchWidget(SketchApp* app, QWidget* parent=nullptr): QWidget(parent), app_(app) { setMinimumSize(600,600); }
protected:
    void paintEvent(QPaintEvent*) override {
        const auto& px = app_->render();
        QImage img(reinterpret_cast<const uchar*>(px.data()), width(), height(), width()*4, QImage::Format_ARGB32);
        QPainter p(this); p.drawImage(0,0,img);
    }
    void resizeEvent(QResizeEvent*) override { app_->resize(width(),height()); update(); }
    void mousePressEvent(QMouseEvent* e) override { drag_=true; last_=e->pos(); }
    void mouseMoveEvent(QMouseEvent* e) override { if(!drag_) return; QPoint d=e->pos()-last_; last_=e->pos(); app_->orbit(d.x()*0.3f, d.y()*0.3f); update(); }
    void mouseReleaseEvent(QMouseEvent*) override { drag_=false; }
    void wheelEvent(QWheelEvent* e) override { app_->dolly(e->angleDelta().y()>0?0.9f:1.1f); update(); }
private:
    SketchApp* app_; bool drag_=false; QPoint last_;
};

int main(int argc, char** argv){
    QApplication qapp(argc, argv);
    if(argc<2){ qWarning("Usage: 3drendering <model.obj>"); return 1; }

    MeshLoader ml;
    if(!ml.loadOBJ(argv[1])){ qWarning("Failed to load OBJ"); return 2; }


    // Build app
    SketchApp app(ml.positions, ml.edges, 1000, 1000);
    app.setThreads(std::max(1u, std::thread::hardware_concurrency()));

    SketchWidget w(&app); w.resize(1000,1000); w.show();
    return qapp.exec();
}
