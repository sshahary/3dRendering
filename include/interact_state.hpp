#pragma once
class InteractState {
public:
    InteractState();
    void scroll(float dy);
    void begin(int x,int y);
    void drag(int x,int y);
    void end();
    float scrollY() const; bool rotating() const; float dx() const; float dy() const;
private:
    float sY_; int lastX_, lastY_; float dX_, dY_; bool rot_;
};
