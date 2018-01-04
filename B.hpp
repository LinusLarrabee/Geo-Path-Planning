//
//  B.hpp
//  无人机路径规划
//
//  Created by 孙浩 on 2018/1/4.
//  Copyright © 2018年 BUAA F632. All rights reserved.
//

#ifndef B_hpp
#define B_hpp
#include <cmath>
#include <ostream>
#include <cstddef>
#include <limits>
#include <vector>
using namespace std;
// B is an aircraft vector, B_ is relative to A;
class B {
private:
    float x_;
    float y_;
public:
    B() : x_(0.0f), y_(0.0f) { }
    B(float x, float y) : x_(x), y_(y) { }
    float x() const { return x_; }
    float y() const { return y_; }
    
    B operator-() const{return B(-x_, -y_);}
    float operator*(const B &B_) const{ return x_ * B_.x() + y_ * B_.y();}
    
    B operator+(const B &B_) const{ return B(x_ + B_.x(), y_ + B_.y());}
    B operator-(const B &B_) const{ return B(x_ - B_.x(), y_ - B_.y());}
    B operator*(float s) const{ return B(x_ * s, y_ * s);}
    B operator/(float s) const{
        const float invS = 1.0f / s;
        return B(x_ * invS, y_ * invS);
    }
    
    bool operator==(const B &B_) const{return x_ == B_.x() && y_ == B_.y();}
    bool operator!=(const B &B_) const{ return x_ != B_.x() || y_ != B_.y();}
    
    
    B &operator+=(const B &B_){
        x_ += B_.x();
        y_ += B_.y();
        return *this;
    }
    B &operator-=(const B &B_){
        x_ -= B_.x();
        y_ -= B_.y();
        return *this;
    }
    B &operator*=(float s){
        x_ *= s;
        y_ *= s;
        return *this;
    }
    B &operator/=(float s)
    {
        const float invS = 1.0f / s;
        x_ *= invS;
        y_ *= invS;
        return *this;
    }
};

B operator*(float s, const B &B_){ return B(s * B_.x(), s * B_.y());}
float abs(const B &B_){ return sqrt(B_ * B_);}
float findnorm(const B &B_){ return B_ * B_;}
float det(const B &A, const B &B){ return A.x() * B.y() - A.y() * B.x();}
B norm(const B &B_){ return B_ / abs(B_);}
float light(const B &a, const B &b, const B &c){ return det(a - c, b - a);}
float sqr(float a){ return a * a;}

ostream &operator<<(ostream &os, const B &B_){
    os << "(" << B_.x() << "," << B_.y() << ")";
    return os;
}

float SignificantArea(const B &a, const B &b, const B &c){
    const float r = ((c - a) * (b - a)) / findnorm(b - a);
    if (r < 0.0f)
        return findnorm(c - a);
    else if (r > 1.0f)
        return findnorm(c - b);
    else return findnorm(c - (a + r * (b - a)));
}
//distDoublePointLineSegment

#endif /* B_hpp */
