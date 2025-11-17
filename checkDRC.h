#include <bits/stdc++.h>
#include "Calculator.h"
using namespace std;

// ===================== 基础向量工具 =====================
struct Vec2 {
    double x, y;
    Vec2() : x(0), y(0) {}
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(double k) const { return {x*k, y*k}; }
};

struct Padinfo{
    double x;
    double y;
    double width;
    double height;
    double radius;
    vector<string> layers;
    string net;
    vector<vector<Vec2>> boundary; 

};
static constexpr double EPS = 1e-9;
static inline double dot(const Vec2&a,const Vec2&b){return a.x*b.x+a.y*b.y;}
static inline double cross(const Vec2&a,const Vec2&b){return a.x*b.y-a.y*b.x;}
static inline double norm2(const Vec2&a){return dot(a,a);}
static inline double norm(const Vec2&a){return sqrt(norm2(a));}
static inline double wrap(double a){while(a<0)a+=2*M_PI;while(a>=2*M_PI)a-=2*M_PI;return a;}

bool SegmentIntersectsArc(const Vec2& P1, const Vec2& P2,
                          const Vec2& A, const Vec2& B, const Vec2& C);

bool ArcIntersectsArc(const Vec2& A1,const Vec2& B1,const Vec2& C1,
                      const Vec2& A2,const Vec2& B2,const Vec2& C2);

bool CircleFrom3Points(const Vec2& A, const Vec2& B, const Vec2& C, Vec2& center, double& r);

bool SegmentIntersectsSegment(const Vec2& p1, const Vec2& p2,
                              const Vec2& q1, const Vec2& q2);

vector<vector<Vec2>> FindSegmentboundarys(double x1, double y1, double x2, double y2, double width);
vector<vector<Vec2>> FindRectBoundarys(double x, double y, double width, double height, double rotation);
vector<vector<Vec2>> FindCircleBoundarys(double x, double y, float radius);
vector<vector<Vec2>> FindOvalBoundarys(double x, double y, double width, double height, double rotation);
vector<vector<Vec2>> FindRoundrectBoundarys(double x, double y, double width, double height, float rratio, double rotation);
std::vector<double> FindPadAbsoluteCoor(double fp_x, double fp_y, std::string padname, std::vector<FootprintPadAbsolute> absCoor);