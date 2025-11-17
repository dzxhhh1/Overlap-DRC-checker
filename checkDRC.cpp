#include "checkDRC.h"

// ==========================================================
// 1️⃣ 线段–线段相交
// ==========================================================
bool SegmentIntersectsSegment(const Vec2& p1, const Vec2& p2,
                              const Vec2& q1, const Vec2& q2)
{
    Vec2 r = p2 - p1, s = q2 - q1;
    double rxs = cross(r, s);
    double q_p_r = cross(q1 - p1, r);

    if (fabs(rxs) < EPS && fabs(q_p_r) < EPS) {
        // 共线情形 -> 判断投影是否重叠
        auto overlap = [](double a,double b,double c,double d){
            if(a>b) swap(a,b); if(c>d) swap(c,d);
            return max(a,c) <= min(b,d) + EPS;
        };
        return overlap(p1.x,p2.x,q1.x,q2.x) && overlap(p1.y,p2.y,q1.y,q2.y);
    }
    if (fabs(rxs) < EPS) return false; // 平行不共线

    double t = cross(q1 - p1, s) / rxs;
    double u = cross(q1 - p1, r) / rxs;
    return (t >= -EPS && t <= 1+EPS && u >= -EPS && u <= 1+EPS);
}

// ==========================================================
// 辅助函数：由三点求圆心与半径
// ==========================================================
bool CircleFrom3Points(const Vec2& A, const Vec2& B, const Vec2& C, Vec2& center, double& r) {
    double a1 = 2*(B.x - A.x), b1 = 2*(B.y - A.y);
    double c1 = B.x*B.x + B.y*B.y - A.x*A.x - A.y*A.y;
    double a2 = 2*(C.x - A.x), b2 = 2*(C.y - A.y);
    double c2 = C.x*C.x + C.y*C.y - A.x*A.x - A.y*A.y;
    double D = a1*b2 - a2*b1;
    if (fabs(D) < EPS) return false; // 三点共线
    center.x = (c1*b2 - c2*b1) / D;
    center.y = (a1*c2 - a2*c1) / D;
    r = norm(A - center);
    return true;
}

// ==========================================================
// 2️⃣ 线段–圆弧相交（圆弧由三点定义）
// ==========================================================
bool SegmentIntersectsArc(const Vec2& P1, const Vec2& P2,
                          const Vec2& A, const Vec2& B, const Vec2& C)
{
    Vec2 center; double r;
    if (!CircleFrom3Points(A,B,C,center,r)) return false;

    // 判断弧方向（逆时针 or 顺时针）
    double dir = cross(B - A, C - A);
    bool ccw = (dir > 0);

    // 线段与圆相交（解二次方程）
    Vec2 d = P2 - P1, f = P1 - center;
    double A_ = dot(d,d);
    double B_ = 2*dot(f,d);
    double C_ = dot(f,f) - r*r;
    double disc = B_*B_ - 4*A_*C_;
    if (disc < -EPS) return false;

    auto test_t = [&](double t){
        if(t<-EPS || t>1+EPS) return false;
        Vec2 p = P1 + d*t;
        double aA = atan2(A.y - center.y, A.x - center.x);
        double aC = atan2(C.y - center.y, C.x - center.x);
        double aP = atan2(p.y - center.y, p.x - center.x);
        aA=wrap(aA); aC=wrap(aC); aP=wrap(aP);
        bool onArc=false;
        if(ccw){
            double span = fmod(aC - aA + 2*M_PI, 2*M_PI);
            double rel = fmod(aP - aA + 2*M_PI, 2*M_PI);
            onArc = rel <= span + 1e-9;
        }else{
            double span = fmod(aA - aC + 2*M_PI, 2*M_PI);
            double rel = fmod(aA - aP + 2*M_PI, 2*M_PI);
            onArc = rel <= span + 1e-9;
        }
        return onArc;
    };

    if(fabs(disc)<=EPS){
        double t = -B_/(2*A_);
        return test_t(t);
    }else{
        double s = sqrt(max(0.0,disc));
        double t1 = (-B_-s)/(2*A_);
        double t2 = (-B_+s)/(2*A_);
        return test_t(t1) || test_t(t2);
    }
}

// ==========================================================
// 3️⃣ 圆弧–圆弧相交（两弧各由三点定义）
// ==========================================================
bool ArcIntersectsArc(const Vec2& A1,const Vec2& B1,const Vec2& C1,
                      const Vec2& A2,const Vec2& B2,const Vec2& C2)
{
    Vec2 c1,c2; double r1,r2;
    if(!CircleFrom3Points(A1,B1,C1,c1,r1)) return false;
    if(!CircleFrom3Points(A2,B2,C2,c2,r2)) return false;
    double d = norm(c2 - c1);
    if (d > r1 + r2 + EPS) return false;
    if (d < fabs(r1 - r2) - EPS) return false;

    // 圆与圆交点（最多两点）
    double a = (r1*r1 - r2*r2 + d*d)/(2*d);
    double h2 = r1*r1 - a*a;
    if(h2 < -EPS) return false;
    if(h2 < 0) h2 = 0;
    double h = sqrt(h2);
    Vec2 u = (c2 - c1) * (1.0/d);
    Vec2 p0 = c1 + u*a;
    Vec2 perp(-u.y,u.x);

    vector<Vec2> cand;
    if(h<=EPS) cand.push_back(p0);
    else {
        cand.push_back(p0 + perp*h);
        cand.push_back(p0 - perp*h);
    }

    double dir1 = cross(B1 - A1, C1 - A1);
    bool ccw1 = (dir1 > 0);
    double dir2 = cross(B2 - A2, C2 - A2);
    bool ccw2 = (dir2 > 0);

    auto onArc = [&](const Vec2& p,const Vec2& A,const Vec2& C,const Vec2& cen,bool ccw){
        double aA = atan2(A.y - cen.y, A.x - cen.x);
        double aC = atan2(C.y - cen.y, C.x - cen.x);
        double aP = atan2(p.y - cen.y, p.x - cen.x);
        aA=wrap(aA); aC=wrap(aC); aP=wrap(aP);
        if(ccw){
            double span = fmod(aC - aA + 2*M_PI, 2*M_PI);
            double rel = fmod(aP - aA + 2*M_PI, 2*M_PI);
            return rel <= span + 1e-9;
        }else{
            double span = fmod(aA - aC + 2*M_PI, 2*M_PI);
            double rel = fmod(aA - aP + 2*M_PI, 2*M_PI);
            return rel <= span + 1e-9;
        }
    };

    for(const auto& p:cand){
        if(onArc(p,A1,C1,c1,ccw1) && onArc(p,A2,C2,c2,ccw2))
            return true;
    }
    return false;
}

// ==========================================================
// ✅ 示例
// ==========================================================

vector<vector<Vec2>> FindSegmentboundarys(double x1, double y1, double x2, double y2, double width){
    vector<vector<Vec2>> boundarys;
    if(x1 == x2){
        //垂直线段
        Vec2 left_down(x1 - width/2, min(y1,y2));
        Vec2 right_down(x1 + width/2, min(y1,y2));
        Vec2 arc_down(x1, min(y1,y2) - width/2);
        Vec2 left_up(x2 - width/2, max(y1,y2));
        Vec2 right_up(x2 + width/2, max(y1,y2));
        Vec2 arc_up(x2, max(y1,y2) + width/2);
        boundarys.push_back({left_down, left_up});
        boundarys.push_back({right_down, right_up});
        boundarys.push_back({left_down, arc_down, right_down});
        boundarys.push_back({left_up, arc_up, right_up});
        
    } else if(y1 == y2){
        //水平线段
        Vec2 left_down(min(x1,x2), y1 - width/2);
        Vec2 right_down(max(x1,x2), y1 - width/2);
        Vec2 arc_left(min(x1,x2) - width/2, y1);
        Vec2 left_up(min(x1,x2), y1 + width/2);
        Vec2 right_up(max(x1,x2), y1 + width/2);
        Vec2 arc_right(max(x1,x2) + width/2, y1);
        boundarys.push_back({left_down, right_down});
        boundarys.push_back({left_up, right_up});
        boundarys.push_back({left_down, arc_left, left_up});
        boundarys.push_back({right_down, arc_right, right_up});
    
    } else {
        // 任意斜率
        Vec2 p1(x1, y1), p2(x2, y2);
        double dx = x2 - x1, dy = y2 - y1;
        double L = std::hypot(dx, dy);
        if (L == 0) return boundarys; // 退化：长度为0，直接返回空

        double half = width * 0.5;

        // 单位切向量 t 与单位法向量 n（左法向）
        Vec2 t(dx / L, dy / L);
        Vec2 n(-t.y, t.x);

        // 四个边界点（两端各两个）
        Vec2 sL = p1 + n * half;   // start 端，"左"边
        Vec2 sR = p1 - n * half;   // start 端，"右"边
        Vec2 eL = p2 + n * half;   // end   端，"左"边
        Vec2 eR = p2 - n * half;   // end   端，"右"边

        // 两个半圆端帽的“中点”（三点式：端边点A, mid, 端边点C）
        // start 端向 -t 方向外扩
        Vec2 midStart = p1 - t * half;
        // end 端向 +t 方向外扩
        Vec2 midEnd   = p2 + t * half;

        // 两条平行边
        boundarys.push_back({ sL, eL });
        boundarys.push_back({ sR, eR });

        // 两个端帽（以三点描述圆弧：起点、弧上中点、终点）
        boundarys.push_back({ sL, midStart, sR }); // 起点端半圆
        boundarys.push_back({ eL, midEnd,   eR }); // 终点端半圆
    }

        /*} else if ((y2 - y1) / (x2 - x1) > 0){
        Vec2 left_down(min(x1,x2)+sqrt(2)/2*width/2, min(y1,y2)-sqrt(2)/2*width/2);
        Vec2 right_down(max(x1,x2)+sqrt(2)/2*width/2, max(y1,y2)-sqrt(2)/2*width/2);
        Vec2 left_up(min(x1,x2)-sqrt(2)/2*width/2, min(y1,y2)+sqrt(2)/2*width/2);
        Vec2 right_up(max(x1,x2)-sqrt(2)/2*width/2, max(y1,y2)+sqrt(2)/2*width/2);
        Vec2 arc_down(min(x1,x2)-sqrt(2)/2*width/2, min(y1,y2)-sqrt(2)/2*width/2);
        Vec2 arc_up(max(x1,x2)+sqrt(2)/2*width/2, max(y1,y2)+sqrt(2)/2*width/2);
        boundarys.push_back({left_down, right_down});
        boundarys.push_back({left_up, right_up});
        boundarys.push_back({left_down, arc_down, left_up});
        boundarys.push_back({right_down, arc_up, right_up});
    } else {
        Vec2 left_down(min(x1,x2)-sqrt(2)/2*width/2, max(y1,y2)-sqrt(2)/2*width/2);
        Vec2 right_down(max(x1,x2)-sqrt(2)/2*width/2, min(y1,y2)-sqrt(2)/2*width/2);
        Vec2 left_up(min(x1,x2)+sqrt(2)/2*width/2, max(y1,y2)+sqrt(2)/2*width/2);
        Vec2 right_up(max(x1,x2)+sqrt(2)/2*width/2, min(y1,y2)+sqrt(2)/2*width/2);
        Vec2 arc_down(max(x1,x2)+sqrt(2)/2*width/2, min(y1,y2)-sqrt(2)/2*width/2);
        Vec2 arc_up(min(x1,x2)-sqrt(2)/2*width/2, max(y1,y2)+sqrt(2)/2*width/2);
        boundarys.push_back({left_down, right_down});
        boundarys.push_back({left_up, right_up});
        boundarys.push_back({left_down, arc_up, left_up});
        boundarys.push_back({right_down, arc_down, right_up});
    }*/

    

    return boundarys;
    
}

vector<vector<Vec2>> FindRectBoundarys(double x, double y, double width, double height, double rotation){
    vector<vector<Vec2>> boundarys;
    if(rotation == 90 || rotation == 270 || rotation == -90){
        return FindRectBoundarys(x, y, height, width, 0);
    }

    Vec2 left_down(x - width/2, y - height/2);
    Vec2 right_down(x + width/2, y - height/2);
    Vec2 left_up(x - width/2, y + height/2);
    Vec2 right_up(x + width/2, y + height/2);

    boundarys.push_back({left_down, right_down});
    boundarys.push_back({left_up, right_up});
    boundarys.push_back({left_down, left_up});
    boundarys.push_back({right_down, right_up});
        

    return boundarys;
}

vector<vector<Vec2>> FindCircleBoundarys(double x, double y, float radius){
    vector<vector<Vec2>> boundarys;

    Vec2 top(x, y + radius);
    Vec2 bottom(x, y - radius);
    Vec2 left(x - radius, y);
    Vec2 right(x + radius, y);

    boundarys.push_back({left, top, right});
    boundarys.push_back({left, bottom, right});

    return boundarys;
}

vector<vector<Vec2>> FindOvalBoundarys(double x, double y, double width, double height, double rotation){
    vector<vector<Vec2>> boundarys;
    if (height > width) {
        if(rotation==90 || rotation == -90 || rotation == 270){
            return FindOvalBoundarys(x, y, height, width, 0);
        } else {
            Vec2 left_down(x-width/2,y-height/2+width/2);
            Vec2 right_down(x+width/2,y-height/2+width/2);
            Vec2 left_up(x-width/2,y+height/2-width/2);
            Vec2 right_up(x+width/2,y+height/2-width/2);
            Vec2 arc_top(x,y+height/2);
            Vec2 arc_bottom(x,y-height/2);
            boundarys.push_back({left_down, left_up});
            boundarys.push_back({right_down, right_up});
            boundarys.push_back({left_down, arc_bottom, right_down});
            boundarys.push_back({left_up, arc_top, right_up});
        }

    } else { 
        if (rotation==90 || rotation == -90 || rotation == 270){
            return FindOvalBoundarys(x, y, height, width, 0);
        } else {
            Vec2 left_down(x-width/2+height/2,y-height/2);
            Vec2 right_down(x+width/2-height/2,y-height/2);
            Vec2 left_up(x-width/2+height/2,y+height/2);
            Vec2 right_up(x+width/2-height/2,y+height/2);
            Vec2 arc_left(x-width/2,y);
            Vec2 arc_right(x+width/2,y);
            boundarys.push_back({left_down, right_down});
            boundarys.push_back({left_up, right_up});
            boundarys.push_back({left_down, arc_left, left_up});
            boundarys.push_back({right_down, arc_right, right_up});
        }
    }

    return boundarys;
}

vector<vector<Vec2>> FindRoundrectBoundarys(double x, double y, double width, double height, float rratio, double rotation){
    vector<vector<Vec2>> boundarys;
    float radius = std::min(width, height) * rratio;
    if(rotation == 90 || rotation == 270 || rotation == -90){
        return FindRoundrectBoundarys(x, y, height, width, rratio, 0);
    }

    Vec2 left_down(x - width/2, y - height/2 + radius);
    Vec2 down_left(x - width/2 + radius, y - height/2);
    Vec2 right_down(x + width/2, y - height/2 +radius);
    Vec2 down_right(x + width/2 - radius, y - height/2);
    Vec2 left_up(x - width/2, y + height/2 - radius);
    Vec2 up_left(x - width/2 + radius, y + height/2);
    Vec2 right_up(x + width/2, y + height/2 - radius);
    Vec2 up_right(x + width/2 - radius, y + height/2);
    Vec2 arc_left_down(x - width/2 + radius - radius*sqrt(2)/2, y - height/2 + radius - radius*sqrt(2)/2);
    Vec2 arc_left_up(x - width/2 + radius - radius*sqrt(2)/2, y + height/2 - radius + radius*sqrt(2)/2);
    Vec2 arc_right_down(x + width/2 - radius + radius*sqrt(2)/2, y - height/2 + radius - radius*sqrt(2)/2);
    Vec2 arc_right_up(x + width/2 - radius + radius*sqrt(2)/2, y + height/2 - radius + radius*sqrt(2)/2);

    boundarys.push_back({left_down, left_up});
    boundarys.push_back({right_down, right_up});
    boundarys.push_back({down_left, down_right});
    boundarys.push_back({up_left, up_right});
    boundarys.push_back({left_down, arc_left_down, down_left});
    boundarys.push_back({right_down, arc_right_down, down_right});
    boundarys.push_back({left_up, arc_left_up, up_left});
    boundarys.push_back({right_up, arc_right_up, up_right});

    return boundarys;
}

std::vector<double> FindPadAbsoluteCoor(double fp_x, double fp_y, std::string padname, std::vector<FootprintPadAbsolute> absCoor){
    std::vector<double> coor;
    for(auto abscoor: absCoor){
        //std::cout << abscoor.footprintOriginX << " " << fp_x << " " << abscoor.footprintOriginY << " " << fp_y << " " << abscoor.padName << " " << padname << std::endl;
        if (abscoor.footprintOriginX == fp_x && abscoor.footprintOriginY ==fp_y && abscoor.padName == padname){
            //std::cout << abscoor.footprintOriginX << " " << fp_x << " " << abscoor.footprintOriginY << " " << fp_y << " " << abscoor.padName << " " << padname << std::endl;
            coor.push_back(abscoor.padAbsoluteX);
            coor.push_back(abscoor.padAbsoluteY);
            break;
        }
    }
    if (coor.empty()){
        std::cerr << "Error: Pad absolute coordinates not found for footprint origin (" << fp_x << ", " << fp_y << ") and pad name '" << padname << "'" << std::endl;
    }
    return coor;
}