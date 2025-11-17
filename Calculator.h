#ifndef CALCULATOR_H
#define CALCULATOR_H

#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Footprint结构体（用于内部解析）
struct Pad {
    std::string name;
    double x;
    double y;
};

struct Footprint {
    std::string name;
    double originX;
    double originY;
    double angle;
    std::vector<Pad> pads;
    Footprint() : originX(0), originY(0), angle(0) {}
};

// 新的结构体，存储footprint和pad的绝对坐标信息（用于返回结果）
struct FootprintPadAbsolute {
    std::string footprintName;
    std::string padName;
    double footprintOriginX;
    double footprintOriginY;
    double footprintAngle;
    double padAbsoluteX;
    double padAbsoluteY;
};

std::vector<FootprintPadAbsolute> calculateFootprintCoordinates(const std::string& inputFile);

#endif // CALCULATOR_H
