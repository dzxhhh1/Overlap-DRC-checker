#include "Calculator.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <regex>
#include <cmath>

// 字符串转 double
double parseDouble(const std::string& s) {
    try {
        return std::stod(s);
    }
    catch (...) {
        return 0.0;
    }
}

// 角度转弧度
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// 旋转坐标 (顺时针为负角度)
std::pair<double, double> rotate(double x, double y, double angleDeg) {
    double rad = deg2rad(angleDeg);
    double xr = x * std::cos(rad) + y * std::sin(rad);
    double yr = -x * std::sin(rad) + y * std::cos(rad);
    return { xr, yr };
}

std::vector<FootprintPadAbsolute> calculateFootprintCoordinates(const std::string& inputFile) {
    std::vector<FootprintPadAbsolute> result;

    std::ifstream fin(inputFile);
    if (!fin.is_open()) {
        std::cerr << "无法打开输入文件\n";
        return result;
    }

    std::string line;
    Footprint currentFootprint;
    bool inFootprint = false;
    int bracketLevel = 0;

    while (std::getline(fin, line)) {
        line = std::regex_replace(line, std::regex("^\\s+|\\s+$"), "");

        // footprint 开始
        if (line.find("(footprint") != std::string::npos) {
            inFootprint = true;
            bracketLevel = 1;
            currentFootprint = Footprint();

            // 提取 footprint 名称
            size_t quote1 = line.find("\"");
            size_t quote2 = line.find("\"", quote1 + 1);
            if (quote1 != std::string::npos && quote2 != std::string::npos) {
                currentFootprint.name = line.substr(quote1 + 1, quote2 - quote1 - 1);
            }
            continue;
        }

        if (inFootprint) {
            // 更新括号层级
            for (char c : line) {
                if (c == '(') bracketLevel++;
                if (c == ')') bracketLevel--;
            }

            // 解析 footprint 原点 (只解析一次)
            if (currentFootprint.originX == 0 && line.find("(at") != std::string::npos) {
                std::regex r_at("\\(at\\s*([-+]?[0-9]*\\.?[0-9]+)\\s+([-+]?[0-9]*\\.?[0-9]+)(?:\\s+([-+]?[0-9]*\\.?[0-9]+))?");
                std::smatch m;
                if (std::regex_search(line, m, r_at)) {
                    currentFootprint.originX = parseDouble(m[1].str());
                    currentFootprint.originY = parseDouble(m[2].str());
                    currentFootprint.angle = m.size() >= 4 && m[3].matched ? parseDouble(m[3].str()) : 0;
                }
            }

            // 解析 pad
            if (line.find("(pad") != std::string::npos) {
                std::string padName;
                size_t quote1 = line.find("\"");
                size_t quote2 = line.find("\"", quote1 + 1);
                if (quote1 != std::string::npos && quote2 != std::string::npos) {
                    padName = line.substr(quote1 + 1, quote2 - quote1 - 1);
                }

                double x_rel = 0, y_rel = 0;
                std::regex r_pad("\\(at\\s*([-+]?[0-9]*\\.?[0-9]+)\\s+([-+]?[0-9]*\\.?[0-9]+)");
                std::smatch m_pad;
                if (std::regex_search(line, m_pad, r_pad)) {
                    x_rel = parseDouble(m_pad[1].str());
                    y_rel = parseDouble(m_pad[2].str());

                    // pad 坐标旋转 + 原点偏移（计算过程不变）
                    std::pair<double, double> rotated = rotate(x_rel, y_rel, currentFootprint.angle);
                    double xr = rotated.first + currentFootprint.originX;
                    double yr = rotated.second + currentFootprint.originY;

                    // 创建FootprintPadAbsolute对象并存储到result中
                    FootprintPadAbsolute fpa;
                    fpa.footprintName = currentFootprint.name;
                    fpa.padName = padName;
                    fpa.footprintOriginX = currentFootprint.originX;
                    fpa.footprintOriginY = currentFootprint.originY;
                    fpa.footprintAngle = currentFootprint.angle;
                    fpa.padAbsoluteX = xr;
                    fpa.padAbsoluteY = yr;

                    result.push_back(fpa);
                } else {
                    std::cerr << "无法解析 pad 坐标: " << line << std::endl;
                }
            }

            // footprint 结束
            if (bracketLevel == 0) {
                // 调试输出
                std::cout << "Footprint: " << currentFootprint.name
                    << " Origin: (" << currentFootprint.originX << ", " << currentFootprint.originY
                    << ") Angle: " << currentFootprint.angle << std::endl;

                inFootprint = false;
            }
        }
    }

    fin.close();
    return result;
}
