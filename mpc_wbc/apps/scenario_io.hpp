/**
 * @file scenario_io.hpp
 * @brief S9 A/B 场景文件读写（两个 harness 共用，保证命令序列完全一致）。
 *
 * 文件格式（只含**运动段**；起立段由各 harness 自己拼，保证两条路径的"起点"都是站姿）：
 *   phase,duration,vx,vy,wz
 *   walk_fwd,3.0,0.15,0.0,0.0
 *   stop,1.0,0.0,0.0,0.0
 *
 * 列序与 `analyze_s9_run.py` 读取的相位表一致；`#` 开头与空行忽略。
 */
#pragma once

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace scenario_io {

struct Item {
    std::string name;
    double duration = 0.0;
    double vx = 0.0, vy = 0.0, wz = 0.0;
};

/// 读场景文件；失败时返回 false 并把原因写进 err。
inline bool Load(const std::string& path, std::vector<Item>* out, std::string* err) {
    std::ifstream in(path);
    if (!in) {
        *err = "cannot open scenario file: " + path;
        return false;
    }
    out->clear();
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::vector<std::string> tok;
        std::string cell;
        while (std::getline(ss, cell, ',')) tok.push_back(cell);
        if (tok.size() < 5) {
            *err = path + ":" + std::to_string(line_no) + " expect 5 columns (phase,duration,vx,vy,wz)";
            return false;
        }
        if (tok[0] == "phase") continue;  // 表头
        Item it;
        it.name = tok[0];
        char* end = nullptr;
        it.duration = std::strtod(tok[1].c_str(), &end);
        it.vx = std::strtod(tok[2].c_str(), &end);
        it.vy = std::strtod(tok[3].c_str(), &end);
        it.wz = std::strtod(tok[4].c_str(), &end);
        if (!(it.duration > 0.0)) {
            *err = path + ":" + std::to_string(line_no) + " duration must be > 0";
            return false;
        }
        out->push_back(it);
    }
    if (out->empty()) {
        *err = path + " has no motion phases";
        return false;
    }
    return true;
}

/// 场景名以 .csv 结尾即视为文件路径（否则用内置预设）。
inline bool LooksLikePath(const std::string& s) {
    return s.size() > 4 && s.compare(s.size() - 4, 4, ".csv") == 0;
}

}  // namespace scenario_io
