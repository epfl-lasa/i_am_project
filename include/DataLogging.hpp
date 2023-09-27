//|
//|    Copyright (C) 2021-2023 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors: Harshit Khurana (maintainer)
//|
//|    email:   harshit.khurana@epfl.ch
//|
//|    website: lasa.epfl.ch
//|
//|    This file is part of iam_dual_arm_control.
//|    This work was supported by the European Community's Horizon 2020 Research and Innovation
//|    programme (call: H2020-ICT-09-2019-2020, RIA), grant agreement 871899 Impact-Aware Manipulation.
//|
//|    iam_dual_arm_control is free software: you can redistribute it and/or modify  it under the terms
//|    of the GNU General Public License as published by  the Free Software Foundation,
//|    either version 3 of the License, or  (at your option) any later version.
//|
//|    iam_dual_arm_control is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|


#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <ros/package.h>

using namespace std;

class DataLogging {
public:
    // Data logging
    std::ofstream outRecordExp;

    bool init(std::string path2Datafolder) {
        auto now = std::chrono::system_clock::now();
        auto inTimeT = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&inTimeT), "%Y-%m-%d-%X");
        std::string dataID = ss.str();

        outRecordExp.open(path2Datafolder + "/exp_" + dataID + ".csv");

        if (!outRecordExp.is_open()) {
            std::cerr << "[outRecordExp]: Cannot open output data files, the Data directory might be missing" << std::endl;
            return false;
        }

        return true;
    }

    bool reset(std::string path2Datafolder) {

        this->closeFiles();

        auto now = std::chrono::system_clock::now();
        auto inTimeT = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&inTimeT), "%Y-%m-%d-%X");
        std::string dataID = ss.str();

        outRecordExp.open(path2Datafolder + "/exp_" + dataID + ".csv");

        if (!outRecordExp.is_open()) {
            std::cerr << "[outRecordExp]: Cannot open output data files, the Data directory might be missing" << std::endl;
            return false;
        }
    }

    bool closeFiles() {
        outRecordExp.close();
        return true;
    }

};
