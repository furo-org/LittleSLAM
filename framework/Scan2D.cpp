/****************************************************************************
 * LittleSLAM: 2D-Laser SLAM for educational use
 * Copyright (C) 2017-2018 Masahiro Tomono
 * Copyright (C) 2018 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file Scan2D.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "Scan2D.h"

using namespace std;

double Scan2D::MAX_SCAN_RANGE = 6;
double Scan2D::MIN_SCAN_RANGE = 0.1;

