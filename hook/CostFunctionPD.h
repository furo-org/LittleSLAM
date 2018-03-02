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
 * @file CostFunctionPD.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef _COST_FUNCTION_PD_H_
#define _COST_FUNCTION_PD_H_

#include "CostFunction.h"

class CostFunctionPD : public CostFunction
{
public:
  CostFunctionPD() {
  }

  ~CostFunctionPD() {
  }

  virtual double calValue(double tx, double ty, double th);
};

#endif
