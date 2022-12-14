/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PLANNING_PLANNER_PLANNER_H_
#define MODULES_PLANNING_PLANNER_PLANNER_H_

#include <vector>

#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/base_types.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class Planner
 * @brief Planner is a base class for specific planners.
 *        It contains a pure virtual function Plan which must be implemented in
 * derived class.---Planner是特定计划人员的基类，它包含纯虚拟函数计划，必须在派生类中实现。
 */
class Planner {
 public:
  /**
   * @brief Constructor
   */
  Planner() = default;

  /**
   * @brief Destructor
   */
  virtual ~Planner() = default;

  /**
   * @brief Compute a trajectory for execution.---计算执行轨迹。
   * @param start_point The trajectory point where planning starts
   * @param discretized_trajectory The computed trajectory
   * @return true if planning succeeds; false otherwise.
   */
  virtual bool Plan(const TrajectoryPoint& start_point,
                    std::vector<TrajectoryPoint>* discretized_trajectory) = 0;
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNER_PLANNER_H_ */
