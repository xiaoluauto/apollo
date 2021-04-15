/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/task_manager/turning_around_routing_manager.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/task_manager/common/task_manager_gflags.h"

namespace apollo {
namespace task_manager {

using apollo::common::PointENU;
using apollo::common::math::Vec2d;

TurningAroundRoutingManager::TurningAroundRoutingManager()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::TASK_MANAGER) {}

common::Status TurningAroundRoutingManager::Init(
    const TurningAroundRoutingTask& turning_around_routing_task) {
  return common::Status::OK();
}
}  // namespace task_manager
}  // namespace apollo
