/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/scenarios/dead_end/turning_around/stage_approaching_turning_point.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace turning_around {
using apollo::common::PointENU;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::JunctionInfoConstPtr;

Stage::StageStatus StageApproachingTurningPoint::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  AERROR << "stage: StageApproachingTurningPoint";
  // CHECK_NOTNULL(frame);
  // GetContext()->dead_end_id.clear();
  // // valid check
  // PointENU test_point;
  // test_point.set_x(7576824.85);
  // test_point.set_y(8337819.44);
  // const hdmap::HDMap* sim_map_ptr = HDMapUtil::SimMapPtr();
  // std::vector<JunctionInfoConstPtr> junctions;
  // JunctionInfoConstPtr junction;
  // if (sim_map_ptr->GetJunctions(test_point, 1.0, &junctions) != 0) {
  //   AERROR << "Fail to get junctions from sim_map.";
  //   return StageStatus::ERROR;
  // }
  // if (junctions.size() <= 0 || junction->junction().type() != 5) {
  //   AERROR << "No dead end message from map";
  //   return StageStatus::ERROR;
  // }
  // // execute task
  // bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  // if (!plan_ok) {
  //   AERROR << "StopSignUnprotectedStagePreStop planning error";
  //   return StageStatus::ERROR;
  // }
  // // stage change
  // if (CheckADCStop(*frame)) {
  //   AERROR << "transfer to around turning";
  //   next_stage_ = ScenarioConfig::TURNING_AROUND_TURNING;
  //   return Stage::FINISHED;
  // }

  return Stage::RUNNING;
}

bool StageApproachingTurningPoint::CheckADCStop(const Frame& frame) {
  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (adc_speed > max_adc_stop_speed) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double stop_fence_start_s =
      frame.open_space_info().open_space_pre_stop_fence_s();
  const double distance_stop_line_to_adc_front_edge =
      stop_fence_start_s - adc_front_edge_s;

  if (distance_stop_line_to_adc_front_edge >
      scenario_config_.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }
  return true;
}

}  // namespace turning_around
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
