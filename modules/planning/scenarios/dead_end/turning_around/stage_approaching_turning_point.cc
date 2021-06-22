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
using apollo::routing::RoutingRequest;
using apollo::common::math::Vec2d;

Stage::StageStatus StageApproachingTurningPoint::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  AERROR << "stage: StageApproachingTurningPoint";
  CHECK_NOTNULL(frame);
  GetContext()->dead_end_id.clear();
  // valid check
  RoutingRequest routing_request_in;
  routing_request_in = frame->local_view().routing->routing_request();
  size_t waypoint_num = routing_request_in.waypoint().size();
  PointENU dead_end_point = routing_request_in
                            .waypoint().at(waypoint_num - 1).pose();
  const hdmap::HDMap* base_map_ptr = HDMapUtil::BaseMapPtr();
  std::vector<JunctionInfoConstPtr> junctions;
  JunctionInfoConstPtr junction;
  if (base_map_ptr->GetJunctions(dead_end_point, 1.0, &junctions) != 0) {
    AERROR << "Fail to get junctions from sim_map.";
    return StageStatus::ERROR;
  }
  if (junctions.size() <= 0) {
    AERROR << "No junctions from map";
    return StageStatus::ERROR;
  }
  if (!SelectTargetDeadEndJunction(&junctions, dead_end_point, &junction)) {
    AERROR << "Target Dead End not found";
    return StageStatus::ERROR;
  }
  // execute task
  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStagePreStop planning error";
    return StageStatus::ERROR;
  }
  // stage change
  if (CheckADCStop(*frame)) {
    AERROR << "transfer to around turning";
    next_stage_ = ScenarioConfig::TURNING_AROUND_TURNING;
    return Stage::FINISHED;
  }

  return Stage::RUNNING;
}

bool StageApproachingTurningPoint::SelectTargetDeadEndJunction(
    std::vector<JunctionInfoConstPtr>* junctions,
    const apollo::common::PointENU& dead_end_point,
    JunctionInfoConstPtr* target_junction) {
  // warning: the car only be the one junction
  size_t junction_num = junctions->size();
  if (junction_num <= 0) {
    AERROR << "No junctions frim map";
    return false;
  }
  for(size_t i = 0; i < junction_num; ++i) {
    if (junctions->at(i)->junction().type() == 5) {
      Vec2d start_point =
        junctions->at(i)->polygon().points().at(0);
      double max_x = start_point.x();
      double max_y = start_point.y();
      double min_x = start_point.x();
      double min_y = start_point.y();
      // in the junction
      size_t point_num = junctions->at(i)->polygon().points().size();
      for(size_t j = 1; j < point_num; ++j) {
        if (max_x < junctions->at(i)->polygon().points().at(j).x()) {
          max_x = junctions->at(i)->polygon().points().at(j).x();
        }
        if (max_y < junctions->at(i)->polygon().points().at(j).y()) {
          max_y = junctions->at(i)->polygon().points().at(j).y();
        }
        if (min_x > junctions->at(i)->polygon().points().at(j).x()) {
          min_x = junctions->at(i)->polygon().points().at(j).x();
        }
        if (min_y > junctions->at(i)->polygon().points().at(j).y()) {
          min_y = junctions->at(i)->polygon().points().at(j).y();
        } 
      }
      AERROR << "the max x is: " << max_x;
      AERROR << "the max y is: " << max_y;
      AERROR << "the min x is: " << min_x;
      AERROR << "the min y is: " << min_y;
      // judge dead end point in the select junction
      if (dead_end_point.x() >= min_x && dead_end_point.x() <= max_x &&
          dead_end_point.y() >= min_y && dead_end_point.y() <= max_y) {
        *target_junction = junctions->at(i);
        return true;
      } else {
        return false;
      }
    } else {
      AERROR << "No dead end junction";
      return false;
    }
  }
  return true;
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
