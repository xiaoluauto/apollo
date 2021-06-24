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
#include "modules/planning/scenarios/dead_end/deadend_turnaround/deadend_turnaround_scenario.h"
#include "modules/planning/scenarios/dead_end/deadend_turnaround/stage_approaching_turning_point.h"
#include "modules/planning/scenarios/dead_end/deadend_turnaround/stage_turning.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace deadend_turnaround {

using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;
using apollo::common::PointENU;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::routing::RoutingRequest;

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
               const std::shared_ptr<DependencyInjector>& injector)>
    DeadEndTurnAroundScenario::s_stage_factory_;

void DeadEndTurnAroundScenario::Init() {
  AERROR << "enter the dead end scenario init";
  if (init_) {
    return;
  }
  AERROR << "befor scenario init";
  Scenario::Init();
  AERROR << "after scenario init";
  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
}

void DeadEndTurnAroundScenario::RegisterStages() {
  if (s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::DEADEND_TURNAROUND_APPROACHING_TURNING_POINT,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StageApproachingTurningPoint(config, injector);
      });
  s_stage_factory_.Register(
      ScenarioConfig::DEADEND_TURNAROUND_TURNING,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StageTurning(config, injector);
      });
}

std::unique_ptr<Stage> DeadEndTurnAroundScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config, injector);
  if (ptr) {
    ptr->SetContext(&context_);
  }
  return ptr;
}

bool DeadEndTurnAroundScenario::GetScenarioConfig() {
  if (!config_.has_deadend_turnaround_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.deadend_turnaround_config());
  return true;
}

bool DeadEndTurnAroundScenario::IsTransferable(const Frame& frame,
                                           const double dead_end_start_range) {
  AERROR << "enter the dead end scenario transfer";
  std::string target_dead_end_id;
  // vaild check
  // get the users,s point
  RoutingRequest routing_request_in;
  routing_request_in = frame.local_view().routing->routing_request();
  size_t waypoint_num = routing_request_in.waypoint().size();
  PointENU dead_end_point = routing_request_in
                            .waypoint().at(waypoint_num - 1).pose();
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<JunctionInfoConstPtr> junctions;
  JunctionInfoConstPtr junction;
  if (base_map_ptr->GetJunctions(dead_end_point, 1.0, &junctions) != 0) {
    AERROR << "Fail to get junctions from base_map.";
    return false;
  }
  AERROR << "the size of junctions is: " << junctions.size();
  if (junctions.size() <= 0) {
    AERROR << "No junction from map";
    return false;
  }
  if (!SelectTargetDeadEndJunction(&junctions, dead_end_point, &junction)) {
    AERROR << "Target Dead End not found";
    return false;
  }
  AERROR << "the type is: " << junction->junction().type();
  target_dead_end_id = junction->id().id();
  const auto& vehicle_state = frame.vehicle_state();
  const auto& nearby_path =
      frame.reference_line_info().front().reference_line().map_path();
  if (!CheckDistanceToDeadEnd(vehicle_state,
                              nearby_path,
                              dead_end_start_range,
                              &junction)) {
    AERROR << "Dead end found, but too far, distance larger than "
              "pre-defined distance "
           << target_dead_end_id;
    return false;
  }
  return true;
}

bool DeadEndTurnAroundScenario::SelectTargetDeadEndJunction(
    std::vector<JunctionInfoConstPtr>* junctions,
    const apollo::common::PointENU& dead_end_point,
    JunctionInfoConstPtr* target_junction) {
  // warning: the car only be the one junction
  size_t junction_num = junctions->size();
  if (junction_num <= 0) {
    AERROR << "No junctions frim map";
    return false;
  }
  for (size_t i = 0; i < junction_num; ++i) {
    if (junctions->at(i)->junction().type() == 5) {
      Vec2d start_point =
        junctions->at(i)->polygon().points().at(0);
      double max_x = start_point.x();
      double max_y = start_point.y();
      double min_x = start_point.x();
      double min_y = start_point.y();
      // in the junction
      size_t point_num = junctions->at(i)->polygon().points().size();
      for (size_t j = 1; j < point_num; ++j) {
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

bool DeadEndTurnAroundScenario::CheckDistanceToDeadEnd(
    const VehicleState& vehicle_state,
    const Path& nearby_path,
    const double dead_end_start_range,
    JunctionInfoConstPtr* junction) {
  // the polygon point is Clockwise
  auto points = (*junction)->polygon().points();
  Vec2d first_point = points.at(0);
  AERROR << "the first x is: " << first_point.x();
  AERROR << "the first y is: " << first_point.y();
  // the last point's s value may be unsuitable
  Vec2d last_point = points.at(3);
  AERROR << "the last x is: " << last_point.x();
  AERROR << "the last y is: " << last_point.y();
  AERROR << "the car x is: " << vehicle_state.x();
  AERROR << "the car y is: " << vehicle_state.y();
  double first_point_s = 0.0;
  double first_point_l = 0.0;
  double last_point_s = 0.0;
  double last_point_l = 0.0;
  nearby_path.GetNearestPoint(first_point, &first_point_s,
                              &first_point_l);
  nearby_path.GetNearestPoint(last_point, &last_point_s,
                              &last_point_l);
  double vehicle_point_s = 0.0;
  double vehicle_point_l = 0.0;
  Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
  nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
  AERROR << "first s is: " << first_point_s;
  AERROR << "last s is: " << last_point_s;
  AERROR << "car s is: " << vehicle_point_s;
  double center_s = (first_point_s + last_point_s) / 2.0;
  AERROR << "the delta is: " << center_s - vehicle_point_s;
  if (std::abs(center_s - vehicle_point_s) <
      dead_end_start_range) {
    return true;
  } else {
    return false;
  }
}

}  // namespace deadend_turnaround
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
