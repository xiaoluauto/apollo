/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/turn_around/turning_around/turning_around_scenario.h"

#include "modules/planning/scenarios/turn_around/turning_around/stage_approaching_turning_point.h"
#include "modules/planning/scenarios/turn_around/turning_around/stage_turning.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace turning_around {

using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
               const std::shared_ptr<DependencyInjector>& injector)>
    TurningAroundScenario::s_stage_factory_;

void TurningAroundScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
}

void TurningAroundScenario::RegisterStages() {
  if (s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::TURNING_AROUND_APPROACHING_TURNING_POINT,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StageApproachingTurningPoint(config, injector);
      });
  s_stage_factory_.Register(
      ScenarioConfig::TURNING_AROUND_TURNING,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StageTurning(config, injector);
      });
}

std::unique_ptr<Stage> TurningAroundScenario::CreateStage(
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

bool TurningAroundScenario::GetScenarioConfig() {
  if (!config_.has_turning_around_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.turning_around_config());
  return true;
}

bool TurningAroundScenario::IsTransferable(const Frame& frame,
                                           const double turn_start_range) {
  // TODO(all) Implement available parking spot detection by preception results
  std::string dead_end_id;
  if (frame.local_view().routing->routing_request().has_dead_end() &&
      frame.local_view()
          .routing->routing_request()
          .dead_end()
          .has_dead_end_id()) {
    dead_end_id = frame.local_view()
                  .routing->routing_request()
                  .dead_end()
                  .dead_end_id();
  } else {
    ADEBUG << "No dead end id from routing";
    return false;
  }

  if (dead_end_id.empty()) {
    return false;
  }
  // get the nearby path of dead end
  const auto& nearby_path =
      frame.reference_line_info().front().reference_line().map_path();
  PathOverlap dead_end_overlap;
  const auto& vehicle_state = frame.vehicle_state();

  if (!SearchTargetDeadEndOnPath(nearby_path, dead_end_id,
                                 &dead_end_overlap)) {
    ADEBUG << "No such dead end found after searching all path forward "
              "possible"
           << dead_end_id;
    return false;
  }

  if (!CheckDistanceToDeadEnd(frame, vehicle_state, nearby_path,
                              turn_start_range, dead_end_overlap)) {
    ADEBUG << "target dead end found, but too far, distance larger than "
              "pre-defined distance"
           << dead_end_id;
    return false;
  }

  return true;
}

bool TurningAroundScenario::SearchTargetDeadEndOnPath(
    const Path& nearby_path, const std::string& dead_end_id,
    PathOverlap* dead_end_overlap) {
  const auto& dead_end_overlaps = nearby_path.dead_end_overlaps();
  for (const auto& deading_overlap : dead_end_overlaps) {
    if (deading_overlap.object_id == dead_end_id) {
      *dead_end_overlap = deading_overlap;
      return true;
    }
  }
  return false;
}

double TurningAroundScenario::ComputeMaxS(const Frame& frame,
                                          const Path& nearby_path) {
  const auto &routing_request =
      frame.local_view().routing->routing_request();
  auto guillotine_point =
      routing_request.dead_end().guillotine_point();
  size_t points_num = guillotine_point.point().size();
  double maximum_s = 0.0;
  double maximum_l = 0.0;
  for(size_t i = 0; i < points_num; ++i) {
    // get the s of point
    double max_s = 0.0;
    double max_l = 0.0;
    Vec2d dead_end_point(guillotine_point.point().at(i).x(),
                         guillotine_point.point().at(i).y());
    bool nearest_flag = nearby_path.GetNearestPoint(dead_end_point, &max_s, &max_l);
    if (nearest_flag) {
      if (max_s > maximum_s) {
        maximum_s = max_s;
      }
      if (max_l > maximum_l) {
        maximum_l = max_l;
      }
    }
  }
  return maximum_s;
}

bool TurningAroundScenario::CheckDistanceToDeadEnd(
    const Frame& frame,
    const VehicleState& vehicle_state, const Path& nearby_path,
    const double turn_start_range,
    const PathOverlap& dead_end_overlap) {
  // get the hdmap pointer
  // const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::Id id;
  id.set_id(dead_end_overlap.object_id);
  // get the dead_end pointer by the id, by the map colleague!!!!!! will be change
  // changing !!!!!!!!!!!!!
  // ParkingSpaceInfoConstPtr target_parking_spot_ptr =
  //     hdmap->GetParkingSpaceById(id);
  // get the dead end points
  double maximum_s = ComputeMaxS(frame, nearby_path);

  // judge logic
  double vehicle_point_s = 0.0;
  double vehicle_point_l = 0.0;
  Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
  nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
  if (std::abs(maximum_s - vehicle_point_s) <
      turn_start_range) {
    return true;
  } else {
    return false;
  }
}

}  // namespace turning_around
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
