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

#pragma once

#include <memory>
#include <string>

#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/map/proto/map_id.pb.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace turning_around {

struct TurningAroundContext {// keng
  ScenarioTurningAroundConfig scenario_config;
  std::string target_dead_end_id;
  bool pre_stop_rightaway_flag = false;
  hdmap::MapPathPoint pre_stop_rightaway_point;
};

class TurningAroundScenario : public Scenario {
 public:
  TurningAroundScenario(const ScenarioConfig& config,
                       const ScenarioContext* context,
                       const std::shared_ptr<DependencyInjector>& injector)
      : Scenario(config, context, injector) {}

  void Init() override;

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config,
      const std::shared_ptr<DependencyInjector>& injector) override;

  static bool IsTransferable(const Frame& frame,
                             const double turn_start_range);

  TurningAroundContext* GetContext() { return &context_; }

 private:
  static void RegisterStages();
  bool GetScenarioConfig();
  static bool SearchTargetDeadEndOnPath(
      const hdmap::Path& nearby_path, const std::string& dead_end_id,
      hdmap::PathOverlap* dead_end_overlap);
  static bool CheckDistanceToDeadEnd(
      const Frame& frame,
      const common::VehicleState& vehicle_state, const hdmap::Path& nearby_path,
      const double turn_start_range,
      const hdmap::PathOverlap& dead_end_overlap);

 private:
  bool init_ = false;
  static apollo::common::util::Factory<
      ScenarioConfig::StageType, Stage,
      Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
                 const std::shared_ptr<DependencyInjector>& injector)>
      s_stage_factory_;
  TurningAroundContext context_;
  const hdmap::HDMap* hdmap_ = nullptr;
};

}  // namespace turning_around
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
