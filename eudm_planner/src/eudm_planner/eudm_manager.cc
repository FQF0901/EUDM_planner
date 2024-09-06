#include "eudm_planner/eudm_manager.h"

#include <glog/logging.h>

namespace planning {

void EudmManager::Init(const std::string& config_path,
                       const decimal_t work_rate) {
  google::InitGoogleLogging("eudm");
  bp_.Init(config_path);
  bp_.set_map_interface(&map_adapter_);
  work_rate_ = work_rate;
  if (bp_.cfg().function().active_lc_enable()) {
    LOG(ERROR) << "[HMI]HMI enabled with active lane change ON.";
  } else {
    LOG(ERROR) << "[HMI]HMI enabled with active lane change OFF.";
  }
}

decimal_t EudmManager::GetNearestFutureDecisionPoint(const decimal_t& stamp,
                                                     const decimal_t& delta) {
  // consider the nearest decision point (rounded by layer time)
  // w.r.t stamp + delta
  decimal_t past_decision_point =
      std::floor((stamp + delta) / bp_.cfg().sim().duration().layer()) *
      bp_.cfg().sim().duration().layer();
  return past_decision_point + bp_.cfg().sim().duration().layer();
}

ErrorType EudmManager::Prepare(
    const decimal_t stamp,
    const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
    const planning::eudm::Task& task) {
  map_adapter_.set_map(map_ptr);

  BtAction desired_action;
  if (!GetReplanDesiredAction(stamp, &desired_action)) {
    desired_action.lat = BtLatAction::kLaneKeeping;
    desired_action.lon = BtLonAction::kMaintain;
    decimal_t fdp_stamp = GetNearestFutureDecisionPoint(stamp, 0.0);
    desired_action.t = fdp_stamp - stamp;
  }

  if (map_adapter_.map()->GetEgoNearestLaneId(&ego_lane_id_) != kSuccess) {
    return kWrongStatus;
  }

  UpdateLaneChangeContextByTask(stamp, task);
  if (lc_context_.completed) {
    desired_action.lat = BtLatAction::kLaneKeeping;
  }

  {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager]Replan context <valid, stamp, seq>:<"
              << context_.is_valid << "," << std::fixed << std::setprecision(3)
              << context_.seq_start_time << ",";
    for (auto& a : context_.action_seq) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (auto& a : context_.action_seq) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << ">";
    LOG(WARNING) << line_info.str();
  }
  {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager]LC context <completed, twa, tt, dt, l_id, "
                 "lat, type>:<"
              << lc_context_.completed << ","
              << lc_context_.trigger_when_appropriate << "," << std::fixed
              << std::setprecision(3) << lc_context_.trigger_time << ","
              << lc_context_.desired_operation_time << ","
              << lc_context_.ego_lane_id << ","
              << common::SemanticsUtils::RetLatBehaviorName(lc_context_.lat)
              << "," << static_cast<int>(lc_context_.type) << ">";
    LOG(WARNING) << line_info.str();
  }

  bp_.UpdateDcpTree(desired_action);
  decimal_t ref_vel;
  EvaluateReferenceVelocity(task, &ref_vel);
  bp_.set_desired_velocity(ref_vel);
  bp_.set_lane_change_info(task.lc_info);

  LOG(WARNING) << "[Eudm][Manager]desired <lon,lat,t>:<"
               << DcpTree::RetLonActionName(desired_action.lon).c_str() << ","
               << DcpTree::RetLatActionName(desired_action.lat) << ","
               << desired_action.t << "> ego lane id:" << ego_lane_id_;

  return kSuccess;
}

ErrorType EudmManager::GenerateLaneChangeProposal(
    const decimal_t& stamp, const planning::eudm::Task& task) {
  if (!bp_.cfg().function().active_lc_enable()) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to disabled lc:"
                 << stamp;
    return kSuccess;
  }

  if (!task.is_under_ctrl) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to not under ctrl:"
                 << stamp;
    return kSuccess;
  }

  if (!lc_context_.completed) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to not completed lc:"
                 << stamp;
    return kSuccess;
  }

  // if stick not reset, will not try active lane change
  if (lc_context_.completed && task.user_perferred_behavior != 0) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to stick not rest:"
                 << stamp;
    return kSuccess;
  }

  if (stamp - last_lc_proposal_.trigger_time < 0.0) {
    last_lc_proposal_.valid = false;
    last_lc_proposal_.trigger_time = stamp;
    last_lc_proposal_.ego_lane_id = ego_lane_id_;
    last_lc_proposal_.lat = LateralBehavior::kLaneKeeping;
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to illegal stamp:"
                 << stamp;
    return kSuccess;
  }

  if (stamp - last_lc_proposal_.trigger_time <
      bp_.cfg().function().active_lc().cold_duration()) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to cold down:" << stamp
                 << " < " << last_lc_proposal_.trigger_time << " + "
                 << bp_.cfg().function().active_lc().cold_duration();
    return kSuccess;
  }

  if (last_snapshot_.plan_state.velocity <
          bp_.cfg().function().active_lc().activate_speed_lower_bound() ||
      last_snapshot_.plan_state.velocity >
          bp_.cfg().function().active_lc().activate_speed_upper_bound()) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to illegal spd:"
                 << stamp;
    return kSuccess;
  }

  if (bp_.cfg()
          .function()
          .active_lc()
          .enable_clear_accumulation_by_forbid_signal() &&
      not preliminary_active_requests_.empty()) {
    auto last_request = preliminary_active_requests_.back();
    if (last_request.lat == LateralBehavior::kLaneChangeLeft &&
        task.lc_info.forbid_lane_change_left) {
      LOG(WARNING) << std::fixed << std::setprecision(5)
                   << "[Eudm][ActiveLc]Clear request due to forbid signal:"
                   << stamp;
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    if (last_request.lat == LateralBehavior::kLaneChangeRight &&
        task.lc_info.forbid_lane_change_right) {
      LOG(WARNING) << std::fixed << std::setprecision(5)
                   << "[Eudm][ActiveLc]Clear request due to forbid signal:"
                   << stamp;
      preliminary_active_requests_.clear();
      return kSuccess;
    }
  }

  common::LateralBehavior lat_behavior;
  decimal_t operation_at_seconds;
  bool is_cancel_behavior;
  bp_.ClassifyActionSeq(
      last_snapshot_.action_script[last_snapshot_.original_winner_id],
      &operation_at_seconds, &lat_behavior, &is_cancel_behavior);
  if (lat_behavior == LateralBehavior::kLaneKeeping || is_cancel_behavior) {
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to not ideal behavior:"
                 << stamp;
    preliminary_active_requests_.clear();
    return kSuccess;
  }

  ActivateLaneChangeRequest this_request;
  this_request.trigger_time = stamp;
  this_request.desired_operation_time = stamp + operation_at_seconds;
  this_request.ego_lane_id = ego_lane_id_;
  this_request.lat = lat_behavior;
  if (preliminary_active_requests_.empty()) {
    preliminary_active_requests_.push_back(this_request);
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]trigger time:" << this_request.trigger_time
                 << " Init requesting "
                 << common::SemanticsUtils::RetLatBehaviorName(this_request.lat)
                 << " at " << this_request.desired_operation_time
                 << " with lane id " << this_request.ego_lane_id;
  } else {
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]trigger time:" << this_request.trigger_time
                 << " Consequent requesting "
                 << common::SemanticsUtils::RetLatBehaviorName(this_request.lat)
                 << " at " << this_request.desired_operation_time
                 << " with lane id " << this_request.ego_lane_id;
    auto last_request = preliminary_active_requests_.back();
    if (last_request.ego_lane_id != this_request.ego_lane_id) {
      LOG(WARNING)
          << "[Eudm][ActiveLc]Invalid this request due to lane id inconsitent.";
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    if (last_request.lat != this_request.lat) {
      LOG(WARNING) << "[Eudm][ActiveLc]Invalid this request due to behavior "
                      "inconsitent.";
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    if (fabs(last_request.desired_operation_time -
             this_request.desired_operation_time) >
        bp_.cfg().function().active_lc().consistent_operate_time_min_gap()) {
      LOG(WARNING)
          << "[Eudm][ActiveLc]Invalid this request due to time inconsitent.";
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    preliminary_active_requests_.push_back(this_request);
    LOG(WARNING) << "[Eudm][ActiveLc]valid this request. Queue size "
                 << preliminary_active_requests_.size() << " and operate at "
                 << operation_at_seconds;
  }

  if (preliminary_active_requests_.size() >=
      bp_.cfg().function().active_lc().consistent_min_num_frame()) {
    if (operation_at_seconds <
        bp_.cfg().function().active_lc().activate_max_duration_in_seconds() +
            kEPS) {
      last_lc_proposal_.valid = true;
      last_lc_proposal_.trigger_time = stamp;
      last_lc_proposal_.operation_at_seconds =
          operation_at_seconds > bp_.cfg()
                                     .function()
                                     .active_lc()
                                     .active_min_operation_in_seconds()
              ? operation_at_seconds
              : GetNearestFutureDecisionPoint(
                    stamp, bp_.cfg()
                               .function()
                               .active_lc()
                               .active_min_operation_in_seconds()) -
                    stamp;
      last_lc_proposal_.ego_lane_id = ego_lane_id_;
      last_lc_proposal_.lat = lat_behavior;
      preliminary_active_requests_.clear();
      LOG(WARNING) << std::fixed << std::setprecision(5)
                   << "[HMI]Gen proposal with trigger time "
                   << last_lc_proposal_.trigger_time << " lane id "
                   << last_lc_proposal_.ego_lane_id << " behavior "
                   << static_cast<int>(last_lc_proposal_.lat) << " operate at "
                   << last_lc_proposal_.operation_at_seconds;
    } else {
      preliminary_active_requests_.clear();
      // LOG(WARNING) << "[HMI]Abandan Queue due to change time not legal.";
    }
  }

  return kSuccess;
}

void EudmManager::UpdateLaneChangeContextByTask(
    const decimal_t stamp, const planning::eudm::Task& task) {
  if (!last_task_.is_under_ctrl && task.is_under_ctrl) {
    LOG(WARNING) << "[HMI]Autonomous mode activated!";
    lc_context_.completed = true;
    lc_context_.trigger_when_appropriate = false;
    last_lc_proposal_.trigger_time = stamp;
  }

  if (last_task_.is_under_ctrl && !task.is_under_ctrl) {
    LOG(WARNING) << "[HMI]Autonomous mode deactivated!";
    lc_context_.completed = true;
    lc_context_.trigger_when_appropriate = false;
    last_lc_proposal_.trigger_time = stamp;
  }

  if (task.user_perferred_behavior != last_task_.user_perferred_behavior) {
    LOG(WARNING) << "[HMI]stick state change from "
                 << last_task_.user_perferred_behavior << " to "
                 << task.user_perferred_behavior;
  }

  if ((task.lc_info.forbid_lane_change_left !=
       last_task_.lc_info.forbid_lane_change_left) ||
      (task.lc_info.forbid_lane_change_right !=
       last_task_.lc_info.forbid_lane_change_right)) {
    LOG(WARNING) << "[HMI]lane change forbid signal [left] "
                 << task.lc_info.forbid_lane_change_left << " [right] "
                 << task.lc_info.forbid_lane_change_right;
  }

  if (task.is_under_ctrl) {
    if (!lc_context_.completed) {
      if (ego_lane_id_ != lc_context_.ego_lane_id) {
        // in progress lane change and lane id change
        LOG(WARNING) << "[HMI]lane change completed due to different lane id "
                     << lc_context_.ego_lane_id << " to " << ego_lane_id_
                     << ". Cd alc.";
        lc_context_.completed = true;
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      } else {
        if (task.user_perferred_behavior != 1 &&
            last_task_.user_perferred_behavior == 1) {
          // receive a lane cancel trigger
          LOG(WARNING) << "[HMI]lane change cancel by stick "
                       << last_task_.user_perferred_behavior << " to "
                       << task.user_perferred_behavior << ". Cd alc.";
          lc_context_.completed = true;
          lc_context_.trigger_when_appropriate = false;
          last_lc_proposal_.trigger_time = stamp;
        } else if (task.user_perferred_behavior != -1 &&
                   last_task_.user_perferred_behavior == -1) {
          // receive a lane cancel trigger
          LOG(WARNING) << "[HMI]lane change cancel by stick "
                       << last_task_.user_perferred_behavior << " to "
                       << task.user_perferred_behavior << ". Cd alc.";
          lc_context_.completed = true;
          lc_context_.trigger_when_appropriate = false;
          last_lc_proposal_.trigger_time = stamp;
        } else if (lc_context_.type == LaneChangeTriggerType::kActive) {
          if (bp_.cfg()
                  .function()
                  .active_lc()
                  .enable_auto_cancel_by_outdate_time() &&
              stamp > lc_context_.desired_operation_time +
                          bp_.cfg()
                              .function()
                              .active_lc()
                              .auto_cancel_if_late_for_seconds()) {
            if (lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
              LOG(WARNING)
                  << "[HMI]ACTIVE [Left] auto cancel due to outdated for "
                  << stamp - lc_context_.desired_operation_time
                  << " s. Cd alc.";
            } else {
              LOG(WARNING)
                  << "[HMI]ACTIVE [Right] auto cancel due to outdated for "
                  << stamp - lc_context_.desired_operation_time
                  << " s. Cd alc.";
            }
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_cancel_by_forbid_signal() &&
                     task.lc_info.forbid_lane_change_left &&
                     lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
            LOG(WARNING) << "[HMI]ACTIVE [Left] canceled due to forbidden "
                            "signal. Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_cancel_by_forbid_signal() &&
                     task.lc_info.forbid_lane_change_right &&
                     lc_context_.lat == LateralBehavior::kLaneChangeRight) {
            LOG(WARNING)
                << "[HMI]ACTIVE [Right] canceled due to forbidden signal. "
                   "Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_canbel_by_stick_signal() &&
                     lc_context_.lat == LateralBehavior::kLaneChangeLeft &&
                     (task.user_perferred_behavior == 1 ||
                      task.user_perferred_behavior == 11)) {
            LOG(WARNING)
                << "[HMI]ACTIVE [left] canceled due to human opposite signal. "
                   "Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_canbel_by_stick_signal() &&
                     lc_context_.lat == LateralBehavior::kLaneChangeRight &&
                     (task.user_perferred_behavior == -1 ||
                      task.user_perferred_behavior == 12)) {
            LOG(WARNING) << "[HMI]ACTIVE canceled due to human active signal. "
                            "Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          }
        }
      }
    } else {
      // lane change completed state: welcome new activations
      // handle user requirement first
      if (task.user_perferred_behavior != 1 &&
          last_task_.user_perferred_behavior == 1 &&
          lc_context_.trigger_when_appropriate) {
        LOG(WARNING) << "[HMI]clear cached stick trigger state. Cd alc.";
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      } else if (task.user_perferred_behavior != -1 &&
                 last_task_.user_perferred_behavior == -1 &&
                 lc_context_.trigger_when_appropriate) {
        LOG(WARNING) << "[HMI]clear cached stick trigger state. Cd alc.";
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      }

      if (task.user_perferred_behavior == 1 &&
          last_task_.user_perferred_behavior != 1) {
        // receive a lane change right trigger and previous action has been
        // completed
        if (task.lc_info.forbid_lane_change_right) {
          LOG(WARNING)
              << "[HMI]cannot stick [Right]. Will trigger when appropriate.";
          lc_context_.trigger_when_appropriate = true;
          lc_context_.lat = LateralBehavior::kLaneChangeRight;
        } else {
          lc_context_.completed = false;
          lc_context_.trigger_when_appropriate = false;
          lc_context_.trigger_time = stamp;
          lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
              stamp, bp_.cfg().function().stick_lane_change_in_seconds());
          lc_context_.ego_lane_id = ego_lane_id_;
          lc_context_.lat = LateralBehavior::kLaneChangeRight;
          lc_context_.type = LaneChangeTriggerType::kStick;
          last_lc_proposal_.trigger_time = stamp;
          LOG(WARNING) << std::fixed << std::setprecision(5)
                       << "[HMI]stick [Right] triggered "
                       << last_task_.user_perferred_behavior << "->"
                       << task.user_perferred_behavior << " in "
                       << bp_.cfg().function().stick_lane_change_in_seconds()
                       << " s. Trigger time " << lc_context_.trigger_time
                       << " and absolute action time: "
                       << lc_context_.desired_operation_time << ". Cd alc.";
        }
      } else if (task.user_perferred_behavior == -1 &&
                 last_task_.user_perferred_behavior != -1) {
        if (task.lc_info.forbid_lane_change_left) {
          LOG(WARNING)
              << "[HMI]cannot stick [Left]. Will trigger when appropriate.";
          lc_context_.trigger_when_appropriate = true;
          lc_context_.lat = LateralBehavior::kLaneChangeLeft;
        } else {
          lc_context_.completed = false;
          lc_context_.trigger_when_appropriate = false;
          lc_context_.trigger_time = stamp;
          lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
              stamp, bp_.cfg().function().stick_lane_change_in_seconds());
          lc_context_.ego_lane_id = ego_lane_id_;
          lc_context_.lat = LateralBehavior::kLaneChangeLeft;
          lc_context_.type = LaneChangeTriggerType::kStick;
          last_lc_proposal_.trigger_time = stamp;
          LOG(WARNING) << std::fixed << std::setprecision(5)
                       << "[HMI]stick [Left] triggered "
                       << last_task_.user_perferred_behavior << "->"
                       << task.user_perferred_behavior << " in "
                       << bp_.cfg().function().stick_lane_change_in_seconds()
                       << " s. Trigger time " << lc_context_.trigger_time
                       << " and absolute action time: "
                       << lc_context_.desired_operation_time << ". Cd alc.";
        }
      } else if (lc_context_.trigger_when_appropriate) {
        if (lc_context_.lat == LateralBehavior::kLaneChangeLeft &&
            !task.lc_info.forbid_lane_change_left) {
          lc_context_.completed = false;
          lc_context_.trigger_when_appropriate = false;
          lc_context_.trigger_time = stamp;
          lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
              stamp, bp_.cfg().function().stick_lane_change_in_seconds());
          lc_context_.ego_lane_id = ego_lane_id_;
          lc_context_.lat = LateralBehavior::kLaneChangeLeft;
          lc_context_.type = LaneChangeTriggerType::kStick;
          last_lc_proposal_.trigger_time = stamp;
          LOG(WARNING) << std::fixed << std::setprecision(5)
                       << "[HMI][[cached]] stick [Left] triggered in "
                       << bp_.cfg().function().stick_lane_change_in_seconds()
                       << " s. Trigger time " << lc_context_.trigger_time
                       << " and absolute action time: "
                       << lc_context_.desired_operation_time << ". Cd alc.";
        } else if (lc_context_.lat == LateralBehavior::kLaneChangeRight &&
                   !task.lc_info.forbid_lane_change_right) {
          lc_context_.completed = false;
          lc_context_.trigger_when_appropriate = false;
          lc_context_.trigger_time = stamp;
          lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
              stamp, bp_.cfg().function().stick_lane_change_in_seconds());
          lc_context_.ego_lane_id = ego_lane_id_;
          lc_context_.lat = LateralBehavior::kLaneChangeRight;
          lc_context_.type = LaneChangeTriggerType::kStick;
          last_lc_proposal_.trigger_time = stamp;
          LOG(WARNING) << std::fixed << std::setprecision(5)
                       << "[HMI][[cached]] stick [Right] triggered in "
                       << bp_.cfg().function().stick_lane_change_in_seconds()
                       << " s. Trigger time " << lc_context_.trigger_time
                       << " and absolute action time: "
                       << lc_context_.desired_operation_time << ". Cd alc.";
        }
      } else {
        if (last_lc_proposal_.valid &&
            ego_lane_id_ == last_lc_proposal_.ego_lane_id &&
            stamp > last_lc_proposal_.trigger_time &&
            last_lc_proposal_.lat != LateralBehavior::kLaneKeeping) {
          if ((last_lc_proposal_.lat == LateralBehavior::kLaneChangeLeft &&
               !task.lc_info.forbid_lane_change_left) ||
              (last_lc_proposal_.lat == LateralBehavior::kLaneChangeRight &&
               !task.lc_info.forbid_lane_change_right)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time =
                last_lc_proposal_.trigger_time +
                last_lc_proposal_.operation_at_seconds;
            lc_context_.ego_lane_id = last_lc_proposal_.ego_lane_id;
            lc_context_.lat = last_lc_proposal_.lat;
            lc_context_.type = LaneChangeTriggerType::kActive;
            last_lc_proposal_.trigger_time = stamp;
            if (last_lc_proposal_.lat == LateralBehavior::kLaneChangeLeft) {
              LOG(WARNING) << std::fixed << std::setprecision(5)
                           << "[HMI][[Active]] [Left] triggered in "
                           << last_lc_proposal_.operation_at_seconds
                           << " s. Trigger time " << lc_context_.trigger_time
                           << " and absolute action time: "
                           << lc_context_.desired_operation_time << ". Cd alc.";
            } else {
              LOG(WARNING) << std::fixed << std::setprecision(5)
                           << "[HMI][[Active]] [Right] triggered in "
                           << last_lc_proposal_.operation_at_seconds
                           << " s. Trigger time " << lc_context_.trigger_time
                           << " and absolute action time: "
                           << lc_context_.desired_operation_time << ". Cd alc.";
            }
          }
        }
      }
    }
  }  // if under control

  // any proposal will not last for more than one cycle
  last_lc_proposal_.valid = false;
  last_task_ = task;
}  // namespace planning

void EudmManager::SaveSnapshot(Snapshot* snapshot) {
  snapshot->valid = true;
  snapshot->plan_state = bp_.plan_state();
  snapshot->original_winner_id = bp_.winner_id();
  snapshot->processed_winner_id = bp_.winner_id();
  snapshot->action_script = bp_.action_script();
  snapshot->sim_res = bp_.sim_res();
  snapshot->risky_res = bp_.risky_res();
  snapshot->sim_info = bp_.sim_info();
  snapshot->cost_val_res = bp_.cost_val_res();
  snapshot->cost_structure_res = bp_.cost_structure_res();
  snapshot->forward_trajs = bp_.forward_trajs();
  snapshot->forward_lat_behaviors = bp_.forward_lat_behaviors();
  snapshot->forward_lon_behaviors = bp_.forward_lon_behaviors();
  snapshot->surround_trajs = bp_.surround_trajs();
}

void EudmManager::ConstructBehavior(common::SemanticBehavior* behavior) {
  if (not last_snapshot_.valid) return;
  int selected_seq_id = last_snapshot_.processed_winner_id;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_final;
  surround_trajs_final.emplace_back(
      last_snapshot_.surround_trajs[selected_seq_id]);
  behavior->lat_behavior =
      last_snapshot_.forward_lat_behaviors[selected_seq_id].front();
  behavior->lon_behavior =
      last_snapshot_.forward_lon_behaviors[selected_seq_id].front();
  behavior->forward_trajs = vec_E<vec_E<common::Vehicle>>{
      last_snapshot_.forward_trajs[selected_seq_id]};
  behavior->forward_behaviors = std::vector<LateralBehavior>{
      last_snapshot_.forward_lat_behaviors[selected_seq_id].front()};
  behavior->surround_trajs = surround_trajs_final;
  behavior->state = last_snapshot_.plan_state;
  behavior->ref_lane = last_snapshot_.ref_lane;
}

void EudmManager::ConstructPlainOutput(planning::eudm::PlainOutput* out) {
  using namespace planning::eudm;

  auto PlainStateTransform = [&](const vec_E<common::Vehicle>& vehicle_seq,
                                 std::vector<PlainState>& state_seq) -> void {
    for (auto& v : vehicle_seq) {
      common::State vehicle_state = v.state();
      state_seq.push_back(
          PlainState(vehicle_state.time_stamp, vehicle_state.vec_position[0],
                     vehicle_state.vec_position[1], vehicle_state.angle,
                     vehicle_state.velocity, vehicle_state.acceleration,
                     vehicle_state.curvature));
    }
  };

  if (not last_snapshot_.valid) return;
  int num_seqs = last_snapshot_.forward_trajs.size();
  // * pack elements
  {
    for (int i = 0; i < num_seqs; i++) {
      OutputElement ele;
      ele.sim_info = last_snapshot_.sim_info[i];

      // * macro cmd
      auto action_script = last_snapshot_.action_script;
      auto action_seq = action_script[i];
      for (const auto& action : action_seq) {
        ele.lat_cmds.push_back(static_cast<int>(action.lat));
        ele.lon_cmds.push_back(static_cast<int>(action.lon));
      }

      if (!last_snapshot_.sim_res[i]) {
        ele.valid = false;
      } else {
        out->valid = true;
        // * total state
        ele.valid = true;
        ele.risky = last_snapshot_.risky_res[i];
        // * final cost summarizes stage cost and may include addiontional
        // consistency cost
        ele.final_cost = last_snapshot_.cost_val_res[i];
        // * macro actions
        for (auto& lat : last_snapshot_.forward_lat_behaviors[i]) {
          ele.lat_behaviors.push_back(static_cast<int>(lat));
        }
        for (auto& lon : last_snapshot_.forward_lon_behaviors[i]) {
          ele.lon_behaviors.push_back(static_cast<int>(lon));
        }
        for (auto& cost : last_snapshot_.cost_structure_res[i]) {
          ele.stage_costs.push_back(
              PlainCost(cost.valid_sample_index_ub, cost.efficiency.ave(),
                        cost.safety.ave(), cost.navigation.ave(), cost.weight));
        }
        // * micro states
        PlainStateTransform(last_snapshot_.forward_trajs[i], ele.ego_traj);
        for (auto it = last_snapshot_.surround_trajs[i].begin();
             it != last_snapshot_.surround_trajs[i].end(); ++it) {
          std::vector<PlainState> plain_states;
          PlainStateTransform(it->second, plain_states);
          ele.surround_trajs[it->first] = plain_states;
        }
      }
      out->elements.push_back(ele);
    }
  }
  out->winner_element_id = last_snapshot_.processed_winner_id;
  out->original_winner_id = last_snapshot_.original_winner_id;
  out->best_cost =
      last_snapshot_.cost_val_res[last_snapshot_.processed_winner_id];
  out->ongoing_action_duration = last_snapshot_.action_script.front().front().t;
}

ErrorType EudmManager::EvaluateReferenceVelocity(
    const planning::eudm::Task& task, decimal_t* ref_vel) {
  if (!last_snapshot_.ref_lane.IsValid()) {
    *ref_vel = task.user_desired_vel;
    return kSuccess;
  }
  common::StateTransformer stf(last_snapshot_.ref_lane);
  common::FrenetState current_fs;
  stf.GetFrenetStateFromState(last_snapshot_.plan_state, &current_fs);

  decimal_t c, cc;
  decimal_t v_max_by_curvature;
  decimal_t v_ref = kInf;

  decimal_t a_comfort = bp_.cfg().sim().ego().lon().limit().soft_brake();
  decimal_t t_forward = last_snapshot_.plan_state.velocity / a_comfort;
  decimal_t s_forward =
      std::min(std::max(20.0, t_forward * last_snapshot_.plan_state.velocity),
               last_snapshot_.ref_lane.end());
  decimal_t resolution = 0.2;

  for (decimal_t s = current_fs.vec_s[0]; s < current_fs.vec_s[0] + s_forward;
       s += resolution) {
    if (last_snapshot_.ref_lane.GetCurvatureByArcLength(s, &c, &cc) ==
        kSuccess) {
      v_max_by_curvature =
          sqrt(bp_.cfg().sim().ego().lat().limit().acc() / fabs(c));  // 非常神奇的多层嵌套函数,需进一步探究
      v_ref = v_max_by_curvature < v_ref ? v_max_by_curvature : v_ref;
    }
  }

  *ref_vel =
      std::floor(std::min(std::max(v_ref - 2.0, 0.0), task.user_desired_vel));

  return kSuccess;
}

ErrorType EudmManager::ReselectByTask(const decimal_t stamp,
                                      const planning::eudm::Task& task,
                                      const Snapshot& snapshot,
                                      int* new_seq_id) {
  // *new_seq_id = snapshot.original_winner_id;
  int selected_seq_id;
  int num_seqs = snapshot.action_script.size();
  bool find_match = false;
  decimal_t cost = kInf;

  if (lc_context_.completed) {
    // only select lane keeeping behaviors if lane change completed
    for (int i = 0; i < num_seqs; i++) {
      if (!snapshot.sim_res[i]) continue;
      common::LateralBehavior lat_behavior;
      decimal_t operation_at_seconds;
      bool is_cancel_behavior;
      bp_.ClassifyActionSeq(snapshot.action_script[i], &operation_at_seconds,
                            &lat_behavior, &is_cancel_behavior);
      if (lat_behavior == common::LateralBehavior::kLaneKeeping) {
        find_match = true;
        if (snapshot.cost_val_res[i] < cost) {
          cost = snapshot.cost_val_res[i];
          selected_seq_id = i;
        }
      }
    }
  } else {
    // try to select the lane change behavior nearest desired lane change time
    // with strictly safe and potentially risky behavior
    if (!find_match) {
      for (int i = 0; i < num_seqs; i++) {
        if (!snapshot.sim_res[i]) continue;
        common::LateralBehavior lat_behavior;
        decimal_t operation_at_seconds;
        bool is_cancel_behavior;
        bp_.ClassifyActionSeq(snapshot.action_script[i], &operation_at_seconds,
                              &lat_behavior, &is_cancel_behavior);
        if (lat_behavior == lc_context_.lat ||
            lat_behavior == common::LateralBehavior::kLaneKeeping) {
          find_match = true;
          decimal_t gap =
              stamp + operation_at_seconds - lc_context_.desired_operation_time;
          decimal_t operate_cost =
              gap < 0.0 ? -bp_.desired_velocity() * gap *
                              bp_.cfg().cost().user().intime_operate_unit_cost()
                        : bp_.desired_velocity() *
                              std::min(gap, bp_.cfg()
                                                .cost()
                                                .user()
                                                .late_operate_saturate_time()) *
                              bp_.cfg().cost().user().late_operate_unit_cost();
          // printf("[CostX]operate at %lf, gap %lf operate cost %lf.\n",
          //        operation_at_seconds, gap, operate_cost);
          decimal_t operation_cancel_cost =
              is_cancel_behavior
                  ? bp_.desired_velocity() *
                        bp_.cfg().cost().user().cancel_operation_unit_cost()
                  : 0.0;
          decimal_t combined_cost =
              snapshot.cost_val_res[i] + operate_cost + operation_cancel_cost;
          if (combined_cost < cost) {
            cost = combined_cost;
            selected_seq_id = i;
          }
        }
      }
    }
  }

  if (!find_match) {
    return kWrongStatus;
  }
  *new_seq_id = selected_seq_id;
  return kSuccess;
}

ErrorType EudmManager::Run(
    const decimal_t stamp,
    const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
    const planning::eudm::Task& task) {
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]******************** Run: " << stamp
               << "******************";

  // * I : Prepare
  if (Prepare(stamp, map_ptr, task) != kSuccess) {
    return kWrongStatus;
  }
  // * II : RunOnce
  if (bp_.RunOnce() != kSuccess) {
    LOG(WARNING) << "[Eudm][Fatal]BP runonce failed.";
    return kWrongStatus;
  }
  // * III: Summarize
  Snapshot snapshot;
  SaveSnapshot(&snapshot);
  // * IV: Reselect
  if (ReselectByTask(stamp, task, snapshot, &snapshot.processed_winner_id) !=
      kSuccess) {
    LOG(WARNING) << "[Eudm][Fatal]Reselect failed.";
    return kWrongStatus;
  }
  LOG(WARNING) << "[Eudm]original id " << snapshot.original_winner_id
               << " reselect : " << snapshot.processed_winner_id;
  {
    std::ostringstream line_info;
    line_info << "[Eudm][Output]Reselected <if_risky:"
              << snapshot.risky_res[snapshot.processed_winner_id] << ">[";
    for (auto& a : snapshot.action_script[snapshot.processed_winner_id]) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (auto& a : snapshot.action_script[snapshot.processed_winner_id]) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << "]";
    for (auto& v : snapshot.forward_trajs[snapshot.processed_winner_id]) {
      line_info << std::fixed << std::setprecision(2) << "<"
                << v.state().time_stamp - stamp << "," << v.state().velocity
                << "," << v.state().acceleration << "," << v.state().curvature
                << ">";
    }
    LOG(WARNING) << line_info.str();
  }

  if (map_adapter_.map()->GetRefLaneForStateByBehavior(
          snapshot.plan_state, std::vector<int>(),
          snapshot.forward_lat_behaviors[snapshot.processed_winner_id].front(),
          150.0, 20.0, true, &(snapshot.ref_lane)) != kSuccess) {
    return kWrongStatus;
  }
  last_snapshot_ = snapshot;
  GenerateLaneChangeProposal(stamp, task);
  // * V: Update
  context_.is_valid = true;
  context_.seq_start_time = stamp;
  context_.action_seq = snapshot.action_script[snapshot.processed_winner_id];
  return kSuccess;
}

bool EudmManager::GetReplanDesiredAction(const decimal_t current_time,
                                         BtAction* desired_action) {
  if (!context_.is_valid) return false;
  decimal_t time_since_last_plan = current_time - context_.seq_start_time;
  if (time_since_last_plan < -kEPS) return false;
  decimal_t t_aggre = 0.0;
  bool find_match_action = false;
  int action_seq_len = context_.action_seq.size();
  for (int i = 0; i < action_seq_len; ++i) {
    t_aggre += context_.action_seq[i].t;
    if (time_since_last_plan + kEPS < t_aggre) {
      *desired_action = context_.action_seq[i];
      desired_action->t = t_aggre - time_since_last_plan;
      find_match_action = true;
      break;
    }
  }
  if (!find_match_action) {
    return false;
  }
  return true;
}

void EudmManager::Reset() { context_.is_valid = false; }

EudmPlanner& EudmManager::planner() { return bp_; }

}  // namespace planning