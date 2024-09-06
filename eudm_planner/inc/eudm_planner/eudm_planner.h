#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_

#include <algorithm>
#include <memory>
#include <string>
#include <thread>

#include "common/basics/basics.h"
#include "common/basics/ma_filter.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/state.h"
#include "eudm_planner/dcp_tree.h"
#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/map_interface.h"
#include "forward_simulator/onlane_forward_simulation.h"

#include "eudm_config.pb.h"

namespace planning {

// EudmPlanner 类继承了 Planner 类的所有公有（public）和保护（protected）成员,并可以访问 Planner 类的公有和保护成员，同时它也可以添加自己的成员
class EudmPlanner : public Planner {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using LateralBehavior = common::LateralBehavior;
  using LongitudinalBehavior = common::LongitudinalBehavior;
  using BtAction = DcpTree::BtAction;
  using BtLonAction = DcpTree::BtLonAction;
  using BtLatAction = DcpTree::BtLatAction;
  using Cfg = planning::eudm::Config;
  using LaneChangeInfo = planning::eudm::LaneChangeInfo;

  struct EfficiencyCost {
    decimal_t ego_to_desired_vel = 0.0;
    decimal_t leading_to_desired_vel = 0.0;
    decimal_t ave() const {
      return (ego_to_desired_vel + leading_to_desired_vel) / 2.0;
    }
  };

  struct SafetyCost {
    decimal_t rss = 0.0;
    decimal_t occu_lane = 0.0;
    decimal_t ave() const { return (rss + occu_lane) / 2.0; }
  };

  struct NavigationCost {
    decimal_t lane_change_preference = 0.0;
    decimal_t ave() const { return lane_change_preference; }
  };

  struct CostStructure {
    // * associate cost with micro action using this index
    int valid_sample_index_ub;
    // * efficiency
    EfficiencyCost efficiency;
    // * safety
    SafetyCost safety;
    // * navigation
    NavigationCost navigation;
    decimal_t weight = 1.0;
    decimal_t ave() const {
      return (efficiency.ave() + safety.ave() + navigation.ave()) * weight;
    }

    // 这段代码定义了如何将 CostStructure 对象格式化并输出到标准流中，以便更方便地查看对象的状态
    // 友元函数 (friend): 使 operator<< 函数可以访问 CostStructure 类的私有和保护成员。这通常用于实现类的自定义输出操作
    // std::ostream& operator<<: 重载了 << 运算符，用于将 CostStructure 对象的内容输出到 std::ostream 对象（如 std::cout）
    // os << std::fixed: 设置输出流 os 使用固定小数点表示法，而不是科学计数法。这里有重复的 os << std::fixed，可能是冗余的
    // os << std::setprecision(3): 设置输出的浮点数的精度为小数点后三位
    // const 表示在函数内部不能修改 cost 引用所指向的 CostStructure 对象的任何成员
    friend std::ostream& operator<<(std::ostream& os,
                                    const CostStructure& cost) {
      os << std::fixed;
      os << std::fixed;
      os << std::setprecision(3);
      os << "(efficiency: "
         << "ego (" << cost.efficiency.ego_to_desired_vel << ") + leading ("
         << cost.efficiency.leading_to_desired_vel << "), safety: ("
         << cost.safety.rss << "," << cost.safety.occu_lane
         << "), navigation: " << cost.navigation.lane_change_preference << ")";
      return os;
    }
  };

  struct PredictedVehicle {
    common::Vehicle vehicle;  // common::Vehicle 应该是定义在 common 命名空间中的一个类或结构体，表示车辆的相关信息
    // std::set 是 C++ 标准库中的一个关联容器，元素是唯一的，并且按照升序排列
    // std::pair 是一个模板类，用于存储一对值
    // 由于 std::set 按照元素的 < 运算符排序，std::pair 的比较是首先比较第一个值（decimal_t），如果第一个值相等，则比较第二个值（LateralBehavior）
    std::set<std::pair<decimal_t, LateralBehavior>> lat_intentions;

    PredictedVehicle() {} // PredictedVehicle 结构体的默认构造函数
    PredictedVehicle(const common::Vehicle& vehicle_, const std::set<std::pair<decimal_t, LateralBehavior>>& lat_intentions_) // 带参数的构造函数，用于初始化 PredictedVehicle 对象的两个成员
        : vehicle(vehicle_), lat_intentions(lat_intentions_) {} // 初始化列表用于在对象创建时初始化成员变量
  };

  std::string Name() override;  // 在实际使用中，这行代码通常出现在派生类中，该类继承自一个基类，并且基类中有一个虚函数 Name()，该虚函数在派生类中被重写以提供特定的实现

  ErrorType Init(const std::string config) override;

  ErrorType RunOnce() override;

  void set_map_interface(EudmPlannerMapItf* itf);
  /**
   * @brief set desired velocity
   */
  void set_desired_velocity(const decimal_t desired_vel);

  void set_lane_change_info(const LaneChangeInfo& lc_info);

  ErrorType RunEudm();

  Behavior behavior() const;

  std::vector<BtAction> winner_action_seq() const { return winner_action_seq_; }

  decimal_t desired_velocity() const;

  vec_E<vec_E<common::Vehicle>> forward_trajs() const;

  int winner_id() const;

  std::vector<bool> sim_res() const // 函数声明后的 const 修饰符表示该函数不会修改类的任何成员变量。它保证函数 sim_res 在执行过程中不会改变类的状态
  {
    std::vector<bool> ret;
    for (auto& r : sim_res_)  // 遍历 sim_res_ 中的每一个元素。auto& 表示 r 是 sim_res_ 中元素的引用，sim_res_ 应该是一个容器，如 std::vector<int>
    {
      if (r == 0) {
        ret.push_back(false);
      } else {
        ret.push_back(true);
      }
    }
    return ret;
  }

  std::vector<bool> risky_res() const {
    std::vector<bool> ret;
    for (auto& r : risky_res_) {
      if (r == 0) {
        ret.push_back(false);
      } else {
        ret.push_back(true);
      }
    }
    return ret;
  }
  std::vector<std::string> sim_info() const { return sim_info_; }
  std::vector<decimal_t> cost_val_res() const { return cost_val_res_; }
  std::vector<std::vector<CostStructure>> cost_structure_res() const {
    return cost_structure_res_;
  }
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors() const {
    return forward_lat_behaviors_;
  }
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors() const {
    return forward_lon_behaviors_;
  }
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs()
      const {
    return surround_trajs_;
  }
  common::State plan_state() {
    State ego_state;
    map_itf_->GetEgoState(&ego_state);
    return ego_state;
  }
  std::vector<std::vector<BtAction>> action_script() {
    return dcp_tree_ptr_->action_script();
  }

  std::unordered_map<int, PredictedVehicle> predicted_vehicles() const {
    return predicted_vehicles_;
  }

  // const 在成员函数 cfg() 的末尾表示该函数不会修改对象的任何成员变量。换句话说，cfg() 是一个只读函数，它承诺不对类的状态进行任何更改
  // const Cfg& 表示函数 cfg() 返回一个 Cfg 类型的常量引用
  const Cfg& cfg() const { return cfg_; }

  EudmPlannerMapItf* map_itf() const;

  void UpdateDcpTree(const BtAction& ongoing_action);

  ErrorType ClassifyActionSeq(const std::vector<BtAction>& action_seq,
                              decimal_t* operation_at_seconds,
                              common::LateralBehavior* lat_behavior,
                              bool* is_cancel_operation);

 protected:
  ErrorType ReadConfig(const std::string config_path);

  ErrorType GetSimParam(const planning::eudm::ForwardSimDetail& cfg,
                        OnLaneForwardSimulation::Param* sim_param);

  ErrorType GetPotentialLaneIds(const int source_lane_id,
                                const LateralBehavior& beh,
                                std::vector<int>* candidate_lane_ids) const;
  ErrorType UpdateEgoLaneId(const int new_ego_lane_id);

  ErrorType JudgeBehaviorByLaneId(const int ego_lane_id_by_pos,
                                  LateralBehavior* behavior_by_lane_id);

  ErrorType UpdateEgoBehavior(const LateralBehavior& behavior_by_lane_id);

  ErrorType TranslateBtActionToLonLatBehavior(const BtAction& action,
                                              LateralBehavior* lat,
                                              LongitudinalBehavior* lon) const;

  // * simulation control loop
  ErrorType InitIntentionBelief(
      const common::SemanticVehicleSet& semantic_key_vehicles,
      const common::Vehicle& ego_vehicle,
      const common::VehicleSet& surrounding_vehicles,
      std::unordered_map<int, PredictedVehicle>* predicted_vehicles);

  ErrorType SimulateActionSequence(
      const common::Vehicle& ego_vehicle,
      const std::unordered_map<int, PredictedVehicle>& predicted_vehicles,
      const std::vector<BtAction>& action_seq, const int& seq_id);  // const 关键字表示 seq_id 是一个常量，它指向的 int 值不能被修改

  ErrorType SimulateScenario(
      const common::Vehicle& ego_vehicle,
      const common::SemanticVehicleSet& semantic_vehicles,
      const std::vector<BtAction>& action_seq, const int& seq_id,
      const int& sub_seq_id, std::vector<int>* sub_sim_res,
      std::vector<int>* sub_risky_res, std::vector<std::string>* sub_sim_info,
      std::vector<std::vector<CostStructure>>* sub_cost_res,
      vec_E<vec_E<common::Vehicle>>* sub_forward_trajs,
      std::vector<std::vector<LateralBehavior>>* sub_forward_lat_behaviors,
      std::vector<std::vector<LongitudinalBehavior>>* sub_forward_lon_behaviors,
      vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>*
          sub_surround_trajs);

  ErrorType SimulateSingleAction(
      const BtAction& action, const decimal_t& ego_desired_vel,
      const std::unordered_map<int, double>& est_desired_vel_set,
      common::SemanticVehicle* ego_semantic_vehicle_this_layer,
      common::SemanticVehicleSet* semantic_vehicle_set_this_layer,
      vec_E<common::Vehicle>* ego_traj_multisteps,
      std::unordered_map<int, vec_E<common::Vehicle>>*
          surround_trajs_multisteps);

  ErrorType MultiAgentSimForward(
      const int ego_id, const decimal_t& ego_desired_vel,
      const common::SemanticVehicleSet& semantic_vehicle_set,
      const std::unordered_map<int, double>& est_desired_vel_set,
      const decimal_t& sim_time_resolution, const decimal_t& sim_time_total,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs); // std::unordered_map是 C++ 标准库中的一个关联容器哈希表

  // * evaluation functions
  ErrorType CostFunction(
      const LongitudinalBehavior& ego_lon_behavior_this_layer,
      const LateralBehavior& ego_lat_behavior_this_layer,
      const decimal_t duration, const common::SemanticVehicle& ego_vehicle,
      const common::SemanticVehicleSet& agent_vehicles,
      const vec_E<common::Vehicle>& ego_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
      const common::LateralBehavior& seq_lat_behavior,
      const bool is_cancel_behavior, bool verbose, CostStructure* cost,
      bool* is_risky, std::set<int>* risky_ids);

  ErrorType StrictSafetyCheck(
      const vec_E<common::Vehicle>& ego_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
      bool* is_safe, int* collided_id);

  ErrorType EvaluateSafetyStatus(const vec_E<common::Vehicle>& traj_a,
                                 const vec_E<common::Vehicle>& traj_b,
                                 decimal_t* cost, bool* is_rss_safe,
                                 int* risky_id);
  ErrorType EvaluateSinglePolicyTrajs(
      const std::vector<CostStructure>& cost_res,
      const std::vector<BtAction>& action_seq, decimal_t* score);

  ErrorType EvaluateMultiThreadSimResults(int* winner_id,
                                          decimal_t* winner_cost);

  // * simulation util functions
  ErrorType UpdateSimulationSetup(
      const BtAction action, const common::State& ego_state_origin,
      LateralBehavior* ego_lat_behavior_this_layer,
      LongitudinalBehavior* ego_lon_behavior_this_layer,
      common::Lane* ego_ref_lane) const;

  bool CheckIfLateralActionFinished(const common::State& cur_state,
                                    const int& action_ref_lane_id,
                                    const LateralBehavior& lat_behavior,
                                    int* current_lane_id) const;

  ErrorType UpdateLateralActionSequence(
      const int cur_idx, std::vector<BtAction>* action_seq) const;

  ErrorType PrepareMultiThreadContainers(const int n_sequence);

  ErrorType GetSemanticVehiclesFromPredictedVehicles(
      const std::unordered_map<int, PredictedVehicle>& predicted_vehicles,
      common::SemanticVehicleSet* smv_set) const;

  // * map
  EudmPlannerMapItf* map_itf_{nullptr};
  // * action
  DcpTree* dcp_tree_ptr_;
  // * setup
  Cfg cfg_;
  LaneChangeInfo lc_info_;
  decimal_t desired_velocity_{5.0};
  decimal_t sim_time_total_ = 0.0;
  std::set<int> pre_deleted_seq_ids_;
  int ego_lane_id_{kInvalidLaneId};
  std::vector<int> potential_lcl_lane_ids_;
  std::vector<int> potential_lcr_lane_ids_;
  std::vector<int> potential_lk_lane_ids_;
  common::Lane rss_lane_;
  common::StateTransformer rss_stf_;
  // * result
  int winner_id_ = 0;
  decimal_t winner_score_ = 0.0;
  std::vector<BtAction> winner_action_seq_;
  std::vector<int> sim_res_;
  std::vector<int> risky_res_;
  std::vector<std::string> sim_info_;
  std::vector<decimal_t> cost_val_res_;
  std::vector<std::vector<CostStructure>> cost_structure_res_;
  vec_E<vec_E<common::Vehicle>> forward_trajs_;
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors_;
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors_;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;
  // temporary
  common::VehicleSet nearby_vehicles_;
  std::unordered_map<int, PredictedVehicle> predicted_vehicles_;
};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_