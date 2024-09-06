#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_MANAGER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_MANAGER_H_

#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"
/**
 * @brief since eudm itself is completely stateless, we use a manager to
 *        track the input and output of the eudm, to satisfy task level
 *        constraints.
 */
namespace planning {
class EudmManager {
 public:
  using BtLatAction = planning::DcpTree::BtLatAction; // using 声明帮助减少冗长的命名，并使代码更具可读性
  using BtLonAction = planning::DcpTree::BtLonAction;
  using BtAction = planning::DcpTree::BtAction;
  using LateralBehavior = common::LateralBehavior;
  using LongitudinalBehavior = common::LongitudinalBehavior;
  using CostStructure = planning::EudmPlanner::CostStructure;

  enum class LaneChangeTriggerType { kStick = 0, kActive }; // 定义了一个 enum class（强类型枚举） LaneChangeTriggerType，用于表示车道变换触发类型

  struct ReplanningContext {
    bool is_valid = false;
    decimal_t seq_start_time; // 貌似是一个自定义的数据类型
    std::vector<BtAction> action_seq;
  };

  struct ActivateLaneChangeRequest {
    decimal_t trigger_time;
    decimal_t desired_operation_time;
    int ego_lane_id;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
  };

  struct LaneChangeProposal {
    bool valid = false;
    decimal_t trigger_time = 0.0;
    decimal_t operation_at_seconds = 0.0;
    int ego_lane_id;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
  };

  struct LaneChangeContext {
    bool completed = true;
    bool trigger_when_appropriate = false;
    decimal_t trigger_time;
    decimal_t desired_operation_time;
    int ego_lane_id;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
    LaneChangeTriggerType type;
  };

  struct Snapshot {
    bool valid = false;
    int original_winner_id;
    int processed_winner_id;
    common::State plan_state; // common 貌似是一个命名空间（namespace）
    std::vector<std::vector<BtAction>> action_script;
    std::vector<bool> sim_res;
    std::vector<bool> risky_res;
    std::vector<std::string> sim_info;
    std::vector<decimal_t> cost_val_res;
    std::vector<std::vector<CostStructure>> cost_structure_res; // 二维数组
    vec_E<vec_E<common::Vehicle>> forward_trajs;  // vec_E<vec_E<common::Vehicle>> forward_trajs 可以理解为一个二维容器。vec_E 貌似是 std::vector 的别名或自定义类型
    std::vector<std::vector<LateralBehavior>> forward_lat_behaviors;
    std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors;
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs;  // 一个包含哈希表的二维容器，其键（int）映射到一个 vec_E<common::Vehicle> 类型的值
    common::Lane ref_lane;
  };

  EudmManager() {}  // 默认构造函数

  void Init(const std::string& config_path, const decimal_t work_rate);

  ErrorType Run(
      const decimal_t stamp,
      // std::shared_ptr<semantic_map_manager::SemanticMapManager>: 这是一个智能指针类型
      // SemanticMapManager 是 semantic_map_manager 命名空间中的一个类
      const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
      // 表示 map_ptr 是对 std::shared_ptr<semantic_map_manager::SemanticMapManager> 的引用，引用的使用避免了复制整个智能指针对象的开销
      const planning::eudm::Task& task);

  void Reset();

  void ConstructBehavior(common::SemanticBehavior* behavior);

  void ConstructPlainOutput(planning::eudm::PlainOutput* out);

  EudmPlanner& planner(); 

  // int: 这是函数的返回类型，表示函数返回一个整数值。
  // original_winner_id(): 这是函数的名称。在这个函数被调用时，它将返回 last_snapshot_ 对象中的 original_winner_id 成员变量的值。
  // const: 这个修饰符表示 original_winner_id() 是一个常量成员函数。它不会修改类的任何成员变量。这确保函数只读取对象的状态，而不会改变它。
  // return last_snapshot_.original_winner_id;: 这是函数的实现部分。它返回 last_snapshot_ 对象中的 original_winner_id 成员的值。
  int original_winner_id() const { return last_snapshot_.original_winner_id; }
  int processed_winner_id() const { return last_snapshot_.processed_winner_id; }

 private:
  decimal_t GetNearestFutureDecisionPoint(const decimal_t& stamp,
                                          const decimal_t& delta);

  ErrorType Prepare(
      const decimal_t stamp,
      const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
      const planning::eudm::Task& task);

  ErrorType EvaluateReferenceVelocity(const planning::eudm::Task& task,
                                      decimal_t* ref_vel);

  bool GetReplanDesiredAction(const decimal_t current_time,
                              BtAction* desired_action);

  void SaveSnapshot(Snapshot* snapshot);

  ErrorType ReselectByTask(const decimal_t stamp,
                           const planning::eudm::Task& task,
                           const Snapshot& snapshot, int* new_seq_id);

  void UpdateLaneChangeContextByTask(const decimal_t stamp,
                                     const planning::eudm::Task& task);

  ErrorType GenerateLaneChangeProposal(const decimal_t& stamp,
                                       const planning::eudm::Task& task);

  EudmPlanner bp_;
  EudmPlannerMapAdapter map_adapter_;
  decimal_t work_rate_{20.0};

  int ego_lane_id_;
  ReplanningContext context_;
  Snapshot last_snapshot_;
  planning::eudm::Task last_task_;  // planning 和 eudm 是命名空间，而 Task 是类型
  LaneChangeContext lc_context_;
  LaneChangeProposal last_lc_proposal_;
  std::vector<ActivateLaneChangeRequest> preliminary_active_requests_;
};

}  // namespace planning

#endif