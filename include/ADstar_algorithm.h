#ifndef PLANNER_ADSTAR_ALGORITHM_H
#define PLANNER_ADSTAR_ALGORITHM_H

#include <float.h>
#include <stdint.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <vector>

#include "grid_input.h"

/* float最大值 */
typedef std::numeric_limits<float> INF_f;

/* 对一张地图进行ARA*搜索 */
void SearchOneMap(int map_num_);

/* 打印结果统计 */
void PrintSumResult();

/* 每一个cell的信息，使用结构体定义 */
struct CellInfo {
  Points xoy;              //坐标点
  float h_value;           //当前状态到终点的启发距离值
  float g_value;           //起始点到当前点的距离值
  float rhs_value;         //引入的v(s)值
  float heuristic_factor;  //启发式权重因子

  /* 返回该Cell的f_value, 其值为key的第一元素 */
  inline float get_f_value() const {
    if (g_value >= rhs_value)
      return rhs_value + heuristic_factor * h_value;
    else
      return g_value + h_value;
  }

  /* 重载小于符号 */
  bool operator<(const CellInfo& pos) const {
    /* if f-vaule 相同，比较 g-vaule */
    if (get_f_value() == pos.get_f_value())
      return std::min(rhs_value, g_value) <
             std::min(pos.rhs_value, pos.g_value);
    else
      return get_f_value() < pos.get_f_value();
  }
};

class ADstar {
 public:
  ADstar(int16_t row_arg, int16_t column_arg, Points statr_arg, Points goal_arg,
         std::vector<Points> obstacle_list_arg);  //构造函数
  ADstar(const ADstar& as) = delete;              //不使用复制构造函数

  /* A*算法 */
  bool AstarAlgorithm();

  /* 得到临近四个点的坐标点 */
  std::vector<Points> GetNeighborsPoint(const Points& current_pos);

  /* 得到临近八个点的坐标点 */
  std::vector<Points> GetNeighborsPointEight(const Points& current_pos);

  /* 打印一次搜索结果 */
  void PrintSearchResult();

  /* LPA*特有的更新某点点的信息 */
  void UpdateVertex(const Points& pos);

  /* A*算法循环判断函数 */
  bool LoopFlg();

  /* 比较两个元素的大小 */
  bool TwoCellCompare(const CellInfo& cmp_1, const CellInfo& cmp_2);

  /* 弹出openlist中最小的元素 */
  void OpenLIstPopMinElem();

  /* 移除列表中的特定元素 */
  template <class T>
  void Remove(int16_t index, std::vector<T>* list);

  /* 更新当前点的临近点的信息 */
  std::vector<Points> UpdataMapInfo();

  /* 计算h_value */
  inline float DistenceToGoal(const Points& current) {
    return static_cast<float>(abs(current.first - current_start_.first) +
                              abs(current.second - current_start_.second));
  }

  /* 递减启发式的启发因子 */
  inline void DecreaseHeuristicFactor() {
    OpenLIstPopMinElem();
    heuristic_factor_ = std::min(heuristic_factor_,
                                 (start.g_value / (open_list_.back().g_value +
                                                   open_list_.back().h_value)));
    std::cout << "heuristic_factor_ =  " << heuristic_factor_ << std::endl;
  }

  /* 将inconslist中的元素push到openlist中 */
  void InconsPushOpenlist();

  /* 启发式因子改变，随即更新openlist */
  void UpdateOpenlisByNewFactor();

  /* 判断是否走到了终点 */
  inline bool ArriveGoal() const { return current_start_ == goal_pos_; }

  /* 沿着当前path前进一步 */
  inline void StartMove() {
    current_start_ = current_path_.back();
    current_path_.pop_back();
    ++move_step_nums_;
  }

  /* 判断要移动的下一个点是否为障碍物 */
  inline bool NextStepIsInObstacleList() {
    return IsInList(current_path_.back(), current_obstacle_list_);
  }

  /* 添加障碍物，每次添加一个障碍物列表 */
  void ChangeCurrentPointBlocked(const std::vector<Points>& obs_list);

  /* 添加障碍物,每次添加一个障碍物点 */
  void ChangeOfInceaseObstacle(const Points& pos);

  /* 设置当前起点 */
  inline void StartSet() {
    if (current_start_ != start_pos_)
      start = consistent_cell_info_list_[current_start_];
  }

  /* 打印计数结果 */
  void PrintCountResult();

  /* 获取类内私有成员函数 */
  inline int get_row() const { return row_; }
  inline int get_column() const { return column_; }
  inline Points get_start_pos() const { return start_pos_; }
  inline Points get_goal_pos() const { return goal_pos_; }
  inline Points get_current_start() const { return current_start_; }
  inline std::vector<Points> get_map_obstacle_list() const {
    return map_obstacle_list_;
  }
  inline std::vector<Points> get_current_obstacle_list() const {
    return current_obstacle_list_;
  }
  inline std::vector<Points> get_current_path() const { return current_path_; }
  inline int get_all_expand_nums() const { return all_expand_points_count_; }
  inline int get_search_nums() const { return search_nums_count_; }
  inline int get_move_step_nums() const { return move_step_nums_; }
  inline float get_heur_factor() const { return heuristic_factor_; }

  /* 设置类内私有成员函数 */
  inline float& set_heur_factor() { return heuristic_factor_; }
  std::vector<Points>& set_current_obstacle() { return current_obstacle_list_; }

 private:
  /* map info */
  int16_t row_, column_;                   // map的行、列数
  Points start_pos_, goal_pos_;            // map的起点与终点
  std::vector<Points> map_obstacle_list_;  //所有障碍物list
  CellInfo start, goal;

  float heuristic_factor_;             //启发式权重因子
  Points last_start_, current_start_;  //当前起点

  std::vector<CellInfo> open_list_;    //当前openlist
  std::vector<CellInfo> incons_list_;  //当前inconslist
  std::vector<Points> close_list_;     //当前closelist

  std::vector<Points> current_path_;           //当前路径
  std::vector<Points> current_obstacle_list_;  //当前已经获得的障碍物列表
  std::map<Points, Points> current_save_path_hash_;  //当前存储path的hash
  std::map<Points, CellInfo>
      consistent_cell_info_list_;  //存储扩展过且一致的cell

  int32_t current_expand_points_count_,  //一次算法中的expand计数
      all_expand_points_count_,          //整体算法中的expand计数
      search_nums_count_,                //搜索次数计数
      move_step_nums_;                   //移动步数计数

  /* 判断点是否在list中 */
  inline bool IsInList(const Points& point,
                       const std::vector<Points>& list) const {
    return std::find(list.begin(), list.end(), point) != list.end();
  }
};

/* 移除openlist中的指定点 */
template <class T>
void ADstar::Remove(int16_t index, std::vector<T>* list) {
  if (list->size() >= 2) list->at(index) = list->back();
  list->pop_back();
}

#endif