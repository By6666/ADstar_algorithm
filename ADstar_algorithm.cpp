#include "include/ADstar_algorithm.h"
std::vector<std::string> sum_result;  //存储总结输出结果的容器
const float kHeuridticFactor = 10.0f;

/* 进行一次总体的ARA*算法
 * 输入：文件序号
 * 输出：无
 *  */
void SearchOneMap(int map_num_) {
  /* 获得map信息 */
  GrideInput map_info(map_num_);
  map_info.GetOneGrid();
  map_info.PrintMap();  //打印原始map

  /* 数据传入，构造D* lite算法对象 */
  ADstar ADstar_algorithm(map_info.get_grid_rows(), map_info.get_grid_columns(),
                          map_info.get_start_pos(), map_info.get_goal_pos(),
                          map_info.get_obstacle_pos());

  bool ARAstar_success_flg = false;
  while (1) {
    /* ARA*部分 */
    while (ADstar_algorithm.get_heur_factor() > 1.0f) {
      /* 规划路径 */
      std::cout << "**********" << std::endl;
      std::cout << "search num : " << ADstar_algorithm.get_search_nums() + 1
                << std::endl;
      bool flg = ADstar_algorithm.AstarAlgorithm();

      /* 如果失败，结束循环 */
      if (!flg) {
        std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
        std::cout << "|final result : no path to goal !!|" << std::endl;
        std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
        ADstar_algorithm.PrintCountResult();
        ARAstar_success_flg = true;
        break;
      }
      ADstar_algorithm.InconsPushOpenlist();
      ADstar_algorithm.DecreaseHeuristicFactor();
      ADstar_algorithm.UpdateOpenlisByNewFactor();
    }

    /* 移动当前起点部分 */
    if (ARAstar_success_flg) {
      break;
    } else {
      /* 沿着当前path移动 */
      while (!ADstar_algorithm.get_current_path().empty()) {
        ADstar_algorithm.ChangeCurrentPointBlocked(
            ADstar_algorithm.UpdataMapInfo());
        /* 如果下一个点是障碍物 */
        if (ADstar_algorithm.NextStepIsInObstacleList())
          break;
        else {
          ADstar_algorithm.StartMove();
        }
      }

      /* 走到了终点 */
      if (ADstar_algorithm.ArriveGoal()) {
        std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
        std::cout << "|final result: get goal successflly!!|" << std::endl;
        std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
        ADstar_algorithm.PrintCountResult();
        break;
      }

      /* 初始化新的一次ARA*计算 */
      ADstar_algorithm.set_heur_factor() = kHeuridticFactor;
      ADstar_algorithm.UpdateOpenlisByNewFactor();
      ADstar_algorithm.StartSet();
    }
  }

  /* 结果统计 */
  sum_result.push_back(
      std::to_string(map_num_ + 1) + "          " +
      std::to_string(ADstar_algorithm.get_search_nums()) + "          " +
      std::to_string(ADstar_algorithm.get_all_expand_nums()) + " " +
      std::to_string(ADstar_algorithm.get_move_step_nums()));
}

/* 构造函数
 * 输入：地图的行数、列数、起点、终点、障碍物点
 * 输出：无
 * */
ADstar::ADstar(int16_t row_arg, int16_t column_arg, Points statr_arg,
               Points goal_arg, std::vector<Points> obstacle_list_arg)
    : row_(row_arg),
      column_(column_arg),
      start_pos_(statr_arg),
      goal_pos_(goal_arg),
      map_obstacle_list_(obstacle_list_arg) {
  /* 类内成员初始化 */
  move_step_nums_ = 0;                        //移动步数
  search_nums_count_ = 0;                     //搜索次数初始化
  all_expand_points_count_ = 0;               //扩展点个数计数初始化
  current_expand_points_count_ = 0;           //当前扩展点计数
  heuristic_factor_ = kHeuridticFactor;       //启发式权重因子
  last_start_ = current_start_ = start_pos_;  //赋值当前起点为最初的起点
  // current_obstacle_list_ = map_obstacle_list_;

  /* 初始化 */
  goal = {goal_pos_, DistenceToGoal(goal_pos_), INF_f::max(), 0.0f,
          heuristic_factor_};
  start = {current_start_, 0.0f, INF_f::max(), INF_f::max(), heuristic_factor_};
  open_list_.push_back(goal);
}

/* 在一次A*算法中获得当前点的四个临近点的坐标
 * 输入：当前点的坐标
 * 输出：四个临近点的坐标
 * */
std::vector<Points> ADstar::GetNeighborsPoint(const Points& current_pos) {
  std::vector<Points> neighbors;
  /* UP */
  if ((current_pos.first - 1) >= 0 &&
      !IsInList(Points(current_pos.first - 1, current_pos.second),
                current_obstacle_list_)) {
    neighbors.push_back(Points(current_pos.first - 1, current_pos.second));
  }
  /* Down */
  if ((current_pos.first + 1) < row_ &&
      !IsInList(Points(current_pos.first + 1, current_pos.second),
                current_obstacle_list_)) {
    neighbors.push_back(Points(current_pos.first + 1, current_pos.second));
  }
  /* Left */
  if ((current_pos.second - 1) >= 0 &&
      !IsInList(Points(current_pos.first, current_pos.second - 1),
                current_obstacle_list_)) {
    neighbors.push_back(Points(current_pos.first, current_pos.second - 1));
  }
  /* Right */
  if ((current_pos.second + 1) < column_ &&
      !IsInList(Points(current_pos.first, current_pos.second + 1),
                current_obstacle_list_)) {
    neighbors.push_back(Points(current_pos.first, current_pos.second + 1));
  }
  return neighbors;
}

/* 在整体A*算法中获得当前起点的四个临近点的信息
 * 输入：无
 * 输出：无
 * */
std::vector<Points> ADstar::UpdataMapInfo() {
  std::vector<Points> obstacle_list;
  /* UP */
  if ((current_start_.first - 1) >= 0) {
    if (IsInList(Points(current_start_.first - 1, current_start_.second),
                 map_obstacle_list_) &&
        !IsInList(Points(current_start_.first - 1, current_start_.second),
                  current_obstacle_list_))
      obstacle_list.push_back(
          Points(current_start_.first - 1, current_start_.second));
  }
  /* Down */
  if ((current_start_.first + 1) < row_) {
    if (IsInList(Points(current_start_.first + 1, current_start_.second),
                 map_obstacle_list_) &&
        !IsInList(Points(current_start_.first + 1, current_start_.second),
                  current_obstacle_list_))
      obstacle_list.push_back(
          Points(current_start_.first + 1, current_start_.second));
  }
  /* Left */
  if ((current_start_.second - 1) >= 0) {
    if (IsInList(Points(current_start_.first, current_start_.second - 1),
                 map_obstacle_list_) &&
        !IsInList(Points(current_start_.first, current_start_.second - 1),
                  current_obstacle_list_))
      obstacle_list.push_back(
          Points(current_start_.first, current_start_.second - 1));
  }
  /* Right */
  if ((current_start_.second + 1) < column_) {
    if (IsInList(Points(current_start_.first, current_start_.second + 1),
                 map_obstacle_list_) &&
        !IsInList(Points(current_start_.first, current_start_.second + 1),
                  current_obstacle_list_))
      obstacle_list.push_back(
          Points(current_start_.first, current_start_.second + 1));
  }
  return obstacle_list;
}

/* 更新点的信息,g、rhs、f等
 * 输入：需要更新的点的坐标
 * 输出：无
 *  */
void ADstar::UpdateVertex(const Points& pos) {
  std::cout << "update pos:(" << pos.first << "," << pos.second << ") | ";

  /* 如果扩展到了起点，立马退出函数 */
  if (pos == goal_pos_) {
    std::cout << "pos==goal ---> quit";
    return;
  }

  /* 去除openlist中与pos相同的点 */
  for (int16_t i = 0; i < open_list_.size(); ++i) {
    if (pos == open_list_[i].xoy) {
      Remove<CellInfo>(i, &open_list_);
      current_save_path_hash_.erase(pos);
      std::cout << "have same pos in openlist --> remove | ";
    }
  }
  /* 去除inconslist中与pos相同的点 */
  for (int16_t i = 0; i < incons_list_.size(); ++i) {
    if (pos == incons_list_[i].xoy) {
      Remove<CellInfo>(i, &incons_list_);
      current_save_path_hash_.erase(pos);
      std::cout << "have same pos in inconslist --> remove | ";
    }
  }

  /* 寻找临近点中最小的g_value */
  std::vector<Points> neighbors = GetNeighborsPoint(pos);
  float min_g = INF_f::max();
  int8_t min_index = 0;
  int8_t flg_have_neighbors = 0;
  std::cout << std::endl;
  for (int8_t i = 0; i < neighbors.size(); ++i) {
    float temp = 0.0f;
    temp = (consistent_cell_info_list_.find(neighbors[i]) !=
            consistent_cell_info_list_.end())
               ? consistent_cell_info_list_[neighbors[i]].g_value
               : INF_f::max();

    std::cout << "   nei pos:(" << neighbors[i].first << ","
              << neighbors[i].second << ")-->g: " << temp;
    if (consistent_cell_info_list_.find(neighbors[i]) !=
        consistent_cell_info_list_.end()) {
      std::cout << "  rhs: "
                << consistent_cell_info_list_[neighbors[i]].rhs_value;
    } else {
      std::cout << "  rhs: INF";
    }

    /* 在有相同的min_gq情况下，选择最后一个 */
    // min_g = std::min(min_g, temp);
    // if (min_g == temp) min_index = i;

    /* 在有相同的min_gq情况下，选择第一个 */
    if (min_g > temp) {
      min_g = temp;
      min_index = i;
    }
    ++flg_have_neighbors;
  }
  std::cout << std::endl;

  /* 判断有无相邻点 */
  if (!flg_have_neighbors) {
    current_save_path_hash_.erase(pos);
    std::cout << "no neighbors,arround by obstscle --> quit | ";
    return;
  } else {
    std::cout << "pre is "
              << "(" << neighbors[min_index].first << ","
              << neighbors[min_index].second << ")  min_g:" << min_g << " | ";
  }

  /* 找到了终点 */
  if (pos == current_start_) {
    if (min_g != INF_f::max()) {
      start.rhs_value = min_g + 1;
    } else {
      start.rhs_value = min_g;
    }
    std::cout << "goal.rhs : " << start.rhs_value
              << "  goal.f : " << start.get_f_value() << " | ";
    std::cout << "find goal !" << std::endl;
  }

  /* push pos into openlist */
  if (consistent_cell_info_list_.find(pos) !=
      consistent_cell_info_list_.end()) {
    consistent_cell_info_list_[pos].rhs_value =
        (min_g == INF_f::max()) ? min_g : min_g + 1;

    if (consistent_cell_info_list_[pos].rhs_value !=
        consistent_cell_info_list_[pos].g_value) {
      consistent_cell_info_list_[pos].heuristic_factor = heuristic_factor_;
      consistent_cell_info_list_[pos].h_value = DistenceToGoal(pos);

      std::cout << "expanded--"
                << "rhs :" << consistent_cell_info_list_[pos].rhs_value
                << "  g: " << consistent_cell_info_list_[pos].g_value
                << "  f: " << consistent_cell_info_list_[pos].get_f_value();

      if (IsInList(pos, close_list_)) {
        incons_list_.push_back(consistent_cell_info_list_[pos]);
        std::cout << "  push-->inconslist | ";
      } else {
        open_list_.push_back(consistent_cell_info_list_[pos]);
        std::cout << "  push-->openlist | ";
      }

    } else {
      std::cout << "rhs=g  --> not push | ";
    }
    current_save_path_hash_[pos] = neighbors[min_index];

  } else {
    if (min_g != INF_f::max()) {
      CellInfo temp = {pos, DistenceToGoal(pos), INF_f::max(), min_g + 1,
                       heuristic_factor_};

      current_save_path_hash_[pos] = neighbors[min_index];

      std::cout << "unexpand--"
                << "rhs :" << min_g + 1 << "  g: " << INF_f::max()
                << "  f: " << temp.get_f_value();
      if (IsInList(pos, close_list_)) {
        incons_list_.push_back(temp);
        std::cout << "  push-->inconslist | ";
      } else {
        open_list_.push_back(temp);
        std::cout << "  push-->openlist | ";
      }
    } else {
      std::cout << "rhs=g=INF  --> not push | ";
    }
  }
}
/* 将openlist中的最小元素放到末尾
 * 输入：无
 * 输出：无
 * */
void ADstar::OpenLIstPopMinElem() {
  /* 如果openlist为空，弹出INF */
  if (open_list_.empty()) {
    open_list_.push_back(
        CellInfo({Points(-1, -1), INF_f::max(), INF_f::max(), INF_f::max()}));
    return;
  }

  if (open_list_.size() < 2) return;

  std::vector<CellInfo>::iterator itr =
      std::min_element(open_list_.begin(), open_list_.end());

  CellInfo temp = *itr;
  *itr = open_list_.back();
  open_list_.back() = temp;
}

/* A*算法中主循环判断条件
 * 输入：无
 * 输出：无
 *  */
bool ADstar::LoopFlg() {
  OpenLIstPopMinElem();
  TwoCellCompare(start, open_list_.back());
}

/* 判断cmp1 > cmp2是否成立
 * 输入：需要判断的两个数
 * 输出：true or false
 *  */
bool ADstar::TwoCellCompare(const CellInfo& cmp_1, const CellInfo& cmp_2) {
  if (cmp_1.get_f_value() == cmp_2.get_f_value())
    return std::min(cmp_1.g_value, cmp_1.rhs_value) >
           std::min(cmp_2.g_value, cmp_2.rhs_value);
  else
    return cmp_1.get_f_value() > cmp_2.get_f_value();
}

/* 执行一次A*算法
 * 输入：当前起点与终点的信息
 * 输出：false：搜索失败; true：搜索成功
 * */
bool ADstar::AstarAlgorithm() {
  std::vector<Points> path_result_list;  //存放本次搜索的路径

  int8_t search_successful_flg = 0;  //判断flg
  current_expand_points_count_ = 0;

  /* 搜索循环 */
  while (LoopFlg() || start.g_value != start.rhs_value) {
    CellInfo current_cell = open_list_.back();
    open_list_.pop_back();

    std::cout << std::endl
              << "current_cell****"
              << "(" << current_cell.xoy.first << "," << current_cell.xoy.second
              << ")   "
              << "  new.f : " << current_cell.get_f_value() << " | ";

    std::cout << std::endl
              << "***begin expand--------------------------" << std::endl;
    int8_t neighbor_expand_cnt = 0;
    std::cout << "rhs :" << current_cell.rhs_value
              << "  g: " << current_cell.g_value
              << "  f: " << current_cell.get_f_value() << " | ";

    if (current_cell.g_value > current_cell.rhs_value) {
      /* 使cell consistent */
      current_cell.g_value = current_cell.rhs_value;
      close_list_.push_back(current_cell.xoy);
      if (current_cell.xoy == current_start_) start.g_value = start.rhs_value;
      std::cout << "g > rhs -->"
                << "new g:" << current_cell.g_value << std::endl;

      /* 存储已经扩展过的cell,该Cell应该是 consistent的 */
      consistent_cell_info_list_[current_cell.xoy] = current_cell;
      std::vector<Points> neighbors_pos = GetNeighborsPoint(current_cell.xoy);
      for (int8_t i = 0; i < neighbors_pos.size(); ++i) {
        ++neighbor_expand_cnt;
        UpdateVertex(neighbors_pos[i]);
        std::cout << std::endl;
      }
    } else {
      current_cell.g_value = INF_f::max();
      std::cout << "g <= rhs -->"
                << "new g:" << current_cell.g_value << std::endl;
      if (current_cell.xoy == current_start_) {
        start.g_value = INF_f::max();
      }
      /* 如果重置了一个cell，那么将它从存储列表中去除 */
      consistent_cell_info_list_.erase(current_cell.xoy);
      UpdateVertex(current_cell.xoy);
      std::cout << std::endl;
      std::vector<Points> neighbors_pos = GetNeighborsPoint(current_cell.xoy);
      for (int8_t i = 0; i < neighbors_pos.size(); ++i) {
        ++neighbor_expand_cnt;
        UpdateVertex(neighbors_pos[i]);
        std::cout << std::endl;
      }
    }
    /* 扩展点自增 */
    if (neighbor_expand_cnt) ++current_expand_points_count_;
  }
  close_list_.clear();

  /* 失败标志 */
  if (start.g_value == INF_f::max()) search_successful_flg = 1;

  /* 搜索结果判断 */
  current_path_.clear();
  if (search_successful_flg) {
    std::cout << "search fail !!" << std::endl;
    PrintSearchResult();
    return false;
  } else {
    std::cout << "search successfully !!" << std::endl;
    Points node = current_start_;

    /* 路径回溯 */
    while (current_save_path_hash_.find(node) !=
           current_save_path_hash_.end()) {
      node = current_save_path_hash_[node];
      path_result_list.push_back(node);
      std::cout << "(" << node.first << "," << node.second << ")  ";
    }
    current_path_ =
        std::vector<Points>(path_result_list.rbegin(), path_result_list.rend());

    /* 打印结果 */
    PrintSearchResult();

    /* 总扩展点计数 */
    all_expand_points_count_ += current_expand_points_count_;

    /* 扩展点计数清零 */
    current_expand_points_count_ = 0;

    ++search_nums_count_;

    return true;
  }
}

/* 增加障碍物，并更新障碍物
 * 输入：需要去除障碍物的坐标
 * 输出：更新障碍物所有临近点的信息
 *  */
void ADstar::ChangeOfInceaseObstacle(const Points& pos) {
  consistent_cell_info_list_.erase(pos);

  for (int16_t i = 0; i < open_list_.size(); ++i) {
    if (pos == open_list_[i].xoy) Remove<CellInfo>(i, &open_list_);
  }
  current_obstacle_list_.push_back(pos);
  std::vector<Points> neighbors =
      GetNeighborsPoint(current_obstacle_list_.back());
  for (int8_t i = 0; i < neighbors.size(); ++i) {
    UpdateVertex(neighbors[i]);
    std::cout << std::endl;
  }
}

/* 更新障碍物周围的点
 * 输入：障碍物
 * 输出：无
 *  */
void ADstar::ChangeCurrentPointBlocked(const std::vector<Points>& obs_list) {
  // Km_ += DisOfLastToCurrStart();

  for (int8_t i = 0; i < obs_list.size(); ++i) {
    std::cout << "  obstacle:(" << obs_list[i].first << ","
              << obs_list[i].second << ")  ------" << std::endl;
    ChangeOfInceaseObstacle(obs_list[i]);
  }

  std::cout << "***The result of obverse current start neighbors : ------------"
            << std::endl;
  PrintSearchResult();
}

/* inconslist与openlist融合
 * 输入：inconslist
 * 输出：无
 * */
void ADstar::InconsPushOpenlist() {
  open_list_.insert(open_list_.end(), incons_list_.begin(), incons_list_.end());
  incons_list_.clear();
}

/* 更新openlist
 * 输入：无
 * 输出：无
 * 说明：由于heuristicfactor改变，需要对openlist中所有的元素进行更新，
 * 这里也是影响整体运行速度的重要环节
 * */
void ADstar::UpdateOpenlisByNewFactor() {
  for (int16_t i = 0; i < open_list_.size(); ++i) {
    open_list_[i].heuristic_factor = heuristic_factor_;
    open_list_[i].h_value = DistenceToGoal(open_list_[i].xoy);
  }
}

/* 打印一次搜索的结果
 * 输入：无
 * 输出：无
 * */
void ADstar::PrintSearchResult() {
  for (int i = 0; i < row_; ++i) {
    for (int j = 0; j < column_; ++j) {
      if (current_start_.first == i && current_start_.second == j)
        std::cout << "s ";

      else if (goal_pos_.first == i && goal_pos_.second == j)
        std::cout << "g ";

      else if (IsInList(Points(i, j), current_obstacle_list_))
        std::cout << "x ";

      else if (IsInList(Points(i, j), current_path_))
        std::cout << "o ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << "shortest path step nums : " << current_path_.size()
            << "    expand point nums : " << current_expand_points_count_
            << std::endl;

  std::cout << std::endl << std::endl;
}

/* 打印计数结果
 * 输入：无
 * 输出：无
 *  */
void ADstar::PrintCountResult() {
  std::cout << std::endl
            << "The nums of search : " << search_nums_count_
            << "  total expanded nums : " << all_expand_points_count_
            << std::endl
            << std::endl;
}

/* 打印统计结果
 * 输入：无
 * 输出：无
 *  */
void PrintSumResult() {
  std::cout << std::endl
            << "-——-——-——-——-——-——-——-——-***-——-——-——-——-——-——-——-——-——-"
            << std::endl
            << "-——                   Sum  Result                    ——-"
            << std::endl
            << "-——-——-——-——-——-——-——-——-***-——-——-——-——-——-——-——-——-——-"
            << std::endl;
  std::cout << "| map num | search nums | expand nums | move_step nums |"
            << std::endl;
  for (int16_t i = 0; i < sum_result.size(); ++i) {
    std::cout << sum_result[i] << std::endl;
  }
}
