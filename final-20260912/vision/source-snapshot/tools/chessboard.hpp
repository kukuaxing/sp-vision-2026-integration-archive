#pragma once

#include <fmt/core.h>

#include <opencv2/opencv.hpp>
#include <vector>

namespace tools {

// 对称棋盘的角点编号存在 180° 歧义：findChessboardCorners 返回的角点顺序可能整体
// 旋转半圈，此时 solvePnP/IPPE 会给出"绕板法线翻转 180°"的位姿（R_board2cam 相对
// 真值旋转 180°）。这种帧混进手眼标定会把整条解拖偏（本项目曾出现光轴被拖成竖直
// 的 ~90° 错误）。标定采集时云台在相邻帧之间只动几度，因此相邻两帧的 R_board2cam
// 只能差几度；一旦相差超过 90°，即可判定后一帧角点顺序被翻转。
//
// 第一遍从原始数据推出每帧相对第一帧的翻转奇偶性（仅在相邻帧相差 >90° 时翻转符号），
// 第二遍把被翻转的帧绕板 z 轴补转 180°（右乘 Rz180）。
// 注意：tvec 不受翻转影响（solvePnP 方程 R·(Rz·P)+t = R_true·P+t_true 给出 t 不变），
// 因此只改 rvec。
inline void fix_flipped_boards(std::vector<cv::Mat> & rvecs)
{
  if (rvecs.size() < 2) return;

  const cv::Mat Rz180 = (cv::Mat_<double>(3, 3) << -1, 0, 0, 0, -1, 0, 0, 0, 1);
  constexpr double kFlipThresholdRad = CV_PI / 2.0;

  // 第一遍：奇偶性（用原始数据，绝不在这一步改 rvecs）
  std::vector<int> parity(rvecs.size(), 1);
  for (size_t i = 0; i + 1 < rvecs.size(); i++) {
    cv::Mat R_i, R_next;
    cv::Rodrigues(rvecs[i], R_i);
    cv::Rodrigues(rvecs[i + 1], R_next);
    cv::Mat dR = R_next * R_i.t();
    cv::Mat rv_d;
    cv::Rodrigues(dR, rv_d);
    if (cv::norm(rv_d) > kFlipThresholdRad) {
      parity[i + 1] = -parity[i];
    } else {
      parity[i + 1] = parity[i];
    }
  }

  // 第二遍：校正被翻转的帧
  int corrected = 0;
  for (size_t i = 0; i < rvecs.size(); i++) {
    if (parity[i] < 0) {
      cv::Mat R_i;
      cv::Rodrigues(rvecs[i], R_i);
      cv::Rodrigues(R_i * Rz180, rvecs[i]);  // 绕板 z 轴补转 180°
      corrected++;
    }
  }
  if (corrected > 0) {
    fmt::print(
      "[chessboard] 检测到 {} 帧棋盘位姿 180° 翻转并已自动校正（对称棋盘角点编号歧义："
      "相邻帧棋盘位姿相差超过 90°）。若不需要校正，请检查采集时相邻帧云台转角是否过大。\n",
      corrected);
  }
}

}  // namespace tools
