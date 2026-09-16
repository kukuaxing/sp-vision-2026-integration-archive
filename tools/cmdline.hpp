#pragma once

#include <string>
#include <vector>

namespace tools {

// OpenCV 的 cv::CommandLineParser 只认 "-opt=value"，不认 "-opt value"：
// 空格写法会被当成布尔开关，选项值被置为字符串 "true"，紧跟的路径则变成位置参数，
// 造成 YAML::LoadFile("true") 之类的错误（项目根目录若恰有名为 true 的目录还会崩）。
// 在构造 CommandLineParser 前先调用 normalize_argv，把空格分隔的选项值规整成 '=' 形式，
// 使文档里写的 "-c xxx" / "-o xxx" 原样可用。
// 例: "-c configs/calibration.yaml assets/img_with_q"
//   -> "-c=configs/calibration.yaml assets/img_with_q"
struct ParsedArgv {
  std::vector<std::string> args;    // 规整后的参数
  std::vector<const char *> cstrs;  // 指向 args 内字符串（args 填完后才构建）

  int argc() const { return static_cast<int>(cstrs.size()); }
  const char * const * argv() const { return cstrs.data(); }
};

inline ParsedArgv normalize_argv(int argc, char * argv[])
{
  ParsedArgv out;
  for (int i = 0; i < argc; i++) {
    std::string arg = argv[i];
    // 是选项（以'-'开头、尚未带 '='），且下一个参数不是另一个选项 → 合并成 "-opt=value"
    bool is_opt = arg.size() > 1 && arg[0] == '-' && arg.find('=') == std::string::npos;
    if (is_opt && i + 1 < argc && argv[i + 1][0] != '-') {
      arg += '=';
      arg += argv[++i];
    }
    out.args.push_back(arg);
  }
  out.cstrs.reserve(out.args.size());
  for (const auto & s : out.args) {
    out.cstrs.push_back(s.c_str());
  }
  return out;
}

}  // namespace tools
