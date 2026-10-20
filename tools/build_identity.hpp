#pragma once

#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#ifndef SP_SOURCE_REVISION
#define SP_SOURCE_REVISION "unrecorded"
#endif
#ifndef SP_SOURCE_SHA256
#define SP_SOURCE_SHA256 "unrecorded"
#endif
#ifndef SP_CONFIGURED_STANDARD_SHA256
#define SP_CONFIGURED_STANDARD_SHA256 "unrecorded"
#endif

namespace tools {
// A deterministic runtime identity, not a cryptographic authenticity check.
// The compiled source identity is SHA256; this also identifies generated YAML.
inline std::string file_fnv1a64(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) return "unavailable";
  std::uint64_t hash = 14695981039346656037ULL;
  std::array<char, 8192> bytes{};
  while (input) {
    input.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    for (std::streamsize i = 0; i < input.gcount(); ++i) {
      hash ^= static_cast<unsigned char>(bytes[static_cast<std::size_t>(i)]);
      hash *= 1099511628211ULL;
    }
  }
  if (!input.eof()) return "read_error";
  std::ostringstream value;
  value << std::hex << std::setfill('0') << std::setw(16) << hash;
  return value.str();
}
}  // namespace tools
