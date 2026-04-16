#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <string>

namespace rm_auto_aim
{
namespace
{

std::filesystem::path sourceRoot()
{
  return std::filesystem::path(AUTO_AIM_TARGETING_SOURCE_DIR);
}

TEST(IncludeLayoutTest, TopLevelIncludeDirectoryContainsExpectedEntries)
{
  const auto include_root = sourceRoot() / "include" / "auto_aim_targeting";
  const std::set<std::string> expected_dirs{
    "io", "measurement", "planning", "tracking"};
  const std::set<std::string> expected_files{
    "config.hpp", "planning_types.hpp", "auto_aim_node.hpp", "types.hpp"};

  std::set<std::string> actual_dirs;
  std::set<std::string> actual_files;
  for (const auto & entry : std::filesystem::directory_iterator(include_root)) {
    if (entry.is_directory()) {
      actual_dirs.insert(entry.path().filename().string());
    } else {
      actual_files.insert(entry.path().filename().string());
    }
  }

  EXPECT_EQ(actual_dirs, expected_dirs);
  EXPECT_EQ(actual_files, expected_files);
}

TEST(IncludeLayoutTest, FlatIncludesAreOnlyAllowedRootHeaders)
{
  const auto package_root = sourceRoot();
  // Match any flat include: auto_aim_targeting/<file>.hpp (no subdirectory)
  const std::regex flat_include_pattern(R"delim(#include\s+"auto_aim_targeting/([^/"]+\.hpp)")delim");
  // These root-level headers are explicitly allowed as flat includes
  const std::set<std::string> allowed_flat{
    "config.hpp", "planning_types.hpp", "auto_aim_node.hpp", "types.hpp"};

  for (const auto & entry : std::filesystem::recursive_directory_iterator(package_root)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    const auto extension = entry.path().extension().string();
    if (extension != ".hpp" && extension != ".cpp") {
      continue;
    }

    std::ifstream input(entry.path());
    ASSERT_TRUE(input.is_open()) << "Failed to open " << entry.path().string();
    const std::string content{
      std::istreambuf_iterator<char>{input},
      std::istreambuf_iterator<char>{}};

    std::sregex_iterator it(content.begin(), content.end(), flat_include_pattern);
    std::sregex_iterator end;
    for (; it != end; ++it) {
      const std::string header = (*it)[1].str();
      EXPECT_TRUE(allowed_flat.count(header) > 0)
        << "Unexpected flat include \"auto_aim_targeting/" << header
        << "\" in " << entry.path().string();
    }
  }
}

}  // namespace
}  // namespace rm_auto_aim
