#pragma once

#include <filesystem>
#include <string_view>

namespace adas_scenario_loader
{

struct Scenario;
struct ValidationResult;

Scenario load_from_file(const std::filesystem::path & path);
Scenario load_from_string(std::string_view json_text);
ValidationResult validate(const Scenario & scenario);

}  // namespace adas_scenario_loader
