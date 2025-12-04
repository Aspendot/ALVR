#pragma once

#include <atomic>
#include <utility>

namespace alvr {

void set_gaze_sample(float x, float y);
std::pair<float, float> get_gaze_sample(float fallback_x, float fallback_y);

} // namespace alvr
