#include "GazeStore.h"

namespace alvr {
namespace {
std::atomic<float> g_x {0.5f};
std::atomic<float> g_y {0.5f};
}

void set_gaze_sample(float x, float y) {
    g_x.store(x);
    g_y.store(y);
}

std::pair<float, float> get_gaze_sample(float fallback_x, float fallback_y) {
    float x = g_x.load();
    float y = g_y.load();
    if (!(x >= 0.0f && x <= 1.0f) || !(y >= 0.0f && y <= 1.0f)) {
        return {fallback_x, fallback_y};
    }
    return {x, y};
}

} // namespace alvr
