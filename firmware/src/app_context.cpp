#include "app_context.hpp"

namespace firmware {

// ---------------------------------------------------------------------------
// Built-in configuration
// ---------------------------------------------------------------------------
//
// Identical in structure to config/graph.json and config/navigation.json.
// scripts/validate_config.py checks the files on disk, and the C++ loader
// validates these strings at boot, so a malformed value is caught either way.

const char* builtinGraphJson() {
    return R"JSON({
  "map": { "map_id": "SMART_MARKET_MAIN", "map_version": 1,
           "frame_id": "smart_market_map", "width_m": 12.0, "height_m": 8.0 },
  "nodes": [
    { "id": 1,  "x_m": 1.5,  "y_m": 2.0, "type": "entry" },
    { "id": 2,  "x_m": 4.5,  "y_m": 2.0, "type": "aisle" },
    { "id": 3,  "x_m": 7.5,  "y_m": 2.0, "type": "aisle" },
    { "id": 4,  "x_m": 10.5, "y_m": 2.0, "type": "checkout" },
    { "id": 5,  "x_m": 1.5,  "y_m": 4.0, "type": "intersection" },
    { "id": 6,  "x_m": 4.5,  "y_m": 4.0, "type": "intersection" },
    { "id": 7,  "x_m": 7.5,  "y_m": 4.0, "type": "intersection" },
    { "id": 8,  "x_m": 10.5, "y_m": 4.0, "type": "intersection" },
    { "id": 9,  "x_m": 1.5,  "y_m": 6.0, "type": "aisle" },
    { "id": 10, "x_m": 4.5,  "y_m": 6.0, "type": "destination" },
    { "id": 11, "x_m": 7.5,  "y_m": 6.0, "type": "destination" },
    { "id": 12, "x_m": 10.5, "y_m": 6.0, "type": "parking" }
  ],
  "edges": [
    { "from": 1,  "to": 2  }, { "from": 2,  "to": 3  }, { "from": 3,  "to": 4  },
    { "from": 5,  "to": 6  }, { "from": 6,  "to": 7  }, { "from": 7,  "to": 8  },
    { "from": 9,  "to": 10 }, { "from": 10, "to": 11 }, { "from": 11, "to": 12 },
    { "from": 1,  "to": 5  }, { "from": 2,  "to": 6  }, { "from": 3,  "to": 7  },
    { "from": 4,  "to": 8  }, { "from": 5,  "to": 9  }, { "from": 6,  "to": 10 },
    { "from": 7,  "to": 11 }, { "from": 8,  "to": 12 }
  ]
})JSON";
}

const char* builtinNavigationJson() {
    return R"JSON({
  "map": { "width_m": 12.0, "height_m": 8.0 },
  "position": {
    "minimum_quality": 0.60,
    "timeout_ms": 750,
    "heading_min_displacement_m": 0.50,
    "heading_correction_gain": 0.35,
    "heading_quality_baseline_boost": 2.0,
    "stale_sample_ms": 1000,
    "map_margin_m": 1.0
  },
  "path": {
    "max_graph_snap_distance_m": 1.5,
    "replan_max_graph_snap_distance_m": 2.5,
    "waypoint_tolerance_m": 0.20,
    "destination_tolerance_m": 0.15,
    "max_cross_track_error_m": 0.40,
    "replan_cross_track_error_m": 0.60,
    "route_deviation_confirm_ms": 500,
    "replan_cooldown_ms": 1000,
    "corner_slowdown_distance_m": 0.80
  },
  "motion": {
    "max_linear_speed_mps": 0.45,
    "min_linear_speed_mps": 0.08,
    "max_angular_speed_radps": 2.2,
    "rotate_in_place_threshold_deg": 55.0,
    "heading_error_slow_deg": 25.0,
    "heading_kp": 1.6,
    "max_linear_accel_mps2": 0.35,
    "max_angular_accel_radps2": 2.5,
    "heading_deadband_deg": 1.5,
    "max_motor_command_change_per_s": 6.0
  },
  "drive_kind": "stepper",
  "stepper": {
    "common": {
      "steps_per_revolution": 200,
      "microsteps": 16,
      "max_step_rate_hz": 20000,
      "min_step_rate_hz": 2,
      "max_step_accel_hz_per_s": 20000
    },
    "left": { "invert_direction": false },
    "right": { "invert_direction": false },
    "enable_odometry": true
  },
  "robot": {
    "wheel_radius_m": 0.05,
    "wheel_base_m": 0.32,
    "max_wheel_speed_mps": 0.70,
    "motor_deadband_mps": 0.005,
    "left_gain": 1.0,
    "right_gain": 1.0
  },
  "control": { "navigation_rate_hz": 20.0, "telemetry_rate_hz": 5.0 }
})JSON";
}

FirmwareContext& context() {
    static FirmwareContext instance;
    return instance;
}

}  // namespace firmware
