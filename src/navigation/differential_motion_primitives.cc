#include "differential_motion_primitives.h"

#include <limits>

#include "config_reader/config_reader.h"

CONFIG_FLOAT(max_curvature, "DifferentialSampler.max_curvature");
CONFIG_FLOAT(clearance_clip, "DifferentialSampler.clearance_path_clip_fraction");
CONFIG_FLOAT(max_fov, "DifferentialSampler.max_fov");

namespace motion_primitives {

std::vector<std::shared_ptr<ConstantCurvatureArc>> DifferentialSampler::getSamples(
    int n) {
  auto samples = AckermannSampler::getSamples(n);

  const float bearing = atan2(local_target_.y(), local_target_.x());
  // Curvature a single arc would need to face the target (pure-pursuit style).
  // Offer a turn in place when that exceeds the sampler's curvature limit, or
  // when the target sits in the rear half-plane where forward arcs can't reach.
  const float dist = fmax(local_target_.norm(), 1e-3f);
  const float required_curvature = 2.0f * fabs(sin(bearing)) / dist;
  const bool needs_turn = fabs(bearing) > static_cast<float>(M_PI_2) ||
                          required_curvature > CONFIG_max_curvature;

  const float turn_angle = fmin(fabs(bearing), CONFIG_max_fov / 2.0f);
  if (needs_turn && turn_angle > 1e-3f) {
    const float kInf = std::numeric_limits<float>::infinity();
    for (const float curvature : {kInf, -kInf}) {
      auto sample = std::make_shared<ConstantCurvatureArc>(
          curvature, turn_angle, nav_params_.max_clearance);
      checkObstacles(sample);
      samples.push_back(sample);
    }
  }

  return samples;
}

void DifferentialSampler::checkObstacles(
    std::shared_ptr<ConstantCurvatureArc> path_ptr) {
  if (!path_ptr->is_turn_in_place()) {
    AckermannSampler::checkObstacles(path_ptr);
    return;
  }

  const float l = nav_params_.robot_length + 2 * nav_params_.obstacle_margin;
  const float w = nav_params_.robot_width + 2 * nav_params_.obstacle_margin;
  const float l_f = l - (l - nav_params_.robot_wheelbase) / 2;  // base to front
  const float l_r = l - l_f;                                    // base to rear

  // Circumscribed radius of the footprint; while turning in place the footprint
  // sweeps the disk of this radius about base_link.
  const float r_max = sqrt(Sq(max(l_f, l_r)) + Sq(w / 2.0f));

  for (const auto& point : point_cloud_) {
    const float r_p = point.norm();
    if (r_p >= r_max) {
      // Point stays outside the swept disk; it never collides during rotation.
      path_ptr->set_clearance(fmin(r_p - r_max, path_ptr->clearance()));
      continue;
    }

    // Point lies within the swept disk: rotating in place would strike it.
    path_ptr->set_arc_length(0);
    path_ptr->set_clearance(0);
    break;
  }

  // Dead-band: collapse a sliver of clearance to a hard collision.
  if (path_ptr->clearance() < CONFIG_clearance_clip * nav_params_.max_clearance) {
    path_ptr->set_clearance(0);
  }
}

}  // namespace motion_primitives
