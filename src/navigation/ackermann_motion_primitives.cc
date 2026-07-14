#include "ackermann_motion_primitives.h"

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace motion_primitives {

std::vector<std::shared_ptr<PathRolloutBase>> AckermannSampler::GetSamples(int n) {
  std::vector<std::shared_ptr<PathRolloutBase>> samples;
  const float linear_speed = vel.norm();
  // Whatever curvature we can achieve while accelerating we can achieve while
  // decelerating, so only consider acceleration
  const float max_dtheta_dot =  // Find angular velocity limit
      nav_params.linear_limits.max_acceleration * nav_params.max_curvature *
      nav_params.dt;
  const float max_ds_dot = nav_params.linear_limits.max_acceleration *
                           nav_params.dt;  // Find maximum velocity

  float cmax = nav_params.max_curvature;
  float cmin = -nav_params.max_curvature;
  // Curvature limits in params only apply when we have 0 velocity
  if (linear_speed > max_ds_dot) {
    cmin = max(cmin, (ang_vel - max_dtheta_dot) / (linear_speed + max_ds_dot));
    cmax = min(cmax, (ang_vel + max_dtheta_dot) / (linear_speed + max_ds_dot));  //
  }
  const float dc = (cmax - cmin) / (n - 1);

  // 1 Take dynamics into for min and max curvature if necessary
  for (float c = cmin; c <= cmax; c += dc) {
    // 2 Compute the arc length
    auto sample = std::make_shared<ConstantCurvatureArc>(
        c, nav_params.max_path_length, nav_params.max_clearance);
    setPathLength(sample);
    checkObstacles(sample);

    samples.push_back(sample);
  }

  if (FLAGS_v > 1) {
    cout << "==================== [Navigation] Sampler ====================" << endl;
    cout << "Number of samples: " << samples.size() << endl;
    cout << "cmax: " << cmax << ", cmin: " << cmin << ", dc: " << dc << endl;
    cout << "Linear velocity: " << linear_speed << endl;
    cout << "Angular velocity: " << ang_vel << endl;
    cout << "range Angular velocity: " << ang_vel - max_dtheta_dot << " to "
         << ang_vel + max_dtheta_dot << endl;
    cout << "Local target: " << local_target.transpose() << endl;
    cout << "==============================================================\n" << endl;
  }

  return samples;
}

void AckermannSampler::setPathLength(std::shared_ptr<ConstantCurvatureArc> path_ptr) {
  // Linear motion
  if (fabs(path_ptr->curvature()) < 1e-5) {
    path_ptr->set_arc_length(fmin(local_target.x(), nav_params.max_path_length));
    return;
  }

  const float radius = 1 / path_ptr->curvature();
  Eigen::Vector2f instant_center(0, radius);
  Eigen::Vector2f instant_center_to_goal =
      fabs(radius) * (local_target - instant_center).normalized();
  const float theta =
      atan2(fabs(instant_center_to_goal.x()), fabs(instant_center_to_goal.y()));
  const float arc_length = fabs(radius * theta);

  path_ptr->set_arc_length(fmin(arc_length, nav_params.max_path_length));
}

void AckermannSampler::checkObstacles(std::shared_ptr<ConstantCurvatureArc> path_ptr) {
  static const bool kDebug = false;

  const float l = nav_params.robot_length + 2 * nav_params.obstacle_margin;
  const float w = nav_params.robot_width + 2 * nav_params.obstacle_margin;
  const float l_f = l - (l - nav_params.robot_wheelbase) / 2;  // base to front
  const float l_r = l - l_f;                                   // base to rear

  // Add special case to handle when car is driving nearly straight
  if (fabs(path_ptr->curvature()) < 1e-5) {
    // Only check if points are within the width of the car
    const float min_y = -w / 2.0f;
    const float max_y = w / 2.0f;

    for (const auto& point : point_cloud) {
      // Point is outside the lateral swept volume of the car
      if (point.y() < min_y || point.y() > max_y) {
        const float clearance = fmin(fabs(point.y() - min_y), fabs(point.y() - max_y));
        path_ptr->set_clearance(fmin(clearance, path_ptr->clearance()));
        continue;
      }

      // Point is outside the forward swept volume of the car
      if (point.x() > path_ptr->arc_length() + l_f || point.x() < -l_r) {
        continue;
      }

      // Point in inside the swept volume of the car
      float arc_length = fmax(point.x() - l_f, 0);
      path_ptr->set_arc_length(fmin(arc_length, path_ptr->arc_length()));
      path_ptr->set_clearance(0);
      break;
    }
    return;
  }

  // compute volume swept by the car during a single time (depends on curvature)
  const float r = 1 / path_ptr->curvature();
  const float r_base_min = fabs(r) - (w / 2.0f);
  const float r_base_max = fabs(r) + (w / 2.0f);
  const float r_front_min = sqrt(Sq(r_base_min) + Sq(l_f));
  const float r_front_max = sqrt(Sq(r_base_max) + Sq(l_f));
  const float r_rear_max = sqrt(Sq(r_base_max) + Sq(l_r));

  if (kDebug) {
    cout << "Checking obstacles for curvature " << path_ptr->curvature() << endl;
  }

  const Eigen::Vector2f instant_center(0, r);
  for (const auto& point : point_cloud) {
    if (point.x() < -l_r) {
      // Case 1: Point is behind the car
      continue;
    }

    const float r_p = (point - instant_center).norm();
    if (r_p < r_base_min || r_p > r_front_max) {
      // Case 2: Point is out of sweep volume
      const float clearance = fmin(fabs(r_p - r_base_min), fabs(r_p - r_front_max));
      path_ptr->set_clearance(fmin(clearance, path_ptr->clearance()));
      continue;
    }

    int phase = 0;
    Eigen::Vector2f point_to_collision(0, 0);
    if (r_p < r_front_min) {
      // Case 3: Point is in swept volume in inside of car
      point_to_collision.y() = Sign(r) * (w / 2.0f);
      point_to_collision.x() = sqrt(Sq(r_p) - Sq(point_to_collision.y() - r));
      if (kDebug) {
        cout << point.transpose() << " is inner area of weep volume" << endl;
        phase = 3;
      }
    } else if (r_p > r_base_max && r_p < r_rear_max && fabs(point.x()) < l_r) {
      // Case 4: Point is in sweapt volum e between base_link and rear bumper of car
      point_to_collision.y() = -Sign(r) * (w / 2.0f);
      point_to_collision.x() = sqrt(Sq(r_p) - Sq(point_to_collision.y() - r));
      if (kDebug) {
        cout << point.transpose() << " is in rear area of sweep volume" << endl;
        phase = 4;
      }
    } else if (r_p < r_front_max && point.x() > l_f) {
      // Case 5: Point is in swept volume in front of car
      point_to_collision.x() = l_f;
      point_to_collision.y() = r - Sign(r) * sqrt(Sq(r_p) - Sq(point_to_collision.x()));
      if (kDebug) {
        cout << point.transpose() << " is in front area of sweep volume" << endl;
        phase = 5;
      }
    } else {
      // Case 6: Point is out of swept volume
      const float clearance = fabs(r_rear_max - r_p);
      path_ptr->set_clearance(fmin(clearance, path_ptr->clearance()));
      if (kDebug) {
        cout << point.transpose() << " is out of sweep volume" << endl;
        phase = 6;
      }
      continue;
    }

    // Compute the arc length until the collision
    const Eigen::Vector2f collision_radial = point_to_collision - instant_center;
    const Eigen::Vector2f point_radial = point - instant_center;
    const float swap_theta = acos(point_radial.dot(collision_radial) /
                                  (point_radial.norm() * collision_radial.norm()));
    const float arc_length = fabs(r) * swap_theta;

    if (kDebug) {
      cout << "Curvature: " << path_ptr->curvature()
           << ", Arc length to collision: " << arc_length << endl;
      cout << "Point to collision: " << point_to_collision.transpose()
           << ", r_p: " << r_p << ", r: " << r << ", swap_theta: " << swap_theta
           << ", phase: " << phase << endl;
      cout << phase << endl;
    }

    // Penalize clearance as well if we have a collision before arc length is traversed
    if (arc_length < path_ptr->arc_length()) {
      path_ptr->set_arc_length(arc_length);
      path_ptr->set_clearance(0);
    }
  }
}

}  // namespace motion_primitives

namespace {
float NormalizeMinMax(const float x, const float lo, const float hi) {
  const float denom = hi - lo;
  if (denom < 1e-6f) return 0.0f;
  return (x - lo) / denom;
}
}  // namespace

namespace motion_primitives {
PathMetrics AckermannEvaluator::computeMetrics(
    const std::shared_ptr<PathRolloutBase>& path) {
  PathMetrics m;
  m.clearance = path->Clearance();

  const pose_2d::Pose2Df endpoint = path->EndPoint();
  // Goal-follow cost: L2 distance from the path endpoint to the local subgoal
  // (the intermediate-plan carrot handed to the evaluator as local_target).
  m.goal_dist = (endpoint.translation - local_target).norm();

  // Heading alignment: residual angle between the endpoint heading and the
  // bearing from the endpoint to the local subgoal (0 = pointed straight at it).
  // This pulls the robot to rotate toward off-axis targets even when no forward
  // arc can actually reach them this tick.
  const Eigen::Vector2f to_target = local_target - endpoint.translation;
  const float target_bearing = (to_target.squaredNorm() < 1e-8f)
                                   ? endpoint.angle
                                   : atan2(to_target.y(), to_target.x());
  m.heading = math_util::AngleDist(target_bearing, endpoint.angle);

  Eigen::Vector2f vel_cmd(0.0f, 0.0f);
  float ang_vel_cmd = 0.0f;
  path->GetControls(nav_params_.linear_limits, nav_params_.angular_limits,
                    static_cast<float>(nav_params_.dt), vel, ang_vel, vel_cmd,
                    ang_vel_cmd);
  m.velocity = vel_cmd.x();
  return m;
}

std::shared_ptr<PathRolloutBase> AckermannEvaluator::FindBest(
    const std::vector<std::shared_ptr<PathRolloutBase>>& samples) {
  if (samples.empty()) return nullptr;

  // Pass 1: raw metrics + per-term range for the min-max-normalized terms.
  // Clearance and velocity have no natural absolute scale, so they stay relative min max
  std::vector<PathMetrics> metrics;
  metrics.reserve(samples.size());
  float clr_lo = std::numeric_limits<float>::infinity();
  float clr_hi = -std::numeric_limits<float>::infinity();
  float vel_lo = clr_lo, vel_hi = clr_hi;
  for (const auto& path : samples) {
    const PathMetrics m = computeMetrics(path);
    clr_lo = min(clr_lo, m.clearance); clr_hi = max(clr_hi, m.clearance);
    vel_lo = min(vel_lo, m.velocity); vel_hi = max(vel_hi, m.velocity);
    metrics.push_back(m);
  }

  // Fixed scales for the absolute terms: goal distance is measured against the
  // arc horizon (max_path_length), heading against a half-turn (pi).
  const float goal_scale = max(nav_params_.max_path_length, 1e-3f);
  const float kPi = static_cast<float>(M_PI);

  // Pass 2: weighted score (higher is better): close to the local target, heading
  // aligned with it, high clearance, and high velocity.
  std::shared_ptr<PathRolloutBase> best_path = nullptr;
  float best_score = -std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < samples.size(); ++i) {
    const float goal_term = 1.0f - min(metrics[i].goal_dist / goal_scale, 1.0f);
    const float heading_term = 1.0f - min(metrics[i].heading / kPi, 1.0f);
    const float score =
        nav_params_.distance_weight * goal_term +
        nav_params_.heading_weight * heading_term +
        nav_params_.clearance_weight *
            NormalizeMinMax(metrics[i].clearance, clr_lo, clr_hi) +
        nav_params_.velocity_weight *
            NormalizeMinMax(metrics[i].velocity, vel_lo, vel_hi);

    if (best_path == nullptr || score > best_score) {
      best_path = samples[i];
      best_score = score;
    }

    if (FLAGS_v > 1) {
      cout << "Length: " << samples[i]->Length()
           << ", goal_dist: " << metrics[i].goal_dist
           << ", heading: " << metrics[i].heading
           << ", clearance: " << metrics[i].clearance
           << ", velocity: " << metrics[i].velocity << ", Score: " << score << endl;
    }
  }

  if (FLAGS_v > 1) {
    cout << "================= [Navigation Evaluator] Best =================" << endl;
    cout << "Length: " << best_path->Length()
         << ", Clearance: " << best_path->Clearance() << ", Score: " << best_score
         << endl;
    cout << "==============================================================\n" << endl;
  }

  return best_path;
}

}  // namespace motion_primitives
