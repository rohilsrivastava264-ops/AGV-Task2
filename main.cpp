#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>

#include "draw.hpp"
#include "geometry.hpp"
#include "simulation.hpp"


int main() {
    // the planners can share data by writing to and reading from variables
    // defined here
 srand((unsigned)time(nullptr));


    agent myagent;
    myagent.calculate_1 =
        [&](const envmap& curmap,
            const array<pair<point, point>, playercount>& playerdata,
            const array<point, rays>& raycasts, const agent& curplayer,
            ftype& a, ftype& steer) {
            // write your planning logic here
            (void)curplayer;

            static bool initialized = false;
            static vector<point> waypoints;
            static int current = 0;
            static ftype prev_steer = 0;

            point pos = playerdata[0].first;
            ftype theta = playerdata[0].second.y;

            // --- Initialization: Task 1 (Arbitrary Waypoints) ---
            auto in_free_space = [&](const point& p) -> bool {
                if (curmap.empty()) return true;
                if (!contains(curmap.back(), p)) return false;
                for (size_t i = 0; i + 1 < curmap.size(); i++) {
                    if (contains(curmap[i], p)) return false;
                }
                return true;
            };

            if (!initialized) {
                for (int i = 0; i < 8; i++) {
                    point p(1.6f * (ftype)rand() / (ftype)RAND_MAX - 0.8f,
                            1.6f * (ftype)rand() / (ftype)RAND_MAX - 0.8f);
                    if (in_free_space(p)) waypoints.push_back(p);
                    else i--; // Retry if point is inside an obstacle
                }
                initialized = true;
            }

            point target = waypoints[current];
            ftype d_to_target = dist(pos, target);

            // Switch waypoint only when reached
            if (d_to_target < 0.12f) {
                current = (current + 1) % waypoints.size();
                target = waypoints[current];
            }

            // --- Task 3: Noisy LiDAR Handling (Moving Average Filter) ---
            vector<ftype> filtered_dists(rays);
            for (int i = 0; i < rays; i++) {
                ftype d_prev = dist(pos, raycasts[(i - 1 + rays) % rays]);
                ftype d_curr = dist(pos, raycasts[i]);
                ftype d_next = dist(pos, raycasts[(i + 1) % rays]);
                filtered_dists[i] = (d_prev + d_curr + d_next) / 3.0f; 
            }

            // --- Task 2: Obstacle Avoidance (Inverse-Square Potential Field) ---
            ftype target_angle = atan2(target.y - pos.y, target.x - pos.x);
            point attr_force(cos(target_angle), sin(target_angle));
            
            point repulse_force(0, 0);
            ftype min_dist = mdist;
            ftype safety_radius = 0.38f; // Increased for dynamic obstacles

            for (int i = 0; i < rays; i++) {
                ftype d = filtered_dists[i];
                if (d < min_dist) min_dist = d;

                if (d < safety_radius) {
                    point away = pos - raycasts[i];
                    // Exponential push-back as distance closes
                    ftype weight = pow((safety_radius - d) / (d + 0.01f), 2);
                    repulse_force = repulse_force + (weight * away);
                }
            }

            // Combine Goal Pull + Obstacle Push
            point total_force = attr_force + (1.6f * repulse_force);
            ftype desired_angle = atan2(total_force.y, total_force.x);

            // --- Motion Control ---
            // Speed logic
            if (min_dist < 0.10f) a = -0.0005f;      // Emergency reverse
            else if (min_dist < 0.25f) a = 0.0001f; // Slow down
            else a = 0.0006f;                       // Normal cruise

            // Steering logic with damping
            ftype angle_err = desired_angle - theta;
            while (angle_err > PI) angle_err -= 2 * PI;
            while (angle_err < -PI) angle_err += 2 * PI;

            ftype steer_raw = clip(angle_err, PI / 8);
            steer = 0.85f * prev_steer + 0.15f * steer_raw; // Smoothing filter
            prev_steer = steer;
                        
        };

    array<agent, playercount> myagents;
    for (int i = 0; i < playercount; i++) myagents[i] = myagent;

    ftype simultime = 30;

    simulationinstance s(myagents, simultime);
     s.humanmode=false;
   
      // Dynamic obstacle motion for all movable obstacles.
    const ftype T = 4.0f, A = 0.015f;
    for (int i = 0; i < (int)s.movementspecifier.size() - 1; i++) {
        const vector<point> base = s.mp[i];
        const ftype phase = 2 * PI * i / max(1, (int)s.movementspecifier.size() - 1);

        s.movementspecifier[i] =
            [base, phase, T, A](vector<point>& obstacle, const ftype curtime) {
                point shift = A * point(cos(2 * PI * curtime / T + phase),
                                        sin(2 * PI * curtime / T + phase));
                obstacle = base + shift;
            };
    }

   
    s.run();

    return 0;
}
