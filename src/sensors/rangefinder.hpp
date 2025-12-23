/* 
   Rangefinder simulator

   Copyright (C) 2025 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <sim_math.hpp>

namespace simsens {

    class SimRangefinder {

        friend class RangefinderVisualizer;
        friend class RobotParser;

        public:

        void read(const pose_t & robot_pose, const vector<Wall *> walls,
                int * distances_mm, vec3_t & dbg_intersection)
        {
            // Get rangefinder rotation w.r.t. vehicle
            vec3_t rangefinder_angles= {};
            rotation_to_euler(rotation, rangefinder_angles);

            // Use vehicle angles and rangefinder angle to get rangefinder
            // azimuth and elevation angles
            const auto azi = robot_pose.psi + rangefinder_angles.z;
            const auto ele = robot_pose.theta + rangefinder_angles.y;

            // Beam starts at robot coordinates
            const simsens::vec2_t beam_start = {robot_pose.x, robot_pose.y};

            // Calculate beam endpoint
            const simsens::vec2_t beam_end = {
                beam_start.x + cos(azi) * max_distance_m,
                beam_start.y - sin(azi) * max_distance_m,
            };

            // Run a classic calculate-min loop to get distance to closest wall
            double dist = INFINITY;
            vec3_t intersection = {};
            for (auto wall : walls) {
                intersect_with_wall( beam_start, beam_end, robot_pose.z, ele,
                        *wall, dist, intersection);
            }

            // Cut off distance at rangefinder's maximum
            if (dist > max_distance_m) {
                dist = INFINITY;
                intersection.z = -1;
            }

            dist = sqrt(dist);

            // Subtract sensor offset from distance
            dist -= sqrt(
                    sqr(this->translation.x) +
                    sqr(this->translation.y) +
                    sqr(this->translation.z));

            //printf("dist=%3.3f\n", dist);
            printf("phi=%+3.3f the=%+3.3f psi=%+3.3f ele=%+3.3f | dist=%3.3f\n",
                    robot_pose.phi, robot_pose.theta, robot_pose.psi, ele, dist);

            // Use just one distance for now
            distances_mm[0] = dist * 1000; // m => mm

            memcpy(&dbg_intersection, &intersection, sizeof(vec3_t));
        }

        void dump()
        {
            printf("Rangefinder: \n");
            printf("  fov: %3.3fr\n", field_of_view_radians);
            printf("  width: %d\n", width);
            printf("  height: %d\n", height);
            printf("  min range: %3.3fm\n", min_distance_m);
            printf("  max range: %3.3fm\n", max_distance_m);
            printf("  translation: x=%+3.3fm y=%+3.3fm z=%+3.3fm\n",
                    translation.x, translation.y, translation.z);
            printf("  rotation: x=%+3.3f y=%+3.3f z=%+3.3f alpha=%+3.3fr\n",
                    rotation.x, rotation.y, rotation.z, rotation.alpha);
            printf("\n");
        }

        private:

        static constexpr double MAX_WORLD_SIZE_M = 500; // arbitrary

        int width;
        int height; 
        double min_distance_m;
        double max_distance_m;
        double field_of_view_radians;
        vec3_t translation;
        rotation_t rotation;

        static void intersect_with_wall(
                const vec2_t beam_start_xy,
                const vec2_t beam_end_xy,
                const double robot_z,
                const double elevation,
                const Wall & wall,
                double & dist,
                vec3_t & intersection)
        {
            // Get wall endpoints
            const auto psi = wall.rotation.alpha; // rot.  always 0 0 1 alpha
            const auto len = wall.size.y / 2;
            const auto dx = len * sin(psi);
            const auto dy = len * cos(psi);
            const auto tx = wall.translation.x;
            const auto ty = wall.translation.y;

            // If beam ((x1,y1),(x2,y2)) intersects with with wall
            // ((x3,y3),(x4,y4)) 
            double px=0, py=0;
            if (line_segments_intersect(
                        beam_start_xy.x, beam_start_xy.y,
                        beam_end_xy.x, beam_end_xy.y,
                        tx + dx, ty + dy,
                        tx - dx, ty - dy,
                        px, py)) {

                // Use intersection (px,py) to calculate XY distance to wall
                const auto xydist =
                    sqr(beam_start_xy.x - px) + sqr(beam_start_xy.y - py);

                // Use XY distance and robot Z to calculate the elevation Z on
                // wall
                (void)robot_z;
                (void)elevation;

                // Calculate XYZ distance by including elevation
                const auto xyzdist = xydist;


                // If XYZ distance is shorter than current, update current
                if (xyzdist < dist) {
                    intersection.x = px;
                    intersection.y = py;
                    intersection.z = robot_z;
                    dist = xyzdist;
                }
            }
        }

        // https://gist.github.com/kylemcdonald/6132fc1c29fd3767691442ba4bc84018
        static bool line_segments_intersect(
                const double x1, const double y1,
                const double x2, const double y2,
                const double x3, const double y3,
                const double x4, const double y4,
                double & px, double & py)
        {
            const auto denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

            if (denom != 0) {

                const auto ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
                const auto ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

                // Check if intersection point lies within both line segments (0 <=
                // ua <= 1 and 0 <= ub <= 1)
                if (0 <= ua && ua <= 1 && 0 <= ub && ub <= 1) {

                    px = x1 + ua * (x2 - x1);
                    py = y1 + ua * (y2 - y1);

                    return true;
                }
            }

            return false;
        }


        static bool ge(const double a, const double b)
        {
            return iszero(a-b) || a > b;
        }

        static bool le(const double a, const double b)
        {
            return iszero(a-b) || b > a;
        }

        static bool iszero(const double x)
        {
            return fabs(x) < 0.001; // mm precision
        }

        static double sqr(const double x)
        {
            return x * x;
        }
    };
}
