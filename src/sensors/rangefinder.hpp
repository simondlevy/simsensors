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

#include <vector>
using namespace std;

#include <simsensors/src/math.hpp>

namespace simsens {

    class Rangefinder {

        public:

            int width;
            int height; 
            double min_distance_m;
            double max_distance_m;

            void read(const Pose & robot_pose, World & world,
                    int * distances_mm)
            {
                const auto robpose = world.adjust_pose(robot_pose);

                for (int k=0; k<this->width; ++k) {

                    // Get rangefinder rotation w.r.t. vehicle
                    Vec3 rangefinder_angles= {};
                    rotation_to_euler(rotation, rangefinder_angles);

                    const double azimuth =
                        robpose.psi + rangefinder_angles.z + 
                        (width == 1 ? 0 :
                         (k / (width - 1.) - 0.5) * field_of_view_radians);

                    const double elevation = robpose.theta + rangefinder_angles.y; 

                    const Vec3 location =
                        Vec3{robpose.x, robpose.y, robpose.z};

                    // Run a classic calculate-min loop to get distance to closest wall
                    double dist = INFINITY;
                    Vec3 intersection = {};
                    for (auto wall : world.walls) {
                        const auto newdist = intersect_with_wall(
                                location, azimuth, elevation, wall,
                                &intersection);

                        dist = min(dist, newdist);
                    }

                    // Cut off distance at rangefinder's maximum
                    if (dist > max_distance_m) {
                        dist = INFINITY;
                    }

                    // Subtract sensor offset from distance
                    dist -= sqrt(
                            sqr(this->translation.x) +
                            sqr(this->translation.y) +
                            sqr(this->translation.z));

                    distances_mm[k] = dist == INFINITY ? -1 : dist * 1000;
                }
            }

            void dump()
            {
                printf("Rangefinder: \n");
                printf("  name: %s\n", name);
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

            double field_of_view_radians;
            Vec3 translation;
            Rotation rotation;
            char name[100];

            friend class RangefinderVisualizer;
            friend class RobotParser;
    };
}
