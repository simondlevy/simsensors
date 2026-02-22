/* 
   Datatypes for simulation

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

namespace simsens {

    class Vec2 {
        public:
            double x, y;
            Vec2() = default;
            Vec2(const double x, const double y) : x(x), y(y) {}
            Vec2(const Vec2 & other) : x(other.x), y(other.y) { }
    };

    class Vec3 {
        public:
            double x, y, z;
            Vec3() = default;
            Vec3(const double x, const double y, const double z) : x(x), y(y), z(z) {}
            Vec3(const Vec3 & other) : x(other.x), y(other.y), z(other.z) { }
    };

    class Pose {
        public:
            double x, y, z, phi, theta, psi;
            Pose() = default;
            Pose(const Pose & other)
                : x(other.x), y(other.y), z(other.z),
                  phi(other.phi), theta(other.theta), psi(other.psi) {}
    };

    class Rotation {
        public:
            double x, y, z, alpha;
            Rotation() = default;
            Rotation(const Rotation & other)
                : x(other.x), y(other.y), z(other.z), alpha(other.alpha) {}
    };

    typedef struct {
        double x;
        double y;
        double z;
        double phi;
        double theta;
        double psi;
    } pose_t;

    // https://www.cyberbotics.com/doc/reference/transform?version=cyberbotics:R2019a-rev1
    typedef struct {
        double x;
        double y;
        double z;
        double alpha;
    } rotation_t;

};
