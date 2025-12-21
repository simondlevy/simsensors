/* 
   Math for simulation

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

#include <math.h>

namespace simsens {

    void rotvec2euler(const vec4_t & vec, vec3_t & angles)
    {
        const auto w = vec.w;
        const auto x = vec.x;
        const auto y = vec.y;
        const auto z = vec.z;

        angles.x = atan2(2*(w*z+x*y), 1 - 2*(y*y+z*z));
        angles.y = asin(2*(w*y-z*x));
        angles.z = atan2(2*(w*x+y*z), 1 - 2*(x*x+y*y));
    }
};
