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

    void rotvec2euler(
            const double w, const double x, const double y, const double z,
            double & phi, double & theta, double & psi)
    {
        phi = atan2(2*(w*z+x*y), 1 - 2*(y*y+z*z));
        theta = asin(2*(w*y-z*x));
        psi = atan2(2*(w*x+y*z), 1 - 2*(x*x+y*y));
    }
};
