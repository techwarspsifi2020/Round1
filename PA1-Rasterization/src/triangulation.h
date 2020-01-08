// Original file Copyright CMU462 Fall 2015: 
// Kayvon Fatahalian, Keenan Crane,
// Sky Gao, Bryce Summers, Michael Choquette.
#ifndef CGL_TRIANGULATION_H
#define CGL_TRIANGULATION_H
#include "svg.h"

namespace CGL {

// triangulates a polygon and save the result as a triangle list
void triangulate(const Polygon& polygon, std::vector<Vector2D>& triangles );
bool inside(float Ax, float Ay,
            float Bx, float By,
            float Cx, float Cy,
            float Px, float Py);
} // namespace CGL

#endif // CGL_TRIANGULATION_H

