#include "transforms.h"
#define _USE_MATH_DEFINES

#include "CGL/matrix3x3.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"
#include <math.h>
#include <iostream>
namespace CGL {

Vector2D operator*(const Matrix3x3 &m, const Vector2D &v) {
	Vector3D mv = m * Vector3D(v.x, v.y, 1);
	return Vector2D(mv.x / mv.z, mv.y / mv.z);
}

Matrix3x3 translate(float dx, float dy) {
	// Part 3: Fill this in.
    double mat_trans[9] = {1,0, (double)dx, 0, 1, (double)dy, 0, 0, 1};
	return Matrix3x3(mat_trans);

}

Matrix3x3 scale(float sx, float sy) {
	// Part 3: Fill this in.
    double mat_scale[9] = {sx, 0, 0, 0, sy, 0, 0 ,0, 1};
	return Matrix3x3(mat_scale);
}

// The input argument is in degrees counterclockwise
Matrix3x3 rotate(float deg) {
	// Part 3: Fill this in.
    double degr = (double) (deg/180.0f * 3.14159265359f);
    double mat_rot[9] = {cos(degr), -sin(degr), 0, sin(degr), cos(degr), 0, 0, 0, 1};
    return Matrix3x3(mat_rot);

}

}
