#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.

    Vector3D o2pm = pm.position - origin;
    if(o2pm.norm() < radius)
    {
        Vector3D tangent_point = origin + o2pm.unit() * radius;
        Vector3D correction_vector = tangent_point - pm.last_position;
        pm.position = pm.last_position + correction_vector * (1-friction);
    }
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  Misc::draw_sphere(shader, origin, radius * 0.92);
}
