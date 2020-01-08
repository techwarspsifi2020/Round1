#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL { namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
    mesh(mesh), v1(v1), v2(v2), v3(v3) { }

BBox Triangle::get_bbox() const {

  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
  BBox bb(p1);
  bb.expand(p2);
  bb.expand(p3);
  return bb;

}

bool Triangle::intersect(const Ray& r) const {

  // Part 1, Task 3: implement ray-triangle intersection
  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);

  Vector3D ab = p2-p1;
  Vector3D ac = p3-p1;
  Vector3D ae = r.o-p1;

  const bool parallel = dot(cross(ab, ac), r.d) == 0;

  if(parallel)
      return false;

  double mat_vals[9] = {r.d.x, ab.x, ac.x, r.d.y, ab.y, ac.y, r.d.z, ab.z, ac.z};
  Matrix3x3 mat = Matrix3x3(mat_vals);
  Vector3D sol = mat.inv() * ae;

  const bool outside = (sol.y + sol.z) > 1 || sol.y < 0 || sol.z < 0;
  const bool in_limits = (-sol.x > r.min_t) && (-sol.x < r.max_t);

  return !outside && in_limits;

}


bool Triangle::intersect(const Ray& r, Intersection *isect) const {

  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
  Vector3D n1(mesh->normals[v1]), n2(mesh->normals[v2]), n3(mesh->normals[v3]);

  Vector3D ab = p2-p1;
  Vector3D ac = p3-p1;
  Vector3D ae = r.o-p1;

  const bool parallel = dot(cross(ab, ac), r.d) == 0;

  if(parallel)
      return false;

  double mat_vals[9] = {r.d.x, ab.x, ac.x, r.d.y, ab.y, ac.y, r.d.z, ab.z, ac.z};
  Matrix3x3 mat = Matrix3x3(mat_vals);
  Vector3D sol = mat.inv() * ae;

  const bool outside = (sol.y + sol.z) > 1 || sol.y < 0 || sol.z < 0;
  const bool in_limits = (-sol.x > r.min_t) && (-sol.x < r.max_t);

  if (!outside && in_limits) { // if there is an intersection, the if condition should be true

    r.max_t = -sol.x;
    isect->t = -sol.x; //replace this with your value of t
    isect->primitive = this;
    isect->n = (1-sol.y-sol.z)*n1+sol.y*n2+sol.z*n3; //replace this with your value of normal at the point of intersection
    isect->bsdf = get_bsdf();

    return true;
  }

  return false;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}



} // namespace StaticScene
} // namespace CGL
