#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CGL { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {
  // Part 1, Task 4:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

    const double A = dot(r.d, r.d);
    const double B = dot(r.d, r.o - o) * 2;
    const double C = dot(r.o - o, r.o - o) - r2;

    const double det = B*B - 4*A*C;

    if(det < 0)
        return false;

    t1 = (-B - sqrt(det))/(2*A);
    t2 = (-B + sqrt(det))/(2*A);
  return true;
  
}

bool Sphere::intersect(const Ray& r) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

    double t1, t2;
    
    if(!test(r, t1, t2))
        return false;

    double t_inter = min(t1, t2);

    const bool hit = (t_inter > r.min_t && t_inter < r.max_t);
    return hit;
}

bool Sphere::intersect(const Ray& r, Intersection *isect) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.


    double t1, t2;
    
    if(!test(r, t1, t2))
        return false;

    double t_inter = min(t1, t2);

    const bool hit = (t_inter > r.min_t && t_inter < r.max_t);
    if(hit)
    {
        Vector3D normal = (r.o + t_inter*r.d - o);
        normal.normalize();
        r.max_t = t_inter;
        isect->t = t_inter; //replace this with your value of t
        isect->primitive = this;
        isect->n = normal; //replace this with your value of normal at the point of intersection
        isect->bsdf = get_bsdf();

        return true;
    }

  return false;

}

void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL
