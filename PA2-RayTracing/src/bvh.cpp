#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c);
  } else {
    draw(node->l, c);
    draw(node->r, c);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c);
  } else {
    drawOutline(node->l, c);
    drawOutline(node->r, c);
  }
}

static int counter = 0;
void splitbbs(BBox& bbox, const vector<Primitive *> &prims, int split_axis, vector<Primitive *> &left,vector<Primitive *> &right)
{
    for(Primitive *p : prims)
    {
        if(p->get_bbox().centroid()[split_axis] < bbox.centroid()[split_axis])
            left.push_back(p);
        else
            right.push_back(p);
    }
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // Part 2, Task 1:
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox centroid_box, bbox;

  for (Primitive *p : prims) {
    BBox bb = p->get_bbox();
    bbox.expand(bb);
    Vector3D c = bb.centroid();
    centroid_box.expand(c);
  }

  BVHNode *node = new BVHNode(bbox);
  int attempts = 0;

  if(prims.size() > max_leaf_size)
  {
      double longest = max(max(bbox.extent.x, bbox.extent.y), bbox.extent.z);
      int split_axis = (longest == bbox.extent.x)? 0: (longest == bbox.extent.y)? 1: 2;
      vector<Primitive *> left;
      vector<Primitive *> right;
      while(1)
      {
          attempts++;
          left.clear();
          right.clear();

        splitbbs(centroid_box, prims, split_axis, left, right);
        if(left.size() != 0 && right.size() != 0 || attempts > 3)
            break;

        split_axis = (split_axis + 1)%3;

      }

      if(left.size() == 0 || right.size() == 0) 
      {
          node->prims = new vector<Primitive *>(prims);
          return node;
      }
      node->l = construct_bvh(left, max_leaf_size);
      node->r = construct_bvh(right, max_leaf_size);
  }
  else
  {
      node->prims = new vector<Primitive *>(prims);
  }
  
  return node;
}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {
  // Part 2, task 3: replace this.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

    double t1, t2;
    if(!node)
        return false;

    total_isects++;
    if(node->bb.intersect(ray, t1, t2))
    {
        if(!node->l && !node->r)
        {
            //if(node->prims == NULL) return false;
            for(Primitive *p : *(node->prims))
            {
                total_isects++;
                if(p->intersect(ray))
                    return true;
            }
        }
        else
        {
            bool lh = intersect(ray, node->l);
            bool lr = intersect(ray, node->r);

            return lh || lr;

        }
    }
    else return false;

}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {
  // Part 2, task 3: replace this

    bool hit = false;
    double t1, t2;
    if(!node)
        return false;
    
    total_isects++;
    if(node->bb.intersect(ray, t1, t2))
    {
        if(!node->l && !node->r)
        {
            //if(node->prims == NULL) return false;
            for(Primitive *p : *(node->prims))
            {
                total_isects++;
                hit |= p->intersect(ray, i);
            }
            return hit;
        }
        else
        {
            bool lh = intersect(ray, i, node->l);
            bool lr = intersect(ray, i, node->r);

            return lh || lr;
        }
    }
    else return false;


}

}  // namespace StaticScene
}  // namespace CGL
