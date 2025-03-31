#include "BVH.h"
#include "AABB.h"
#include "FunctionLayer/Shape/Shape.h"
#include <algorithm>
#include <cstddef>
#include <memory>
struct BVH::BVHNode {
  //* todo BVH节点结构设计
  AABB boundingBox;
  size_t lIndex, rIndex;
  BVHNode *ls;
  BVHNode *rs;
  BVHNode() = default;
};

void BVH::build_helper(size_t l, size_t r, BVH::BVHNode *&now,
                       std::vector<std::shared_ptr<Shape>> &shapes,
                       int dimension) {
  if (l > r) {
    now = nullptr;
    return;
  }
  now = new BVH::BVHNode();
  now->lIndex = l;
  now->rIndex = r;
  now->ls = now->rs = nullptr;
  if (r - l + 1 <= (size_t)BVH::bvhLeafMaxSize) { // leaf
    for (size_t i = l; i <= r; ++i) {
      now->boundingBox.Expand(shapes[i]->getAABB());
    }
    return;
  }
  auto cmpDim = [dimension](const std::shared_ptr<Shape> &shape1,
                            const std::shared_ptr<Shape> shape2) {
    return shape1->getAABB().Center()[dimension] <
           shape2->getAABB().Center()[dimension];
  };
  auto mid = (l + r) >> 1;
  std::nth_element(shapes.begin() + l, shapes.begin() + mid,
                   shapes.begin() + r + 1, cmpDim);
  int newDim = (dimension + 1) % 3;
  build_helper(l, mid, now->ls, shapes, newDim);
  build_helper(mid + 1, r, now->rs, shapes, newDim);
  now->boundingBox.Expand(now->ls->boundingBox);
  now->boundingBox.Expand(now->rs->boundingBox);
}
void BVH::build() {
  AABB sceneBox;
  for (const auto &shape : shapes) {
    //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
    //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
    //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
    //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
    shape->initInternalAcceleration();
    sceneBox.Expand(shape->getAABB());
    this->boundingBox.Expand(shape->getAABB());
  }
  //* todo 完成BVH构建
  root = nullptr;
  if (!shapes.empty()) {
    BVH::build_helper(0, shapes.size() - 1, root, shapes, 0);
  }
}
bool BVH::rayIntersect_helper(size_t l, size_t r, BVH::BVHNode *now,
                              const std::vector<std::shared_ptr<Shape>> &shapes,
                              int dimension, Ray &ray, int *geomID, int *primID,
                              float *u, float *v) const {
  bool hit = false;

  if (now->ls == nullptr && now->rs == nullptr) { // leaf
    for (auto i = l; i <= r; ++i) {
      if (shapes[i]->rayIntersectShape(ray, primID, u, v)) {
        hit = true;
        *geomID = i;
      }
    }
    return hit;
  }
  auto mid = (l + r) >> 1;
  auto newDim = (dimension + 1) % 3;
  if (ray.direction[dimension] > 0) { // left child first
    if (now->ls && now->ls->boundingBox.RayIntersect(ray)) {
      hit |= rayIntersect_helper(l, mid, now->ls, shapes, newDim, ray, geomID,
                                 primID, u, v);
    }
    if (now->rs && now->rs->boundingBox.RayIntersect(ray)) {
      hit |= rayIntersect_helper(mid + 1, r, now->rs, shapes, newDim, ray,
                                 geomID, primID, u, v);
    }
  } else { // right child first
    if (now->rs && now->rs->boundingBox.RayIntersect(ray)) {
      hit |= rayIntersect_helper(mid + 1, r, now->rs, shapes, newDim, ray,
                                 geomID, primID, u, v);
    }
    if (now->ls && now->ls->boundingBox.RayIntersect(ray)) {
      hit |= rayIntersect_helper(l, mid, now->ls, shapes, newDim, ray, geomID,
                                 primID, u, v);
    }
  }
  return hit;
}
bool BVH::rayIntersect(Ray &ray, int *geomID, int *primID, float *u,
                       float *v) const {
  //* todo 完成BVH求交
  if (shapes.empty())
    return false;
  bool hit = false;
  float tmpu, tmpv;
  hit = BVH::rayIntersect_helper(0, shapes.size() - 1, root, shapes, 0, ray,
                                 geomID, primID, &tmpu, &tmpv);
  // for (auto i = 0; i < shapes.size(); ++i) {
  //   if (shapes[i]->rayIntersectShape(ray, primID, u, v)) {
  //     hit = true;
  //     *geomID = i;
  //   }
  // }
  return hit;
}
