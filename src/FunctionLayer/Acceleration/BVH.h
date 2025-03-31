#pragma once
#include "Acceleration.h"
class BVH : public Acceleration {
public:
  BVH() = default;
  void build() override;
  bool rayIntersect(Ray &ray, int *geomID, int *primID, float *u,
                    float *v) const override;

protected:
  static constexpr int bvhLeafMaxSize = 64;
  struct BVHNode;
  BVHNode *root;
  void build_helper(size_t l, size_t r, BVHNode *&now,
                    std::vector<std::shared_ptr<Shape>> &shapes, int dimension);
  bool rayIntersect_helper(size_t l, size_t r, BVHNode *now,
                           const std::vector<std::shared_ptr<Shape>> &shapes,
                           int dimension, Ray &ray, int *geomID, int *primID,
                           float *u, float *v) const;
};