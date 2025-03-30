#include "Triangle.h"
#include "CoreLayer/Math/Geometry.h"
#include "FastMath/VecMat.h"
#include <FunctionLayer/Acceleration/Linear.h>
//--- Triangle ---
Triangle::Triangle(int _primID, int _vtx0Idx, int _vtx1Idx, int _vtx2Idx,
                   const TriangleMesh *_mesh)
    : primID(_primID), vtx0Idx(_vtx0Idx), vtx1Idx(_vtx1Idx), vtx2Idx(_vtx2Idx),
      mesh(_mesh) {
  Point3f vtx0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]),
          vtx1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]),
          vtx2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);
  boundingBox.Expand(vtx0);
  boundingBox.Expand(vtx1);
  boundingBox.Expand(vtx2);
  this->geometryID = mesh->geometryID;
}

bool Triangle::rayIntersectShape(Ray &ray, int *primID, float *u,
                                 float *v) const {
  //* todo 实现三角形与光线求交

  // transform ray WITHOUT SCALE
  Point3f origin = ray.origin;
  Vector3f direction = ray.direction;

  // 求交点
  Point3f V0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]),
          V1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]),
          V2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);
  Vector3f E1 = V1 - V0, E2 = V2 - V0, T = origin - V0;
  Vector3f Q = cross(T, E1), P = cross(direction, E2);
  float inv = 1.f / dot(P, E1);
  float t = dot(Q, E2) * inv;
  *u = dot(P, T) * inv, *v = dot(Q, direction) * inv;
  if (*u >= 0 && *v >= 0 && *u + *v <= 1 && t >= ray.tNear && t <= ray.tFar) {
    ray.tFar = std::min(ray.tFar, t);
    *primID = this->primID;
    return true;
  }
  return false;
}

void Triangle::fillIntersection(float distance, int primID, float u, float v,
                                Intersection *intersection) const {
  // 该函数实际上不会被调用
  return;
}

//--- TriangleMesh ---
TriangleMesh::TriangleMesh(const Json &json) : Shape(json) {
  const auto &filepath = fetchRequired<std::string>(json, "file");
  meshData = MeshData::loadFromFile(filepath);
}

RTCGeometry TriangleMesh::getEmbreeGeometry(RTCDevice device) const {
  RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

  float *vertexBuffer = (float *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float),
      meshData->vertexCount);
  for (int i = 0; i < meshData->vertexCount; ++i) {
    Point3f vertex = transform.toWorld(meshData->vertexBuffer[i]);
    vertexBuffer[3 * i] = vertex[0];
    vertexBuffer[3 * i + 1] = vertex[1];
    vertexBuffer[3 * i + 2] = vertex[2];
  }

  unsigned *indexBuffer = (unsigned *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
      3 * sizeof(unsigned), meshData->faceCount);
  for (int i = 0; i < meshData->faceCount; ++i) {
    indexBuffer[i * 3] = meshData->faceBuffer[i][0].vertexIndex;
    indexBuffer[i * 3 + 1] = meshData->faceBuffer[i][1].vertexIndex;
    indexBuffer[i * 3 + 2] = meshData->faceBuffer[i][2].vertexIndex;
  }
  rtcCommitGeometry(geometry);
  return geometry;
}

bool TriangleMesh::rayIntersectShape(Ray &ray, int *primID, float *u,
                                     float *v) const {
  //* 当使用embree加速时，该方法不会被调用
  int geomID = -1;
  return acceleration->rayIntersect(ray, &geomID, primID, u, v);
}

void TriangleMesh::fillIntersection(float distance, int primID, float u,
                                    float v, Intersection *intersection) const {
  //* todo 填充光线与三角网格求交得到的交点信息
  intersection->distance = distance;
  intersection->shape = this;

  Point3f
      V0 = transform.toWorld(
          meshData->vertexBuffer[meshData->faceBuffer[primID][0].vertexIndex]),
      V1 = transform.toWorld(
          meshData->vertexBuffer[meshData->faceBuffer[primID][1].vertexIndex]),
      V2 = transform.toWorld(
          meshData->vertexBuffer[meshData->faceBuffer[primID][2].vertexIndex]);
  Vector3f E1 = V1 - V0, E2 = V2 - V0;

  //* 1. 在三角形内部用插值计算交点坐标
  Point3f hitpoint = V0 + u * E1 + v * E2;
  intersection->position = hitpoint;

  //* 2. 在三角形内部用插值计算法线
  intersection->normal = normalize(cross(E1, E2));

  //* 3. 在三角形内部用插值计算纹理坐标
  intersection->texCoord = Vector2f{u, v};

  //* 4. 在三角形内部用插值计算交点的切线和副切线
  Vector3f tangent = E1;
  Vector3f bitangent = normalize(cross(tangent, intersection->normal));
  tangent = normalize(cross(intersection->normal, bitangent));
  intersection->tangent = tangent;
  intersection->bitangent = bitangent;
}

void TriangleMesh::initInternalAcceleration() {
  acceleration = Acceleration::createAcceleration();
  int primCount = meshData->faceCount;
  for (int primID = 0; primID < primCount; ++primID) {
    int vtx0Idx = meshData->faceBuffer[primID][0].vertexIndex,
        vtx1Idx = meshData->faceBuffer[primID][1].vertexIndex,
        vtx2Idx = meshData->faceBuffer[primID][2].vertexIndex;
    std::shared_ptr<Triangle> triangle =
        std::make_shared<Triangle>(primID, vtx0Idx, vtx1Idx, vtx2Idx, this);
    acceleration->attachShape(triangle);
  }
  acceleration->build();
  // TriangleMesh的包围盒就是其内部加速结构的包围盒
  boundingBox = acceleration->boundingBox;
}
REGISTER_CLASS(TriangleMesh, "triangle")