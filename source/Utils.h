#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

#define MOLLERTRUMBORE

#define BVH


namespace dae
{
	namespace GeometryUtils
	{


#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			Vector3 originToSphere{ sphere.origin - ray.origin };
			float originToSphereDistanceSqr{ originToSphere.SqrMagnitude() };

			float oTSProjectedOnDirection{ Vector3::Dot(originToSphere, ray.direction) }; //oTS == origin To Sphere
			float oTSPerpDistanceSqr{ originToSphereDistanceSqr - (oTSProjectedOnDirection * oTSProjectedOnDirection) };
			float radiusSqr{ sphere.radius * sphere.radius };

			if (oTSPerpDistanceSqr > radiusSqr)
				return false;

			const float hitPointOnSphere{ sqrtf(radiusSqr - oTSPerpDistanceSqr) };

			float t = oTSProjectedOnDirection - hitPointOnSphere;

			if (t < ray.min || t > ray.max)
				return false;


			//check fors
			//
			if (!ignoreHitRecord)
			{
				hitRecord.didHit = true;
				hitRecord.materialIndex = sphere.materialIndex;
				hitRecord.origin = ray.origin + ray.direction * t;
				hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
				hitRecord.t = t;
			}


			return true;
		}


		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}

#pragma endregion

#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{

			//check if looking at plane
			float denom = Vector3::Dot(ray.direction, plane.normal);

			//formula
			//t = ((origin_plane - origin_ray) dot normal_plane) / direction_ray dot normal_plane

			if (denom > 0)
				return false;


			Vector3 rayToPlane = { plane.origin - ray.origin };
			float t{ (Vector3::Dot(rayToPlane, plane.normal) / denom) };
			if (ray.min < t && t < ray.max)
			{
				if (!ignoreHitRecord)
				{
					hitRecord.didHit = true;
					hitRecord.materialIndex = plane.materialIndex;
					hitRecord.origin = ray.origin + ray.direction * t;
					hitRecord.normal = plane.normal;
					hitRecord.t = t;
				}
				return true;
			}
			hitRecord.didHit = false;
			return false;


		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion

#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			TriangleCullMode mode{ triangle.cullMode };
			switch (triangle.cullMode)
			{
			case TriangleCullMode::BackFaceCulling:
			{
				switch (ignoreHitRecord)
				{
				case true:
					mode = TriangleCullMode::FrontFaceCulling;
					break;
				case false:
					mode = triangle.cullMode;
					break;
				}

			}
			break;
			case TriangleCullMode::FrontFaceCulling:
			{
				switch (ignoreHitRecord)
				{
				case true:
					mode = TriangleCullMode::BackFaceCulling;
					break;
				case false:
					mode = triangle.cullMode;
					break;
				}
			}
			break;
			}

			float dotNR{ Vector3::Dot(triangle.normal, ray.direction) };

			if (abs(dotNR) < 0) return false;


			switch (mode)
			{
			case dae::TriangleCullMode::FrontFaceCulling:
				if (dotNR < 0)
				{
					return false;
				}
				break;
			case dae::TriangleCullMode::BackFaceCulling:
				if (dotNR > 0)
				{
					return false;
				}
				break;
			}



#ifdef MOLLERTRUMBORE

			const Vector3 edge{ triangle.v1 - triangle.v0 };
			const Vector3 edge2{ triangle.v2 - triangle.v0 };

			const Vector3 rayEdgeCross{ Vector3::Cross(ray.direction, edge2) };

			const float f{ 1.0f / Vector3::Dot(edge, rayEdgeCross) };
			const Vector3 triangleV0ToRay{ ray.origin - triangle.v0 };
			const float u{ f * Vector3::Dot(triangleV0ToRay,rayEdgeCross) };

			if (u < 0.0f || u > 1.0f) return false;

			const Vector3 q{ Vector3::Cross(triangleV0ToRay, edge) };
			const float v{ f * Vector3::Dot(ray.direction, q) };

			if (v < 0.0f || u + v > 1.0f) return false;


			const float t{ f * Vector3::Dot(edge2, q) };

			if (t < ray.min || t > ray.max) return false;

			if (!ignoreHitRecord)
			{
				hitRecord.didHit = true;
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.origin = ray.origin + ray.direction * t;
				hitRecord.normal = triangle.normal;
				hitRecord.t = t;
			}
			return true;

#else

			Vector3 center = (triangle.v0 + triangle.v1 + triangle.v2) / 3;

			Vector3 centerToRayOrigin = center - ray.origin;
			float t{ Vector3::Dot(centerToRayOrigin, triangle.normal) / dotNR };

			if (t < ray.min || t > ray.max)
				return false;


			Vector3 p = ray.origin + t * ray.direction;

			Vector3 edgeToCheck = triangle.v1 - triangle.v0;
			Vector3 pointToSide = p - triangle.v0;
			Vector3 edgecheckCross = Vector3::Cross(edgeToCheck, pointToSide).Normalized();
			float rightSideCheck = Vector3::Dot(triangle.normal, edgecheckCross);
			if (rightSideCheck < 0)
				return false;

			edgeToCheck = triangle.v0 - triangle.v2;
			pointToSide = p - triangle.v2;
			edgecheckCross = Vector3::Cross(edgeToCheck, pointToSide).Normalized();
			rightSideCheck = Vector3::Dot(triangle.normal, edgecheckCross);
			if (rightSideCheck < 0)
				return false;

			edgeToCheck = triangle.v2 - triangle.v1;
			pointToSide = p - triangle.v1;
			edgecheckCross = Vector3::Cross(edgeToCheck, pointToSide).Normalized();
			rightSideCheck = Vector3::Dot(triangle.normal, edgecheckCross);
			if (rightSideCheck < 0)
				return false;

			if (!ignoreHitRecord)
			{
				hitRecord.origin = p;
				hitRecord.t = t;
				hitRecord.normal = triangle.normal;
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.didHit = true;
			}

			return true;

#endif // MOLLERTRUMBORE

		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion

#pragma region TriangeMesh HitTest


		inline bool SlabTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			float tXMin{ (mesh.transformedMinAABB.x - ray.origin.x) * ray.inversedDirection.x };
			float tXMax{ (mesh.transformedMaxAABB.x - ray.origin.x) * ray.inversedDirection.x };

			float tMin{ std::min(tXMin, tXMax) };
			float tMax{ std::max(tXMin, tXMax) };

			float tYMin{ (mesh.transformedMinAABB.y - ray.origin.y) * ray.inversedDirection.y };
			float tYMax{ (mesh.transformedMaxAABB.y - ray.origin.y) * ray.inversedDirection.y };

			tMin = std::min(tMin, std::min(tYMin, tYMax));
			tMax = std::max(tMax, std::max(tYMin, tYMax));

			float tZMin{ (mesh.transformedMinAABB.z - ray.origin.z) * ray.inversedDirection.z };
			float tZMax{ (mesh.transformedMaxAABB.z - ray.origin.z) * ray.inversedDirection.z };

			tMin = std::min(tMin, std::min(tZMin, tZMax));
			tMax = std::max(tMax, std::max(tZMin, tZMax));


			return tMax > 0 && tMax > tMin;
		}

#ifdef BVH
		inline void IntersectBVH(const TriangleMesh& mesh, const Ray& ray, Triangle& sharedTriangle, HitRecord& hitRecord, bool& hasHit, HitRecord& curClosestHit, bool ignoreHitRecord, unsigned int bvhNodeIdx)
		{

			BVHNode& node{ mesh.pBvhNodes[bvhNodeIdx] };

			if (!SlabTest_TriangleMesh(mesh, ray)) return;

			// If the current node is not the end node, recursively search the two child nodes 
			if (!node.IsLeaf())
			{
				IntersectBVH(mesh, ray, sharedTriangle, hitRecord, hasHit, curClosestHit, ignoreHitRecord, node.leftChild);
				IntersectBVH(mesh, ray, sharedTriangle, hitRecord, hasHit, curClosestHit, ignoreHitRecord, node.leftChild + 1);
				return;
			}

			// For each triangle in the node
			for (unsigned int triangleIdx{}; triangleIdx < node.indexCount; triangleIdx += 3)
			{
				// Set the position and normal of the current triangle to the triangle object
				sharedTriangle.v0 = mesh.transformedPositions[mesh.indices[node.firstIndex + triangleIdx]];
				sharedTriangle.v1 = mesh.transformedPositions[mesh.indices[node.firstIndex + triangleIdx + 1]];
				sharedTriangle.v2 = mesh.transformedPositions[mesh.indices[node.firstIndex + triangleIdx + 2]];
				sharedTriangle.normal = mesh.transformedNormals[(node.firstIndex + triangleIdx) / 3];

				// If the ray doesn't a triangle in the mesh, continue to the next triangle
				if (!HitTest_Triangle(sharedTriangle, ray, curClosestHit, ignoreHitRecord)) continue;

				// If the ray hits a triangle, set hasHit to true
				hasHit = true;

				// If the hit records needs to be ignored, it doesn't matter if there is a triangle closer or not, so just return
				if (ignoreHitRecord) return;

				// Check if the current hit is closer then the previous hit
				if (hitRecord.t > curClosestHit.t)
				{
					hitRecord = curClosestHit;
				}
			}
		}
#endif // BVH


		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
#ifdef BVH

			HitRecord tempRecord{};
			bool hasHit{};
			Triangle tri{};
			tri.cullMode = mesh.cullMode;
			tri.materialIndex = mesh.materialIndex;
			IntersectBVH(mesh, ray, tri, hitRecord, hasHit, tempRecord, ignoreHitRecord, 0);
#else
			if (!SlabTest_TriangleMesh(mesh, ray))
				return false;

			HitRecord tempRecord{};
			bool hasHit{};

			Triangle tri{};
			tri.cullMode = mesh.cullMode;
			tri.materialIndex = mesh.materialIndex;
			for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
			{
				tri.v0 = mesh.transformedPositions[mesh.indices[i]];
				tri.v1 = mesh.transformedPositions[mesh.indices[i + 1]];
				tri.v2 = mesh.transformedPositions[mesh.indices[i + 2]];
				tri.normal = mesh.transformedNormals[i / 3];

				if (HitTest_Triangle(tri, ray, tempRecord, ignoreHitRecord))
				{
					if (ignoreHitRecord) return true;

					if (hitRecord.t > tempRecord.t)
					{
						hitRecord = tempRecord;
					}
					hasHit = true;
				}
			}
#endif // BVH
			return hasHit;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion

	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			switch (light.type)
			{
			case LightType::Point:
				return Vector3{ light.origin - origin };
				break;
			default:
				return{};
			}
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			return light.color * (light.intensity / (light.origin - target).SqrMagnitude());
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof())
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if (isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}


#pragma warning(pop)
	}
}