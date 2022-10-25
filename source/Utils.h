#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{

			Vector3 originToCenter{ sphere.origin - ray.origin };
			float originToCenterDistance{ originToCenter.Magnitude() };


			float t{ Vector3::Dot(originToCenter, ray.direction) };
			Vector3 originToCenterProjected{ ray.origin + ray.direction * t };

			float centerToProjectedLength{ (sphere.origin - originToCenterProjected).Magnitude() };

			if (centerToProjectedLength > sphere.radius)
			{
				hitRecord.didHit = false;
				return false;
			}

			float projectedToEdge = sqrt(sphere.radius * sphere.radius - centerToProjectedLength * centerToProjectedLength);

			float t1{ t - projectedToEdge };
			float t2{ t + projectedToEdge };

			// TODO:check for negatives
			float tActual{ t1 > t2 ? t2 : t1 };
			if (ray.min < tActual && tActual < ray.max)
			{
				if (!ignoreHitRecord)
				{

					hitRecord.didHit = true;
					hitRecord.materialIndex = sphere.materialIndex;
					hitRecord.t = tActual;

					hitRecord.origin = ray.origin + ray.direction * hitRecord.t;
					hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
				}

				return true;
			}

			hitRecord.didHit = false;
			return false;

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
			// t = ((origin_plane - origin_ray) dot normal_plane) / direction_ray dot normal_plane

			/*if (denom > 0)
			{
				hitRecord.didHit = false;
				return false;
			}*/


			Vector3 rayToPlane = { plane.origin - ray.origin };
			float t{ (Vector3::Dot(rayToPlane, plane.normal) / denom) };
			if (ray.min < t && t < ray.max)
			{
				if (!ignoreHitRecord)
				{
					hitRecord.didHit = true;
					hitRecord.materialIndex = plane.materialIndex;
					hitRecord.t = t;
					hitRecord.normal = plane.normal;
					hitRecord.origin = ray.origin + ray.direction * hitRecord.t;
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
			TriangleCullMode mode{};
			switch (triangle.cullMode)
			{
			case TriangleCullMode::BackFaceCulling:
			{
				mode = ignoreHitRecord ? TriangleCullMode::FrontFaceCulling : triangle.cullMode;
			}
			break;
			case TriangleCullMode::FrontFaceCulling:
			{
				mode = ignoreHitRecord ? TriangleCullMode::BackFaceCulling : triangle.cullMode;
			}
			break;
			default:
			{
				mode = triangle.cullMode;
			}
			break;
			}

			float dotNR{ Vector3::Dot(triangle.normal, ray.direction) };


			if (mode == TriangleCullMode::FrontFaceCulling && dotNR < 0)
			{
				return false;
			}
			if (mode == TriangleCullMode::BackFaceCulling && dotNR > 0)
			{
				return false;
			}
			if (dotNR == 0)
				return false;

			Vector3 center = (triangle.v0 + triangle.v1 + triangle.v2) / 3;

			Vector3 centerToRayOrigin = center - ray.origin;
			float t{ Vector3::Dot(centerToRayOrigin, triangle.normal) / dotNR };

			if (t < ray.min || t > ray.max)
				return false;



			Vector3 p = ray.origin + t * ray.direction;

			Vector3 edgeToCheck = triangle.v1 - triangle.v0;
			Vector3 pointToSide = p - triangle.v0;
			Vector3 edgecheckCross = Vector3::Cross(edgeToCheck, pointToSide).Normalized();
			float rightSideCheck = Vector3::Dot(triangle.normal,
				edgecheckCross);
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


		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion

#pragma region TriangeMesh HitTest
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			HitRecord toKeepRecord{};
			Triangle tri{};
			tri.cullMode = mesh.cullMode;
			tri.materialIndex = mesh.materialIndex;
			for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
			{
				tri.normal = mesh.transformedNormals[i / 3];
				tri.v0 = mesh.transformedPositions[mesh.indices[i]];
				tri.v1 = mesh.transformedPositions[mesh.indices[i + 1]];
				tri.v2 = mesh.transformedPositions[mesh.indices[i + 2]];

				if (!ignoreHitRecord)
				{
					HitTest_Triangle(tri, ray, toKeepRecord, ignoreHitRecord);
					if (toKeepRecord.t < hitRecord.t)
					{
						hitRecord = toKeepRecord;

					}
				}
				else
				{
					if (GeometryUtils::HitTest_Triangle(tri, ray))
						return true;
				}
			}


			return false;
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
			if (light.type == LightType::Point)
			{
				return Vector3{ light.origin - origin };
			}
			return{};
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