#pragma once
#include <cassert>

#include "Math.h"
#include "vector"

//bvh via: https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
#define BVH

namespace dae
{
#pragma region GEOMETRY
	struct Sphere
	{
		Vector3 origin{};
		float radius{};

		unsigned char materialIndex{ 0 };
	};

	struct Plane
	{
		Vector3 origin{};
		Vector3 normal{};

		unsigned char materialIndex{ 0 };
	};

	struct BVHNode
	{
		Vector3 minAABB{};
		Vector3 MaxAABB{};
		unsigned int leftChild{};
		unsigned int firstIndex{};
		unsigned int indexCount{};
		bool IsLeaf() { return indexCount > 0; };
	};

	struct AABB
	{
		Vector3 min{ Vector3::Unit * FLT_MAX };
		Vector3 max{ Vector3::Unit * FLT_MIN };
		void Grow(const Vector3& point)
		{
			min = Vector3::Min(min, point);
			max = Vector3::Max(max, point);
		}
		void Grow(const AABB& bounds)
		{
			min = Vector3::Min(min, bounds.min);
			max = Vector3::Max(max, bounds.max);
		}
		float GetArea()
		{
			Vector3 boxSize{ max - min };
			return boxSize.x * boxSize.y + boxSize.y * boxSize.z + boxSize.z * boxSize.x;
		}
	};

	struct Bin
	{
		AABB bounds{};
		int indexCount{};
	};


	enum class TriangleCullMode
	{
		FrontFaceCulling,
		BackFaceCulling,
		NoCulling
	};

	struct Triangle
	{
		Triangle() = default;
		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2, const Vector3& _normal) :
			v0{ _v0 }, v1{ _v1 }, v2{ _v2 }, normal{ _normal.Normalized() }{}

		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2) :
			v0{ _v0 }, v1{ _v1 }, v2{ _v2 }
		{
			const Vector3 edgeV0V1 = v1 - v0;
			const Vector3 edgeV0V2 = v2 - v0;
			normal = Vector3::Cross(edgeV0V1, edgeV0V2).Normalized();
		}

		Vector3 v0{};
		Vector3 v1{};
		Vector3 v2{};

		Vector3 normal{};

		TriangleCullMode cullMode{};
		unsigned char materialIndex{};
	};

	struct TriangleMesh
	{
		TriangleMesh() = default;
		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, TriangleCullMode _cullMode) :
			positions(_positions), indices(_indices), cullMode(_cullMode)
		{
			//Calculate Normals
			CalculateNormals();

			//Update Transforms
			UpdateTransforms();
		}

		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, const std::vector<Vector3>& _normals, TriangleCullMode _cullMode) :
			positions(_positions), indices(_indices), normals(_normals), cullMode(_cullMode)
		{
			UpdateTransforms();
		}


		~TriangleMesh()
		{
			delete[] pBvhNodes;
		}

		std::vector<Vector3> positions{};
		std::vector<Vector3> normals{};
		std::vector<int> indices{};
		unsigned char materialIndex{};

		TriangleCullMode cullMode{ TriangleCullMode::BackFaceCulling };

		Matrix rotationTransform{};
		Matrix translationTransform{};
		Matrix scaleTransform{};

		Vector3 minAABB;
		Vector3 maxAABB;

		Vector3 transformedMinAABB;
		Vector3 transformedMaxAABB;


		BVHNode* pBvhNodes{};
		unsigned int firstBvhNodeIdx{};
		unsigned int bvhNodesUsed{};

		std::vector<Vector3> transformedPositions{};
		std::vector<Vector3> transformedNormals{};

		void Translate(const Vector3& translation)
		{
			translationTransform = Matrix::CreateTranslation(translation);
		}

		void RotateY(float yaw)
		{
			rotationTransform = Matrix::CreateRotationY(yaw);
		}

		void Scale(const Vector3& scale)
		{
			scaleTransform = Matrix::CreateScale(scale);
		}

		void AppendTriangle(const Triangle& triangle, bool ignoreTransformUpdate = false)
		{
			int startIndex = static_cast<int>(positions.size());

			positions.emplace_back(triangle.v0);
			positions.emplace_back(triangle.v1);
			positions.emplace_back(triangle.v2);

			indices.emplace_back(startIndex);
			indices.emplace_back(++startIndex);
			indices.emplace_back(++startIndex);

			normals.emplace_back(triangle.normal);

			//Not ideal, but making sure all vertices are updated
			if (!ignoreTransformUpdate)
				UpdateTransforms();
		}

		void CalculateNormals()
		{
			for (size_t i{}; i + 2 < indices.size(); i += 3)
			{
				const Vector3& edgeV0V1 = positions[indices[i + 1]] - positions[indices[i]];
				const Vector3& edgeV0V2 = positions[indices[i + 2]] - positions[indices[i]];
				normals.emplace_back(Vector3::Cross(edgeV0V1, edgeV0V2).Normalized());
			}
		}


		void UpdateAABB()
		{
			if (positions.size() > 0)
			{
				minAABB = positions[0];
				maxAABB = positions[0];


				for (auto& p : positions)
				{
					minAABB = Vector3::Min(p, minAABB);
					maxAABB = Vector3::Max(p, maxAABB);
				}
			}
		}

		void UpdateTransforms()
		{
			const Matrix finalTranformation{ scaleTransform * rotationTransform * translationTransform };
			const Matrix normalTranformation{ rotationTransform * translationTransform };

			transformedPositions.clear();
			transformedPositions.reserve(positions.size());
			for (int i{}; i < positions.size(); ++i)
			{
				transformedPositions.emplace_back(finalTranformation.TransformPoint(positions[i]));
			}

			transformedNormals.clear();
			transformedNormals.reserve(normals.size());
			for (int i{}; i < normals.size(); ++i)
			{
				transformedNormals.emplace_back(normalTranformation.TransformVector(normals[i]));
			}

#ifdef BVH
			BuildBVH();
#else
			UpdateTransformedAABB(finalTranformation);
#endif
		}

		void BuildBVH()
		{
			if (!pBvhNodes) pBvhNodes = new BVHNode[indices.size()]{};

			bvhNodesUsed = 0;

			BVHNode& root = pBvhNodes[firstBvhNodeIdx];
			root.leftChild = 0;
			root.firstIndex = 0;
			root.indexCount = static_cast<unsigned int>(indices.size());

			MakeBVHNodeBounds(firstBvhNodeIdx);

			Subdivide(firstBvhNodeIdx);
		}

		void MakeBVHNodeBounds(int nodeIdx)
		{
			BVHNode& node{ pBvhNodes[nodeIdx] };

			node.minAABB = Vector3::Unit * FLT_MAX;
			node.MaxAABB = Vector3::Unit * FLT_MIN;

			for (unsigned int i{ node.firstIndex }; i < node.firstIndex + node.indexCount; ++i)
			{
				Vector3& currentVertex{ transformedPositions[indices[i]] };
				node.minAABB = Vector3::Min(node.minAABB, currentVertex);
				node.MaxAABB = Vector3::Max(node.MaxAABB, currentVertex);
			}
		}

		void Subdivide(int nodeIdx)
		{
			BVHNode& currentNode{ pBvhNodes[nodeIdx] };

			if (currentNode.indexCount <= 5) return;

			int axis{ -1 };
			float splitPosition{ 0 };
			const float cost{ CalculateBestSplitCost(currentNode, axis, splitPosition) };

			const float noSplitCost{ CalculateNodeCost(currentNode) };
			if (cost >= noSplitCost) return;

			int i{ static_cast<int>(currentNode.firstIndex) };
			int j{ i + static_cast<int>(currentNode.indexCount) - 1 };
			while (i <= j)
			{
				const Vector3 centroid{ (transformedPositions[indices[i]] +
					transformedPositions[indices[i + 1]] +
					transformedPositions[indices[i + 2]]) / 3.0f };

				if (centroid[axis] < splitPosition)
				{
					i += 3;
				}
				else
				{
					std::swap(indices[i], indices[j - 2]);
					std::swap(indices[i + 1], indices[j - 1]);
					std::swap(indices[i + 2], indices[j]);
					std::swap(normals[i / 3], normals[(j - 2) / 3]);
					std::swap(transformedNormals[i / 3], transformedNormals[(j - 2) / 3]);

					j -= 3;
				}
			}

			unsigned int leftCount{ i - currentNode.firstIndex };
			if (leftCount == 0 || leftCount == currentNode.indexCount)
			{
				return;
			}

			const unsigned int leftChildIdx{ ++bvhNodesUsed };
			const unsigned int rightChildIdx{ ++bvhNodesUsed };

			currentNode.leftChild = leftChildIdx;

			pBvhNodes[leftChildIdx].firstIndex = currentNode.firstIndex;
			pBvhNodes[leftChildIdx].indexCount = leftCount;
			pBvhNodes[rightChildIdx].firstIndex = i;
			pBvhNodes[rightChildIdx].indexCount = currentNode.indexCount - leftCount;
			currentNode.indexCount = 0;

			MakeBVHNodeBounds(leftChildIdx);
			MakeBVHNodeBounds(rightChildIdx);


			Subdivide(leftChildIdx);
			Subdivide(rightChildIdx);
		}

		float CalculateBestSplitCost(BVHNode& node, int& axis, float& splitPosition) const
		{
			float bestCost{ FLT_MAX };
			for (int currrentAxis{}; currrentAxis < 3; ++currrentAxis)
			{
				float boundsMin{ FLT_MAX };
				float boundsMax{ FLT_MIN };
				for (unsigned int i{}; i < node.indexCount; i += 3)
				{
					const Vector3 centroid{ (transformedPositions[indices[node.firstIndex + i]] +
						transformedPositions[indices[node.firstIndex + i + 1]] +
						transformedPositions[indices[node.firstIndex + i + 2]])
						* 0.3333333f };
					boundsMin = std::min(centroid[currrentAxis], boundsMin);
					boundsMax = std::max(centroid[currrentAxis], boundsMax);
				}

				if (abs(boundsMin - boundsMax) < FLT_EPSILON) continue;

				const int nrOfBins{ 8 };

				Bin bins[nrOfBins];

				float scale{ nrOfBins / (boundsMax - boundsMin) };

				for (unsigned int i{}; i < node.indexCount; i += 3)
				{
					const Vector3& v0{ transformedPositions[indices[node.firstIndex + i]] };
					const Vector3& v1{ transformedPositions[indices[node.firstIndex + i + 1]] };
					const Vector3& v2{ transformedPositions[indices[node.firstIndex + i + 2]] };

					const Vector3 centroid{ (v0 + v1 + v2) / 3.0f };

					int binIdx{ std::min(nrOfBins - 1, static_cast<int>((centroid[currrentAxis] - boundsMin) * scale)) };

					bins[binIdx].indexCount += 3;
					float areaBeforeGrow{ bins[binIdx].bounds.GetArea() };
					bins[binIdx].bounds.Grow(v0);
					bins[binIdx].bounds.Grow(v1);
					bins[binIdx].bounds.Grow(v2);
					float area{ bins[binIdx].bounds.max.x };
				}

				float leftArea[nrOfBins - 1]{};
				float rightArea[nrOfBins - 1]{};
				float leftCount[nrOfBins - 1]{};
				float rightCount[nrOfBins - 1]{};

				AABB leftBox;
				AABB rightBox;
				float leftSum{};
				float rightSum{};

				for (int i{}; i < nrOfBins - 1; ++i)
				{
					leftSum += bins[i].indexCount;
					leftCount[i] = leftSum;
					leftBox.Grow(bins[i].bounds);
					leftArea[i] = leftBox.GetArea();

					rightSum += bins[nrOfBins - 1 - i].indexCount;
					rightCount[nrOfBins - 2 - i] = rightSum;
					rightBox.Grow(bins[nrOfBins - 1 - i].bounds);
					rightArea[nrOfBins - 2 - i] = rightBox.GetArea();
				}

				scale = (boundsMax - boundsMin) / nrOfBins;

				for (unsigned int i{}; i < nrOfBins - 1; ++i)
				{
					const float planeCost{ leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i] };

					if (planeCost < bestCost)
					{
						splitPosition = boundsMin + scale * (i + 1);
						axis = currrentAxis;
						bestCost = planeCost;
					}
				}
			}
			return bestCost;
		}

		float CalculateNodeCost(const BVHNode& node) const
		{
			const Vector3 boxSize{ node.MaxAABB - node.minAABB };
			const float parentArea{ boxSize.x * boxSize.y + boxSize.y * boxSize.z + boxSize.z * boxSize.x };
			return node.indexCount * parentArea;
		}

		void UpdateTransformedAABB(const Matrix& finalTransform)
		{

			Vector3 tMinAABB = finalTransform.TransformPoint(minAABB);
			Vector3 tMaxAABB = tMinAABB;
			//max, min, min
			Vector3 tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// max, min, max
			tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// min, min, max
			tAABB = finalTransform.TransformPoint(minAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// min, max, min
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// max, max, min
			tAABB = finalTransform.TransformPoint(maxAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// max, max, max
			tAABB = finalTransform.TransformPoint(maxAABB);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// min, max, max
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			transformedMinAABB = tMinAABB;
			transformedMaxAABB = tMaxAABB;
		}


	};
#pragma endregion
#pragma region LIGHT
	enum class LightType
	{
		Point,
		Directional
	};

	struct Light
	{
		Vector3 origin{};
		Vector3 direction{};
		ColorRGB color{};
		float intensity{};

		LightType type{};
	};
#pragma endregion
#pragma region MISC
	struct Ray
	{
		Ray(const Vector3& _origin, const Vector3& _direction)
			: origin{ _origin }
			, direction{ _direction }
			, inversedDirection{ Vector3{ 1.0f / _direction.x,  1.0f / _direction.y,  1.0f / _direction.z } }
		{

		}
		Ray(const Vector3& _origin, const Vector3& _direction, float _min, float _max)
			: origin{ _origin }
			, direction{ _direction }
			, inversedDirection{ Vector3{ 1.0f / _direction.x,  1.0f / _direction.y,  1.0f / _direction.z } }
			, min{ _min }
			, max{ _max }
		{

		}

		Vector3 origin{};
		Vector3 direction{};
		Vector3 inversedDirection{};

		float min{ 0.0001f };
		float max{ FLT_MAX };
	};

	struct HitRecord
	{
		Vector3 origin{};
		Vector3 normal{};
		float t = FLT_MAX;

		bool didHit{ false };
		unsigned char materialIndex{ 0 };
	};
#pragma endregion
}