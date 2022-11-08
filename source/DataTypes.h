#pragma once
#include <cassert>

#include "Math.h"
#include "vector"

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
		int indicesCount{};
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
			for (size_t i = 0; i + 2 < indices.size(); i += 3)
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
					maxAABB = Vector3::Max(p, minAABB);
				}
			}
		}

		void UpdateTransforms()
		{
			const Matrix finalTranformation{ scaleTransform * rotationTransform * translationTransform };
			const Matrix normalTranformation{ rotationTransform * translationTransform };

			transformedPositions.clear();
			transformedPositions.reserve(positions.size());
			for (int i = 0; i < positions.size(); ++i)
			{
				transformedPositions.emplace_back(finalTranformation.TransformPoint(positions[i]));
			}

			transformedNormals.clear();
			transformedNormals.reserve(normals.size());
			for (int i = 0; i < normals.size(); ++i)
			{
				transformedNormals.emplace_back(normalTranformation.TransformVector(normals[i]));
			}

#ifdef BVH
			UpdateBVH();
#else
			UpdateTransformedAABB(finalTranformation);
#endif
		}

		void UpdateBVH()
		{
			if (!pBvhNodes) pBvhNodes = new BVHNode[indices.size()]{};

			bvhNodesUsed = 0;

			BVHNode& root = pBvhNodes[firstBvhNodeIdx];
			root.leftChild = 0;
			root.firstIndex = 0;
			root.indexCount = static_cast<unsigned int>(indices.size());

			UpdateBVHNodeBounds(firstBvhNodeIdx);

			Subdivide(firstBvhNodeIdx);
		}

		void UpdateBVHNodeBounds(int nodeIdx)
		{
			BVHNode& node{ pBvhNodes[nodeIdx] };

			node.minAABB = Vector3::Unit * FLT_MAX;
			node.MaxAABB = Vector3::Unit * FLT_MIN;

			for (unsigned int i{ node.firstIndex }; i < node.firstIndex + node.indexCount; ++i)
			{
				Vector3& curVertex{ transformedPositions[indices[i]] };
				node.minAABB = Vector3::Min(node.minAABB, curVertex);
				node.MaxAABB = Vector3::Max(node.MaxAABB, curVertex);
			}
		}

		void Subdivide(int nodeIdx)
		{
			BVHNode& node{ pBvhNodes[nodeIdx] };

			if (node.indexCount <= 6) return;

			// Determine split axis and position using SAH
			int axis{ -1 };
			float splitPos{ 0 };
			float cost{ FindBestSplitPlane(node, axis, splitPos) };

			const float noSplitCost{ CalculateNodeCost(node) };
			if (cost >= noSplitCost) return;

			// in-place partition
			int i{ static_cast<int>(node.firstIndex) };
			int j{ i + static_cast<int>(node.indexCount) - 1 };
			while (i <= j)
			{
				const Vector3 centroid{ (transformedPositions[indices[i]] +
					transformedPositions[indices[i + 1]] +
					transformedPositions[indices[i + 2]]) / 3.0f };

				if (centroid[axis] < splitPos)
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

			// abort split if one of the sides is empty
			unsigned int leftCount{ i - node.firstIndex };
			if (leftCount == 0 || leftCount == node.indexCount)
			{
				return;
			}

			// Create child nodes
			unsigned int leftChildIdx{ ++bvhNodesUsed };
			unsigned int rightChildIdx{ ++bvhNodesUsed };

			node.leftChild = leftChildIdx;

			pBvhNodes[leftChildIdx].firstIndex = node.firstIndex;
			pBvhNodes[leftChildIdx].indexCount = leftCount;
			pBvhNodes[rightChildIdx].firstIndex = i;
			pBvhNodes[rightChildIdx].indexCount = node.indexCount - leftCount;
			node.indexCount = 0;

			UpdateBVHNodeBounds(leftChildIdx);
			UpdateBVHNodeBounds(rightChildIdx);

			// Recurse
			Subdivide(leftChildIdx);
			Subdivide(rightChildIdx);
		}

		float FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos) const
		{
			float bestCost{ FLT_MAX };
			for (int curAxis{}; curAxis < 3; curAxis++)
			{
				float boundsMin{ FLT_MAX };
				float boundsMax{ FLT_MIN };
				for (unsigned int i{}; i < node.indexCount; i += 3)
				{
					const Vector3 centroid{ (transformedPositions[indices[node.firstIndex + i]] +
						transformedPositions[indices[node.firstIndex + i + 1]] +
						transformedPositions[indices[node.firstIndex + i + 2]])
						* 0.3333333f };
					boundsMin = std::min(centroid[curAxis], boundsMin);
					boundsMax = std::max(centroid[curAxis], boundsMax);
				}

				if (abs(boundsMin - boundsMax) < FLT_EPSILON) continue;

				const int nrBins{ 8 };

				Bin bins[nrBins];

				float scale{ nrBins / (boundsMax - boundsMin) };

				for (unsigned int i{}; i < node.indexCount; i += 3)
				{
					const Vector3& v0{ transformedPositions[indices[node.firstIndex + i]] };
					const Vector3& v1{ transformedPositions[indices[node.firstIndex + i + 1]] };
					const Vector3& v2{ transformedPositions[indices[node.firstIndex + i + 2]] };

					const Vector3 centroid{ (v0 + v1 + v2) / 3.0f };

					int binIdx{ std::min(nrBins - 1, static_cast<int>((centroid[curAxis] - boundsMin) * scale)) };

					bins[binIdx].indicesCount += 3;
					float areaBeforeGrow{ bins[binIdx].bounds.GetArea() };
					bins[binIdx].bounds.Grow(v0);
					bins[binIdx].bounds.Grow(v1);
					bins[binIdx].bounds.Grow(v2);
					float area{ bins[binIdx].bounds.max.x };
					int dada = 0;
				}

				float leftArea[nrBins - 1]{};
				float rightArea[nrBins - 1]{};
				float leftCount[nrBins - 1]{};
				float rightCount[nrBins - 1]{};

				AABB leftBox;
				AABB rightBox;
				float leftSum{};
				float rightSum{};

				for (int i{}; i < nrBins - 1; ++i)
				{
					leftSum += bins[i].indicesCount;
					leftCount[i] = leftSum;
					leftBox.Grow(bins[i].bounds);
					leftArea[i] = leftBox.GetArea();

					rightSum += bins[nrBins - 1 - i].indicesCount;
					rightCount[nrBins - 2 - i] = rightSum;
					rightBox.Grow(bins[nrBins - 1 - i].bounds);
					rightArea[nrBins - 2 - i] = rightBox.GetArea();
				}

				scale = (boundsMax - boundsMin) / nrBins;

				for (unsigned int i{}; i < nrBins - 1; ++i)
				{
					const float planeCost{ leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i] };

					if (planeCost < bestCost)
					{
						splitPos = boundsMin + scale * (i + 1);
						axis = curAxis;
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

		float EvaluateSAH(BVHNode& node, int axis, float pos) const
		{
			AABB leftBox{};
			AABB rightBox{};
			int leftCount{};
			int rightCount{};

			for (unsigned int i{}; i < node.indexCount; ++i)
			{
				const Vector3& v0{ transformedPositions[indices[node.firstIndex + i]] };
				const Vector3& v1{ transformedPositions[indices[node.firstIndex + i + 1]] };
				const Vector3& v2{ transformedPositions[indices[node.firstIndex + i + 2]] };

				const Vector3 centroid{ (v0 + v1 + v2) / 3.0f };

				if (centroid[axis] < pos)
				{
					leftCount++;
					leftBox.Grow(v0);
					leftBox.Grow(v1);
					leftBox.Grow(v2);
				}
				else
				{
					rightCount++;
					rightBox.Grow(v0);
					rightBox.Grow(v1);
					rightBox.Grow(v2);
				}
			}
			float cost{ leftCount * leftBox.GetArea() + rightCount * rightBox.GetArea() };
			return cost > 0 ? cost : FLT_MAX;
		}

		void UpdateTransformedAABB(const Matrix& finalTransform)
		{
			Vector3 tMinAABB{ finalTransform.TransformPoint(minAABB) };
			Vector3 tMaxAABB{ tMinAABB };
			// max, min, min
			Vector3 tAABB{ finalTransform.TransformPoint({maxAABB.x, minAABB.y, minAABB.z}) };
			tMinAABB = Vector3::Min(tMinAABB, tAABB);
			tMaxAABB = Vector3::Max(tMaxAABB, tAABB);
			// max, min, max
			tAABB = finalTransform.TransformPoint({ maxAABB.x, minAABB.y, maxAABB.z });
			tMinAABB = Vector3::Min(tMinAABB, tAABB);
			tMaxAABB = Vector3::Max(tMaxAABB, tAABB);
			//min, min, max
			tAABB = finalTransform.TransformPoint({ minAABB.x, minAABB.y, maxAABB.z });
			tMinAABB = Vector3::Min(tMinAABB, tAABB);
			tMaxAABB = Vector3::Max(tMaxAABB, tAABB);
			//max, max, min
			tAABB = finalTransform.TransformPoint({ maxAABB.x, maxAABB.y, minAABB.z });
			tMinAABB = Vector3::Min(tMinAABB, tAABB);
			tMaxAABB = Vector3::Max(tMaxAABB, tAABB);
			//max, max, max
			tAABB = finalTransform.TransformPoint({ maxAABB.x, maxAABB.y, maxAABB.z });
			tMinAABB = Vector3::Min(tMinAABB, tAABB);
			tMaxAABB = Vector3::Max(tMaxAABB, tAABB);
			//min, max, max
			tAABB = finalTransform.TransformPoint({ minAABB.x, maxAABB.y, maxAABB.z });
			tMinAABB = Vector3::Min(tMinAABB, tAABB);
			tMaxAABB = Vector3::Max(tMaxAABB, tAABB);

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