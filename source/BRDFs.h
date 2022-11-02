#pragma once
#include <cassert>
#include "Math.h"

namespace dae
{
	namespace BRDF
	{
		/**
		 * \param kd Diffuse Reflection Coefficient
		 * \param cd Diffuse Color
		 * \return Lambert Diffuse Color
		 */
		static ColorRGB Lambert(float kd, const ColorRGB& cd)
		{
			return (kd * cd) / static_cast<float>(M_PI);
		}

		static ColorRGB Lambert(const ColorRGB& kd, const ColorRGB& cd)
		{

			return (kd * cd) / static_cast<float>(M_PI);
		}

		/**
		 * \brief todo
		 * \param ks Specular Reflection Coefficient
		 * \param exp Phong Exponent
		 * \param l Incoming (incident) Light Direction
		 * \param v View Direction
		 * \param n Normal of the Surface
		 * \return Phong Specular Color
		 */
		static ColorRGB Phong(float ks, float exp, const Vector3& l, const Vector3& v, const Vector3& n)
		{
			// Phong: get reflection of the light, get angle between the light reflection and the view direction,
			//clamp the angle, calculate the Phong Specular and return as rgb value
			//Phong Specular = Specular Reflection Coefficient * angle of the light reflection ^Phong Exponent

			Vector3 lightReflection{ Vector3::Reflect(l, n) };

			float angleCos{ std::max(Vector3::Dot(lightReflection, v),0.f) };
			float phongSpecular = ks * powf(angleCos, exp);

			return ColorRGB{ phongSpecular ,phongSpecular ,phongSpecular };
		}

		/**
		 * \brief BRDF Fresnel Function >> Schlick
		 * \param h Normalized Halfvector between View and Light directions
		 * \param v Normalized View direction
		 * \param f0 Base reflectivity of a surface based on IOR (Indices Of Refrection), this is different for Dielectrics (Non-Metal) and Conductors (Metal)
		 * \return
		 */
		static ColorRGB FresnelFunction_Schlick(const Vector3& h, const Vector3& v, const ColorRGB& f0)
		{
			return f0 +
				(ColorRGB{ 1,1,1 } - f0) *
				powf((1 - (Vector3::Dot(h, v))), 5);
		}

		/**
		 * \brief BRDF NormalDistribution >> Trowbridge-Reitz GGX (UE4 implemetation - squared(roughness))
		 * \param n Surface normal
		 * \param h Normalized half vector
		 * \param roughness Roughness of the material
		 * \return BRDF Normal Distribution Term using Trowbridge-Reitz GGX
		 */
		static float NormalDistribution_GGX(const Vector3& n, const Vector3& h, float roughness)
		{

			float roughnessSquared{ powf(roughness, 2) }, piF{ static_cast<float>(M_PI) },
				normalHalfVectorSquared{ powf(Vector3::Dot(n, h), 2) };

			return  roughnessSquared / (piF *
				powf(normalHalfVectorSquared * (roughnessSquared - 1) + 1, 2));
		}


		/**
		 * \brief BRDF Geometry Function >> Schlick GGX (Direct Lighting + UE4 implementation - squared(roughness))
		 * \param n Normal of the surface
		 * \param v Normalized view direction
		 * \param roughness Roughness of the material
		 * \return BRDF Geometry Term using SchlickGGX
		 */
		static float GeometryFunction_SchlickGGX(const Vector3& n, const Vector3& v, float roughness)
		{
			float dotNV(Vector3::Dot(n, v));

			if (dotNV < 0)  dotNV = 0;

			float k{ powf(roughness + 1, 2) / 8 };

			return dotNV / ((dotNV) * (1 - k) + k);
		}

		/**
		 * \brief BRDF Geometry Function >> Smith (Direct Lighting)
		 * \param n Normal of the surface
		 * \param v Normalized view direction
		 * \param l Normalized light direction
		 * \param roughness Roughness of the material
		 * \return BRDF Geometry Term using Smith (> SchlickGGX(n,v,roughness) * SchlickGGX(n,l,roughness))
		 */
		static float GeometryFunction_Smith(const Vector3& n, const Vector3& v, const Vector3& l, float roughness)
		{
			return BRDF::GeometryFunction_SchlickGGX(n, v, roughness)
				* BRDF::GeometryFunction_SchlickGGX(n, l, roughness);
		}

	}
}