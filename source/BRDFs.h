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
			return (kd * cd) * PI_INVERSE;
		}

		static ColorRGB Lambert(const ColorRGB& kd, const ColorRGB& cd)
		{

			return (kd * cd) * PI_INVERSE;
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

			const Vector3 lightReflection{ Vector3::Reflect(l, n) };

			const float angleCos{ std::max(Vector3::Dot(lightReflection, v),0.f) };
			const float phongSpecular = ks * powf(angleCos, exp);

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
			const float powVar{ 1 - Vector3::Dot(h, v) };
			return f0 +
				(ColorRGB{ 1 - f0.r, 1 - f0.g, 1 - f0.b }) *
				(powVar * powVar * powVar * powVar * powVar);
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

			const float roughnessSquared{ roughness * roughness }, piF{ static_cast<float>(M_PI) },
				normalHalfVectorSquared{ Square(Vector3::Dot(n, h)) };

			return  roughnessSquared / (piF *
				Square(normalHalfVectorSquared * (roughnessSquared - 1) + 1));
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
			const float dotNV(std::max(Vector3::Dot(n, v), 0.0f));

			const float k{ Square(roughness + 1) * 0.125f };

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