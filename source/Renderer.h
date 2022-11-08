#pragma once

#include <cstdint>
#include <vector>

struct SDL_Window;
struct SDL_Surface;

namespace dae
{


	struct Camera;
	struct Light;

	class Scene;
	class Material;

	class Renderer final
	{
	public:
		Renderer(SDL_Window* pWindow);
		~Renderer() = default;

		Renderer(const Renderer&) = delete;
		Renderer(Renderer&&) noexcept = delete;
		Renderer& operator=(const Renderer&) = delete;
		Renderer& operator=(Renderer&&) noexcept = delete;

		void Render(Scene* pScene) const;



		void RenderPixel(Scene* scene, uint32_t pixelIndex,
			const Camera& camera, const std::vector<Light>& lights, const std::vector<Material*>& materials) const;

		bool SaveBufferToImage() const;

		void CycleLightingMode();
		void ToggleShadows() { m_ShadowsEnabled = !m_ShadowsEnabled; }


	private:
		enum class LightingMode
		{
			ObservedArea,
			Radiance,
			BRDF,
			Combined,
		};

		LightingMode m_CurrentLightingMode{ LightingMode::Combined };
		bool m_ShadowsEnabled{ true };


		SDL_Window* m_pWindow{};

		SDL_Surface* m_pBuffer{};
		uint32_t* m_pBufferPixels{};

		int m_Width{};
		float m_WidthDivision;
		int m_Height{};
		float m_HeightDivision = 1.f / m_Height;

		float m_AR{};
	};
}
