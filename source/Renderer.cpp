//External includes
#include "SDL.h"
#include "SDL_surface.h"
#include <iostream>;

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

using namespace dae;

Renderer::Renderer(SDL_Window* pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	Vector3 rayDirection{};
	Ray viewRay{};
	float ar{ m_Width / static_cast<float>(m_Height) };
	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{

#pragma region Calculate Ray Direction
			float pxc, pyc, cx, cy;

			pxc = px + 0.5f;
			pyc = py + 0.5f;

			cx = ((2 * pxc / m_Width) - 1) * ar;
			cy = 1 - ((2 * pyc) / m_Height);

			Vector3 right, up, look;

			right = Vector3{ cx, 0, 0 };

			up = Vector3{ 0, cy, 0 };

			look = Vector3{ 0,0,1 };


			rayDirection = (right + up + look).Normalized();
#pragma endregion


			//Ray we cast from the camera to the pixel
			viewRay = Ray{ camera.origin, rayDirection };

			//Color containing info about possible hit
			ColorRGB finalColor{};

			//HitRecord containing more info about possible hit
			HitRecord closestHit{};
			pScene->GetClosestHit(viewRay, closestHit);

			//Testing
			//Sphere testSphere{ {0.f,0.f,100.f}, 50.f, 0 };
			//GeometryUtils::HitTest_Sphere(testPlane, viewRay, closestHit);

			//Plane testPlane{ {0.f,-50.f,0.f}, {0,1,0}, 0 };
			//GeometryUtils::HitTest_Plane(testPlane, viewRay, closestHit);

			if (closestHit.didHit)
			{
				//if hit change the color to the material color
				finalColor = materials[closestHit.materialIndex]->Shade();

				//verify t-values for sphere
				/*const float scaled_t = (closestHit.t - 50.f) / 40.f;
				finalColor = { scaled_t,scaled_t ,scaled_t };*/

				//verify t-values for plane
				//const float scaled_t = closestHit.t - 500.f;
				//finalColor = { scaled_t,scaled_t ,scaled_t };

			}

			//Update Color in Buffer
			finalColor.MaxToOne();

			m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
	}

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}
