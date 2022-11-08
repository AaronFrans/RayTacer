//External includes
#include "SDL.h"
#include "SDL_surface.h"
#include <iostream>

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"
#include <thread>
#include <future>//async stuff
#include <ppl.h>//parrallel stuff


using namespace dae;


//#define ASYNC
#define PARALLEL_FOR

Renderer::Renderer(SDL_Window* pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
	m_WidthDivision = 1.f / m_Width;
	m_HeightDivision = 1.f / m_Height;
	m_AR = m_Width / static_cast<float>(m_Height);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	camera.CalculateCameraToWorld();

	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	const uint32_t numPixels = m_Width * m_Height;




#if defined(ASYNC)

	// Async task

	const uint32_t numCores{ std::thread::hardware_concurrency() };
	std::vector<std::future<void>> asyncFutures{};


	const uint32_t numPixelsPerTask{ numPixels / numCores };
	uint32_t numUnassignedPixels{ numPixels % numCores };
	uint32_t currentPixelIndex{ 0 };

	for (uint32_t coreId{ 0 }; coreId < numCores; ++coreId)
	{

		//create tasks
		uint32_t taskSize{ numPixelsPerTask };
		if (numUnassignedPixels > 0)
		{
			++taskSize;
			--numUnassignedPixels;
		}

		//add tasks to vector
		asyncFutures.push_back(
			std::async(std::launch::async,
				[=, this] {

					const uint32_t pixelIndexEnd = currentPixelIndex + taskSize;
					for (uint32_t pixelIndex{ currentPixelIndex }; pixelIndex < pixelIndexEnd; ++pixelIndex)
					{
						RenderPixel(pScene, pixelIndex, camera, lights, materials);
					}

				})
		);

		currentPixelIndex += taskSize;
	}

	//wait for tasks to finish


	for (auto& f : asyncFutures)
	{
		f.wait();
	}

#elif defined(PARALLEL_FOR)
	//Parrallel for logic

	concurrency::parallel_for(0u, numPixels,
		[=, this](int i)
		{
			RenderPixel(pScene, i, camera, lights, materials);
		});
#else

	//Synchronous logic
	for (uint32_t i = 0; i < numPixels; ++i)
	{
		RenderPixel(pScene, i, camera, lights, materials);
	}

#endif // defined(ASYNC)


	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}


void Renderer::RenderPixel(Scene* scene, uint32_t pixelIndex, const Camera& camera, const std::vector<Light>& lights, const std::vector<Material*>& materials) const
{
	float pxc, pyc, cx, cy;


	const int px = pixelIndex % m_Width;
	const int py = pixelIndex / m_Width;

	pxc = px + 0.5f;
	pyc = py + 0.5f;

	cx = ((2 * pxc * m_WidthDivision) - 1) * m_AR * camera.cameraFOV;
	cy = (1 - ((2 * pyc) * m_HeightDivision)) * camera.cameraFOV;

	Vector3 right, up, look;

	right = Vector3{ cx, 0, 0 };

	up = Vector3{ 0, cy, 0 };

	look = Vector3{ 0,0,1 };



	Vector3 rayDirection{ camera.cameraToWorld.TransformVector((right + up + look)).Normalized() };


	//Ray we cast from the camera to the pixel
	Ray viewRay{ camera.origin, rayDirection };

	//Color containing info about possible hit
	ColorRGB finalColor{};

	//HitRecord containing more info about possible hit
	HitRecord closestHit{};
	scene->GetClosestHit(viewRay, closestHit);

	//Testing
	//Sphere testSphere{ {0.f,0.f,100.f}, 50.f, 0 };
	//GeometryUtils::HitTest_Sphere(testPlane, viewRay, closestHit);

	//Plane testPlane{ {0.f,-50.f,0.f}, {0,1,0}, 0 };
	//GeometryUtils::HitTest_Plane(testPlane, viewRay, closestHit);

	if (closestHit.didHit)
	{

		//if hit change the color to the material color


		//verify t-values for sphere
		/*const float scaled_t = (closestHit.t - 50.f) / 40.f;
		finalColor = { scaled_t,scaled_t ,scaled_t };*/

		//verify t-values for plane
		//const float scaled_t = closestHit.t - 500.f;
		//finalColor = { scaled_t,scaled_t ,scaled_t };

		Vector3 hitPoint = closestHit.origin + closestHit.normal * 0.05f;

		for (auto& light : lights)
		{
			Vector3 lightDirection = LightUtils::GetDirectionToLight(light, hitPoint);

			float lightDistance{ lightDirection.Normalize() };

			if (m_ShadowsEnabled)
			{
				Ray lightRay = Ray{ hitPoint, lightDirection,  0.0001f, lightDistance };

				if (scene->DoesHit(lightRay))
				{
					continue;

				}
			}

			const float observedArea{ Vector3::Dot(closestHit.normal, lightDirection)};
			if (observedArea > 0)
			{
				switch (m_CurrentLightingMode)
				{
				case LightingMode::ObservedArea:
					finalColor += ColorRGB{ observedArea, observedArea, observedArea };
					break;
				case LightingMode::Radiance:
					finalColor += LightUtils::GetRadiance(light, closestHit.origin);
					break;
				case LightingMode::BRDF:
					finalColor += materials[closestHit.materialIndex]->
						Shade(closestHit, lightDirection, viewRay.direction);
					break;
				case LightingMode::Combined:
					finalColor += materials[closestHit.materialIndex]->
						Shade(closestHit, lightDirection, viewRay.direction) *
						LightUtils::GetRadiance(light, closestHit.origin) *
						observedArea;
				}
			}
		

		}


	}

	//Update Color in Buffer
	finalColor.MaxToOne();

	m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));



}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode()
{
	m_CurrentLightingMode = static_cast<LightingMode>((static_cast<int>(m_CurrentLightingMode) + 1) % 4);
}
