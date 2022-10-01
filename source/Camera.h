#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle) :
			origin{ _origin },
			fovAngle{ _fovAngle }
		{
		}


		Vector3 origin{};
		float fovAngle{ 90.f };

		Vector3 forward{ Vector3::UnitZ };
		Vector3 up{ Vector3::UnitY };
		Vector3 right{ Vector3::UnitX };

		float totalPitch{ 45.f };
		float totalYaw{ 180.f };

		Matrix cameraToWorld{};


		Matrix CalculateCameraToWorld()
		{
			Vector3 worldUp{ Vector3::UnitY };
			Vector3 cameraRight{ Vector3::Cross(worldUp, forward).Normalized() };
			Vector3 cameraUp{ Vector3::Cross(forward, cameraRight).Normalized() };
			Matrix cameraToWorld{
				cameraRight,
				cameraUp,
				forward,
				origin
			};

			return cameraToWorld;

		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();
			const float moveSpeed{ 10 };
			const float rotationSpeed{ 7 };
			const Matrix cameraToWorld{ CalculateCameraToWorld() };

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);


			//FOV Buttons
			if (fovAngle > 25)
			{
				fovAngle -= pKeyboardState[SDL_SCANCODE_LEFT] * 0.5;

			}
			else if (fovAngle < 120)
			{
				fovAngle += pKeyboardState[SDL_SCANCODE_RIGHT] * 0.5;
			}



			//WS for Forward-Backwards amount
			Vector3 direction{};
			direction.z += pKeyboardState[SDL_SCANCODE_W] * moveSpeed;
			direction.z += pKeyboardState[SDL_SCANCODE_S] * -moveSpeed;


			//DA for Left-Right amount
			direction.x += pKeyboardState[SDL_SCANCODE_D] * moveSpeed;
			direction.x += pKeyboardState[SDL_SCANCODE_A] * -moveSpeed;

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);



			// both mouse buttons for Up-Dowm movement
			if (mouseState & SDL_BUTTON_LMASK && mouseState & SDL_BUTTON_RMASK)
			{
				if (mouseY > 0)
				{
					direction.y = moveSpeed;
				}
				else if (mouseY < 0)
				{
					direction.y = -moveSpeed;
				}
			}
			else if (mouseState & SDL_BUTTON_RMASK)
			{
				totalYaw += mouseX * rotationSpeed * deltaTime;
				totalPitch += mouseY * rotationSpeed * deltaTime;
			}
			else if (mouseState & SDL_BUTTON_LMASK)
			{
				direction.z = mouseY * moveSpeed * deltaTime;


				totalYaw += mouseX * rotationSpeed * deltaTime;
			}

			Matrix pitchMatrix{ Matrix::CreateRotationX(totalPitch * TO_RADIANS) };
			Matrix yawMatrix{ Matrix::CreateRotationY(totalYaw * TO_RADIANS) };
			Matrix rollMatrix{ Matrix::CreateRotationZ(0) };

			Matrix rotationMatrix{ yawMatrix * pitchMatrix * rollMatrix };

			forward = rotationMatrix.TransformVector(Vector3::UnitZ);
			forward.Normalize();


			// if shift pressed movement * 4
			if (pKeyboardState[SDL_SCANCODE_LSHIFT] || pKeyboardState[SDL_SCANCODE_RSHIFT])
			{
				direction *= 4;
			}


			//make direction time dependand and move forward towards camera forward
			direction *= deltaTime;
			Vector3 transformedVector = cameraToWorld.TransformVector(direction);
			origin += transformedVector;
		}
	};
}
