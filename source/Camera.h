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
			cameraFOV = tanf(TO_RADIANS * fovAngle / 2.0f);
		}


		Vector3 origin{};
		float fovAngle{ 90.f };
		float cameraFOV{ 1.0f };

		float defaultMoveSpeed{ 10 };
		float moveSpeed{ 10 };
		float mouseMoveSpeed{ 2 };
		float rotationSpeed{ 10 * TO_RADIANS };

		Vector3 forward{ Vector3::UnitZ };
		Vector3 up{ Vector3::UnitY };
		Vector3 right{ Vector3::UnitX };

		float totalPitch{ 0 };
		float totalYaw{ 0 };

		Matrix cameraToWorld{};


		Matrix CalculateCameraToWorld()
		{
			Vector3 worldUp{ Vector3::UnitY };
			right = { Vector3::Cross(worldUp, forward).Normalized() };
			up = { Vector3::Cross(forward, right) };

			cameraToWorld = Matrix{
				right,
				up,
				forward,
				origin
			};

			return cameraToWorld;

		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();


			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

			// if shift pressed movement * 4
			if (pKeyboardState[SDL_SCANCODE_LSHIFT] || pKeyboardState[SDL_SCANCODE_RSHIFT])
				moveSpeed = defaultMoveSpeed * 4;
			else
				moveSpeed = defaultMoveSpeed;
		
			//WS for Forward-Backwards amount 
			origin += (pKeyboardState[SDL_SCANCODE_W] | pKeyboardState[SDL_SCANCODE_UP]) * moveSpeed * forward * deltaTime;
			origin += (pKeyboardState[SDL_SCANCODE_S] | pKeyboardState[SDL_SCANCODE_DOWN]) * moveSpeed * -forward * deltaTime;


			//DA for Left-Right amount				 
			origin += (pKeyboardState[SDL_SCANCODE_D] | pKeyboardState[SDL_SCANCODE_RIGHT]) * moveSpeed * right * deltaTime;
			origin += (pKeyboardState[SDL_SCANCODE_A] | pKeyboardState[SDL_SCANCODE_LEFT]) * moveSpeed * -right * deltaTime;

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

			switch (mouseState)
			{
			case SDL_BUTTON_LMASK:
				origin += mouseY * mouseMoveSpeed * forward * deltaTime;

				totalYaw += mouseX * rotationSpeed;
				break;
			case SDL_BUTTON_RMASK:
				totalYaw += mouseX * rotationSpeed;
				totalPitch -= mouseY * rotationSpeed;
				break;
			case SDL_BUTTON_X2:
				origin += mouseY * mouseMoveSpeed * deltaTime * up;
				break;
			}


			Matrix pitchMatrix{ Matrix::CreateRotationX(totalPitch * TO_RADIANS) };
			Matrix yawMatrix{ Matrix::CreateRotationY(totalYaw * TO_RADIANS) };
			Matrix rollMatrix{ Matrix::CreateRotationZ(0) };

			Matrix rotationMatrix{ pitchMatrix * yawMatrix * rollMatrix };

			forward = rotationMatrix.TransformVector(Vector3::UnitZ);
			forward.Normalize();

		}
	};
}
