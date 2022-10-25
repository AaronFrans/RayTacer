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

		float totalPitch{ 0 };
		float totalYaw{ 0 };

		Matrix cameraToWorld{};


		Matrix CalculateCameraToWorld()
		{
			Vector3 worldUp{ Vector3::UnitY };
			right = { Vector3::Cross(worldUp, forward).Normalized() };
			up = { Vector3::Cross(forward, right).Normalized() };
			Matrix cameraToWorld{
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
			const float defaultMoveSpeed{ 10 };
			float moveSpeed{ 10 };
			const float rotationSpeed{ 10 * TO_RADIANS};
			const Matrix cameraToWorld{ CalculateCameraToWorld() };

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);


			//FOV Buttons
			if (fovAngle > 25)
			{
				fovAngle -= pKeyboardState[SDL_SCANCODE_LEFT] * 0.5f;

			}
			else if (fovAngle < 120)
			{
				fovAngle += pKeyboardState[SDL_SCANCODE_RIGHT] * 0.5f;
			}

			// if shift pressed movement * 4
			if (pKeyboardState[SDL_SCANCODE_LSHIFT] || pKeyboardState[SDL_SCANCODE_RSHIFT])
				moveSpeed = defaultMoveSpeed * 4;
			else
				moveSpeed = defaultMoveSpeed;

			//WS for Forward-Backwards amount
			origin += pKeyboardState[SDL_SCANCODE_W] * moveSpeed * forward * deltaTime;
			origin += pKeyboardState[SDL_SCANCODE_S] * moveSpeed * -forward * deltaTime;


			//DA for Left-Right amount
			origin += pKeyboardState[SDL_SCANCODE_D] * moveSpeed * right * deltaTime;
			origin += pKeyboardState[SDL_SCANCODE_A] * moveSpeed * -right * deltaTime;

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);



			// both mouse buttons for Up-Dowm movement
			// mouse mask link:
			// https://stackoverflow.com/questions/71030102/how-to-detect-if-left-mousebutton-is-being-held-down-with-sdl2
			if (mouseState == (SDL_BUTTON_LMASK | SDL_BUTTON_RMASK))
			{
				if (mouseY > 0)
				{
					origin += moveSpeed * deltaTime * up;
				}
				else if (mouseY < 0)
				{
					origin += -moveSpeed * deltaTime * up;
				}
			}
			else if (mouseState & SDL_BUTTON_RMASK)
			{
				totalYaw += mouseX * rotationSpeed;
				totalPitch += mouseY * rotationSpeed;
			}
			else if (mouseState & SDL_BUTTON_LMASK)
			{
				origin += mouseY * moveSpeed * forward * deltaTime;

				totalYaw += mouseX * rotationSpeed;
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
