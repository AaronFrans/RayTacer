#pragma once

//Standard includes
#include <cstdint>

namespace dae
{
	class Timer
	{
	public:
		Timer();
		virtual ~Timer() = default;

		Timer(const Timer&) = delete;
		Timer(Timer&&) noexcept = delete;
		Timer& operator=(const Timer&) = delete;
		Timer& operator=(Timer&&) noexcept = delete;

		void Reset();
		void Start();
		void Update();
		void Stop();

		uint32_t GetFPS() const { return m_FPS; };
		float GetdFPS() const { return m_dFPS; };
		float GetElapsed() const { return m_ElapsedTime; };
		float GetTotal() const { return m_TotalTime; };
		bool IsRunning() const { return !m_IsStopped; };

	private:
		uint64_t m_BaseTime{};
		uint64_t m_PausedTime{};
		uint64_t m_StopTime{};
		uint64_t m_PreviousTime{};
		uint64_t m_CurrentTime{};

		uint32_t m_FPS{};
		float m_dFPS{};
		uint32_t m_FPSCount{};

		float m_TotalTime {};
		float m_ElapsedTime{};
		float m_SecondsPerCount{};
		float m_ElapsedUpperBound{ 0.03f };
		float m_FPSTimer{};

		bool m_IsStopped = true;
		bool m_ForceElapsedUpperBound = false;
	};
}