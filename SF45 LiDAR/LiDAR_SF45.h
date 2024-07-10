#pragma once

#include "mbed.h"
#include "ConstForLidar.h"

#include <set>
#include <cstdlib>
#include <list>
#include <vector>

using namespace std;

class Lidar_SF45
{
private:
	bool m_isActive = false;

public:
	Lidar_SF45() = default;
	~Lidar_SF45() = default;

	const bool& get_is_active() const { return m_isActive; }	
	void set_is_active(const bool& isActive) { m_isActive = isActive; }

	void initialize(const bool& isWarmBoot);

	static void SF45_process_thread(Lidar_SF45* pLidar);
	static void LidarSF45_RegisterCallback(int (*cb)(uint32_t ticks, int angle, int distance));

	static void send_command(const SF45_command& command, bool Write, uint8_t* Data, uint32_t DataSize);
	static void write_data_stream(const vector<uint8_t>& packetBuffer);

	static void send_scan_start_command(const bool& isStart);

	static void send_scan_position_command(const float& angle);

	static void send_update_rate_command(const int& targetHz);

	static void send_output_format_command(const bool& bFirstRaw, const bool& bFirstFilter, const bool& bFirstStrength,
		const bool& bLastRaw, const bool& bLastFilter, const bool& bLastStrength,
		const bool& bBackgroundNoise, const bool& bTemperature, const bool& bYawAngle);

	static void send_stream_command(const bool& isStreamOn);

	static void enable_measurement(const bool& isStart);

	static int translate_angle_to_sector_id(const float& angle);
};

static Thread LidarSF45_ManagerThread;

static void LidarSF45_Sio_isr();