#include "Lidar_SF45.h"
#include "SerialIF.h"
#include "UAVCommonProtocol.h"
#include "UAVLinkCommon.h"
#include "SF45Parser.h"
#include "MavLink.h"
#include "vehicle_info.h"

#include <sstream>

using namespace chrono;

Mail<char, 1024> lidar_sf45_mail_box;

void Lidar_SF45::initialize(const bool& isWarmBoot)
{
	Serial_D4_initialize("SF45", 115200);

	if (isWarmBoot == false)
	{
		send_command(SF45_command::None, false, nullptr, 0);
		ThisThread::sleep_for(100ms);

		send_update_rate_command(1000);
		ThisThread::sleep_for(100ms);

		send_output_format_command(true, false, false, false, false, false, false, false, true);
		ThisThread::sleep_for(100ms);

		send_stream_command(true);
		ThisThread::sleep_for(100ms);
	}

	Serial_D4_RegisterCallback(LidarSF45_Sio_isr);

#if (USE_FC_AVOIDANCE_FUNC == 1)
	// In FC version, we just operate the LiDAR always.
	enable_measurement(true);
#else
	// In SF45 birdcom version, we operate the LiDAR only when the vehicle is armed.
	MAVLink* pMAVLink = static_cast<MAVLink*>(g_pUAVLinkBase);
	pMAVLink->callback_armingChanged = enable_measurement;
#endif

	LidarSF45_ManagerThread.start(callback(SF45_process_thread, this));
}



void Lidar_SF45::SF45_process_thread(Lidar_SF45* pLidar)
{
	osEvent evt;

	SF45Parser parser;

	LidarStatus lidarStatus = Lidar_Alive;

	int lastReceivedSectorID = 0;
	int minDistanceInSector = LiDAR_SF45_MAX_DISTANCE_cm;
	//for debug
	float angleWithMinDistance = 0;

	while (lidarStatus == Lidar_Alive)
	{
		evt = lidar_sf45_mail_box.get();

		if (evt.status == osEventMail)
		{
			char* data = (char*)evt.value.p;
			
			bool isParsingCompleted = parser.push_data_to_parser((uint8_t)*data);
			if (isParsingCompleted == true)
			{
				pair<SF45_command, bool> parsedCommand = parser.pop_parsed_command();
				if (parsedCommand.second == true && parsedCommand.first == SF45_command::DistanceDataInCm)
				{
					pair<SF45_distanceRecord, bool> parsedDistanceRecord = parser.pop_parsed_distance_record();
					if (parsedDistanceRecord.second == true)
					{
						SF45_distanceRecord record = parsedDistanceRecord.first;
						
#if (USE_FC_AVOIDANCE_FUNC == 1)
						int sectorID = translate_angle_to_sector_id(record.angle);
						if (sectorID != lastReceivedSectorID)
						{
							g_pUAVCommandBase->DoDistanceSensor(LiDAR_SYSTEM_ID, LiDAR_COMPONENT_ID, minDistanceInSector, LiDAR_SF45_MAX_DISTANCE_cm, LiDAR_SF45_MIN_DISTANCE_cm, lastReceivedSectorID);
							
							lastReceivedSectorID = sectorID;
							minDistanceInSector = record.distance;
							angleWithMinDistance = record.angle;
						}
						else
						{
							if (record.distance < minDistanceInSector)
							{
								minDistanceInSector = record.distance;
								angleWithMinDistance = record.angle;
							}
						}
#else
						if (callback_lidarDistance != nullptr)
						{
							time_t tCnt = time(NULL);
							int ret = (*callback_lidarDistance)((unsigned int)tCnt, record.angle, record.distance);
						}
#endif
					}
				}
			}

			lidar_sf45_mail_box.free(data);
		}
	}
}



void Lidar_SF45::send_command(const SF45_command& command, bool Write, uint8_t* Data, uint32_t DataSize)
{
	vector<uint8_t> packetBuffer;
	uint32_t payloadLength = 1 + DataSize;
	uint16_t flags = (payloadLength << 6) | (Write & 0x1);

	packetBuffer.push_back(LiDAR_SF45_PACKET_START_BYTE);		// Start byte.
	packetBuffer.push_back(((uint8_t*)&flags)[0]);				// Flags low.
	packetBuffer.push_back(((uint8_t*)&flags)[1]);				// Flags high.
	packetBuffer.push_back(static_cast<uint8_t>(command));							// Payload: Command ID.

	for (int i = 0; i < DataSize; i++)								//Payloads
		packetBuffer.push_back(Data[i]);

	uint16_t crc = SF45Parser::calculate_crc(packetBuffer);

	packetBuffer.push_back(((uint8_t*)&crc)[0]);		// Checksum low.
	packetBuffer.push_back(((uint8_t*)&crc)[1]);		// Checksum high.

	write_data_stream(packetBuffer);
}



void Lidar_SF45::write_data_stream(const vector<uint8_t>& packetBuffer)
{
	for (int i = 0; i < packetBuffer.size(); i++)
	{
		Serial_D4_put(packetBuffer.at(i));
	}
}



void Lidar_SF45::send_scan_start_command(const bool& isStart)
{
	if(isStart == true)
		send_command(SF45_command::ScanEnable, true, (uint8_t*)&isStart, 1);
	else
		send_command(SF45_command::ScanEnable, true, (uint8_t*)&isStart, 1);
}



void Lidar_SF45::send_scan_position_command(const float& angle)
{
	send_command(SF45_command::ScanPosition, true, (uint8_t*)&angle, 4);
}



void Lidar_SF45::send_update_rate_command(const int& targetHz)
{
	// Send update rate option command.
	// For specific format, refer https://support.lightware.co.za/sf45b/#/command_detail/command%20descriptions/66.%20update%20rate


	uint8_t updateRateCommandValue = 1;

	if (targetHz <= 50)
		updateRateCommandValue = 1;
	else if (targetHz <= 100)
		updateRateCommandValue = 2;
	else if (targetHz <= 200)
		updateRateCommandValue = 3;
	else if (targetHz <= 400)
		updateRateCommandValue = 4;
	else if (targetHz <= 500)
		updateRateCommandValue = 5;
	else if (targetHz <= 625)
		updateRateCommandValue = 6;
	else if (targetHz <= 1000)
		updateRateCommandValue = 7;
	else if (targetHz <= 1250)
		updateRateCommandValue = 8;
	else if (targetHz <= 1538)
		updateRateCommandValue = 9;
	else if (targetHz <= 2000)
		updateRateCommandValue = 10;
	else if (targetHz <= 2500)
		updateRateCommandValue = 11;
	else
		updateRateCommandValue = 12;

	send_command(SF45_command::UpdateRate, true, (uint8_t*)&updateRateCommandValue, 1);
}



void Lidar_SF45::send_output_format_command(const bool& bFirstRaw, const bool& bFirstFilter, const bool& bFirstStrength, const bool& bLastRaw, const bool& bLastFilter, const bool& bLastStrength, const bool& bBackgroundNoise, const bool& bTemperature, const bool& bYawAngle)
{
	// Send output format option command.
	// For specific format, refer https://support.lightware.co.za/sf45b/#/command_detail/command%20descriptions/27.%20distance%20output

	stringstream outputFormatStream;
	outputFormatStream << noboolalpha << bFirstRaw << bFirstFilter << bFirstStrength << bLastRaw << bLastFilter << bLastStrength << bBackgroundNoise << bTemperature << bYawAngle;
	string outputFormatStr = outputFormatStream.str();

	uint32_t outputFormat = 0;
	for (int i = 0; i < outputFormatStr.size(); i++)
	{
		if (outputFormatStr.c_str()[i] == '1')
			outputFormat |= (0x1 << i);
	}

	send_command(SF45_command::DistanceOutput, true, (uint8_t*)&outputFormat, 4);
}



void Lidar_SF45::send_stream_command(const bool& isStreamOn)
{
	uint32_t parameter = 0;
	if (isStreamOn)
		parameter = 5;

	send_command(SF45_command::Stream, true, (uint8_t*)&parameter, 4);
}



void Lidar_SF45::enable_measurement(const bool& isStart)
{
	send_scan_start_command(isStart);
	send_stream_command(isStart);

	if (isStart == false)
		send_scan_position_command(0);
}



int Lidar_SF45::translate_angle_to_sector_id(const float& angle)
{
	// Sector is categorized into 8 sectors - refer here: https://ardupilot.org/dev/docs/code-overview-object-avoidance.html
	// Angle range: -180 ~ 180

	int sectorID = -1;

	if (angle < -157.5) //Backward sector
		sectorID = 4;
	else if (angle < -112.5)
		sectorID = 5;
	else if (angle < -67.5)
		sectorID = 6;
	else if (angle < -22.5)
		sectorID = 7;
	else if (angle < 22.5) //Forward sector
		sectorID = 0;
	else if (angle < 67.5)
		sectorID = 1;
	else if (angle < 112.5)
		sectorID = 2;
	else if (angle < 157.5)
		sectorID = 3;
	else
		sectorID = 4; //Backward sector

	return sectorID;
}



/// Register Call-Back function
void Lidar_SF45::LidarSF45_RegisterCallback(int (*cb)(uint32_t ticks, int angle, int distance))
{
	callback_lidarDistance = cb;
}



void LidarSF45_Sio_isr()
{
	char ch = Serial_D4_get();

	char* mail = (char*)lidar_sf45_mail_box.alloc();
	if (mail != NULL)
	{
		*mail = ch;
		lidar_sf45_mail_box.put(mail);
	}
}