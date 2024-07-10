#include "rclcpp/rclcpp.hpp"
#include "SerialDriver.h"

#pragma once


#include <vector>
#include <list>

#include <string>
#include <cstring>
#include <iostream>
#include "ConstForSerialDriver.h"
#include <string>
#include "interfaces/msg/fc_mavlink_ctrl.hpp"

// #define LiDAR_VOA_TYPE LiDAR_VOA_NONE
//  LiDAR_VOA_TYPE should be defined to choose which Lidar to use in object avoidance.
//  Note that Downward LiDAR will always be enbaled for precision landing.

#define LiDAR_SYSTEM_ID 0x01
#define LiDAR_COMPONENT_ID 158

#define LiDAR_SF45_MAILBOX_SIZE 16
#define LiDAR_SF45_MAIL_EVT_RXSIO 0x01

#define LiDAR_SF45_PACKET_START_BYTE 0xAA
#define LiDAR_SF45_PACKET_TIMEOUT 100

#define LiDAR_SF45_MAX_RETRY 5
#define LiDAR_SF45_MAX_RESET 5

#define LiDAR_SF45_PACKET_SIZE 1024
#define LiDAR_SF45_PAYLOAD_LENGTH 1024

#define LiDAR_SF45_MIN_DISTANCE_cm 20
#define LiDAR_SF45_MAX_DISTANCE_cm 5000

// Orientation of a sensor
#define SENSOR_FORWARD 0
#define SENSOR_BACKWARD 4  // Backward
#define SENSOR_UPWARD 24   // Upward
#define SENSOR_DOWNWARD 25 // Downward

#define LiDAR_NUMBER_RETRY_TRIGGER_COUNT 10
#define LiDAR_NUMBER_DEAD_COUNT 5
#ifndef NUMBER_LIDAR_SENSORS
#define NUMBER_LIDAR_SENSORS 1
#endif

#define LiDAR_MAIL_EVT_RXSIO 0x01
#define LiDAR_BUFFER_SIZE 16
#define LiDAR_MAILBOX_SIZE (LiDAR_BUFFER_SIZE * NUMBER_LIDAR_SENSORS)

#define LiDAR_SAMPLING_CYCLE_HZ 20
#define LiDAR_COMMAND_GET_DISTANCE 0x64
#define LiDAR_MSG_POSTFIX 0x0D

// #define LiDAR_LW20_MIN_DISTANCE_cm 2
// #define LiDAR_LW20_MAX_DISTANCE_cm 10000
#define LIGHTWARE_OUT_OF_RANGE_ADD_CM 100

#define MIN_DWN_DISTANCE_FOR_BWD_FWD_LIDAR 500 // in cm

using namespace std;

enum MavlinkMsgType
{
    Lidar_Dwn = 0,
    Lidar_Fwd,
    Lidar_Bwd,
    Lidar_Upw,
    GimbalCam_Front
};

enum LidarStatus
{
    Lidar_Alive = 0,
    Lidar_Dead
};

struct SF45_flag
{
    uint16_t dataSize = 0;
    uint8_t reserved = 0;
    bool isWrite = false;
};

struct SF45_distanceRecord
{
    int distance = 0;
    float angle = 0.0f;
};

enum class SF45_parsing_state
{
    Idle,
    StartCodeReceived,
    FlagLowReceived,
    FlagHighReceived,
    PayloadReceiving,
    PayloadReceived,
    CRCReceived
};

// You may refer here for specific commands: https://support.lightware.co.za/sf45b/#/commands
enum class SF45_command
{
    ProductName = 0,
    Token = 10,
    SaveParameter = 12,
    Reset = 14,
    DistanceOutput = 27,
    Stream = 30,
    DistanceDataInCm = 44,
    DistanceDataInMm = 45,
    UpdateRate = 66,
    BaudRate = 79,
    ScanSpeed = 85,
    ScanEnable = 96,
    ScanPosition = 97,
    ScanOnStartup = 94,
    LowScanAngle = 98,
    HighScanAngle = 99,
    None = 100
};

struct lidar_data_t
{
    float sum;
    char buffer[LiDAR_BUFFER_SIZE];
    int nDat;
    int samples;
    int invalidCnt;
    int deadCount;
    int totalData;
    int prevTotalData;
    LidarStatus alive;
};

class LidarSF45Node : public rclcpp::Node
{
private:
    using FcMavlinkCtrl = interfaces::msg::FcMavlinkCtrl;

    lidar_data_t lidar_storage;
    rclcpp::Publisher<FcMavlinkCtrl>::SharedPtr fcMavlinkCtrl;
    
    
public:
    LidarSF45Node();
  //SerialDriver serialDriver;
    // recv lidar data and  
    //bool read_command(SF45_command& command, uint8_t* Data, uint32_t& DataSize);
    void communicate_serial(SerialDriver *serialDriver);
    bool SaveDataToSensorStorage(char dat);
   //void setScanSpeed(const int speed);
	void send_command(const SF45_command& command, bool Write, uint8_t* Data, uint32_t DataSize);
    void send_scan_start_command(const bool isStart);
    void send_scan_position_command(const float angle);
    void send_update_rate_command(const int targetHz);
    void send_output_format_command(const bool& bFirstRaw, const bool& bFirstFilter, const bool& bFirstStrength,
		const bool& bLastRaw, const bool& bLastFilter, const bool& bLastStrength,
		const bool& bBackgroundNoise, const bool& bTemperature, const bool& bYawAngle);
    void setLowScanAngle(const float angle);
    void setHighScanAngle(const float angle);
    void send_stream_command(const bool& isStreamOn);
    void enable_measurement(const bool& isStart);
	int translate_angle_to_sector_id(const float& angle);
 void mavlink_DoDistanceSensor(int sysid, int componentid, float distance, float max_distance, float min_distance, int orientation);
};


class SF45Parser
{
private:

    list<SF45_command> m_parsedCommands;
    list<SF45_distanceRecord> m_parsedDistanceRecords;

    //State variables for parsing
    SF45_parsing_state parsingState = SF45_parsing_state::Idle;
    uint8_t headerBuffer[2] = { 0, 0 };
    uint16_t payloadSize = 0;
    vector<uint8_t> payloadBuffer;
    uint16_t calculatedCRC = 0;

public:
    SF45Parser() = default;
    SF45Parser(const SF45Parser& rhs) { copy(rhs); }
    ~SF45Parser() = default;

    SF45Parser& operator=(const SF45Parser& rhs);

    void copy(const SF45Parser& rhs);

    pair<SF45_command, bool> pop_parsed_command();
    pair<SF45_distanceRecord, bool> pop_parsed_distance_record();

    void initialize_parsing_status();

    bool push_data_to_parser(const uint8_t& data);

    SF45_parsing_state parse_SF45_header(const uint8_t& data, const SF45_parsing_state& parsingState, uint8_t* headerBuffer);
    SF45_flag parse_SF45_flag(const uint16_t& flagBytes);

    SF45_parsing_state save_SF45_packet(const uint8_t& data, const SF45_parsing_state& parsingState, const uint16_t& payloadSize, vector<uint8_t>& payloadBuffer);

    SF45_distanceRecord parse_SF45_distance_record(const vector<uint8_t>& payloadBuffer);

    SF45_command translate_SF45_command_code(const uint8_t& commandCode);

    static int16_t transform_two_uint8_into_int16(const uint8_t& byte1, const uint8_t& byte2);

    static uint16_t calculate_crc(const uint8_t* headerBuffer, const vector<uint8_t>& payloadBuffer);
    static uint16_t calculate_crc(const vector<uint8_t>& packetBuffer);
};
