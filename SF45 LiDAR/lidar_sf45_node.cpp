#include "lidar_sf45_node.hpp"
//#include "SF45_Parser.hpp"

SerialDriver serialDriver; 
LidarSF45Node::LidarSF45Node() : Node("lidar_sf45_node")
{
  RCLCPP_INFO(get_logger(), "LidarSF45Node Started");
  fcMavlinkCtrl = this->create_publisher<FcMavlinkCtrl>("mavlink_control", 10);

  // ActivateLiDAR();
  send_command(SF45_command::None, false, nullptr, 0);
		//ThisThread::sleep_for(100ms);
   rclcpp::sleep_for(std::chrono::milliseconds(100));

		send_update_rate_command(50);
		//ros::Duration(1).sleep();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
		send_output_format_command(true, false, false, false, false, false, false, false, true);
		//ros::Duration(1).sleep();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    enable_measurement(false);
    
    send_stream_command(true);
   
		
     rclcpp::sleep_for(std::chrono::milliseconds(100));
    
  RCLCPP_INFO(get_logger(), "LidarSF45Node Activated");
}



void LidarSF45Node::setLowScanAngle(const float angle)
{
	send_command(SF45_command::LowScanAngle, true, (uint8_t*)&angle, 4);
}

void LidarSF45Node::setHighScanAngle(const float angle)
{
	send_command(SF45_command::HighScanAngle, true, (uint8_t*)&angle, 4);
}



void LidarSF45Node::communicate_serial(SerialDriver *serialDriver)
{rclcpp::sleep_for(std::chrono::seconds(2));
  enable_measurement(true);
  //rclcpp::sleep_for(std::chrono::seconds(5));
      setLowScanAngle(-60);
     rclcpp::sleep_for(std::chrono::milliseconds(100));
       setHighScanAngle(60);
       rclcpp::sleep_for(std::chrono::milliseconds(100));
 
		
     
     
  //serialDriver->write_data(LiDAR_COMMAND_GET_DISTANCE);
  SF45Parser parser;
    LidarStatus lidarStatus = Lidar_Alive;
    int lastReceivedSectorID = 0;
    int minDistanceInSector = LiDAR_SF45_MAX_DISTANCE_cm;
    float angleWithMinDistance = 0;

    while (lidarStatus == Lidar_Alive) {
        serialDriver->read_data();

        if (!serialDriver->get_read_Q().empty()) {
            //std::cout << "2" << std::endl;
            SerialData readData;
            serialDriver->pop_read_Q(readData);

            // Parse the readData according to the SF45/B data format
            for (const auto& byte : readData.data) {
                //std::cout << (uint8_t)byte << std::endl; // Print each byte
                bool isParsingCompleted = parser.push_data_to_parser(byte);
                if (isParsingCompleted) {
                //cout<<"yo"<<endl;
                    auto parsedCommand = parser.pop_parsed_command();
                    if (parsedCommand.second && parsedCommand.first == SF45_command::DistanceDataInCm) {
                        auto parsedDistanceRecord = parser.pop_parsed_distance_record();
                        if (parsedDistanceRecord.second) {
                            SF45_distanceRecord record = parsedDistanceRecord.first;
                            int sectorID = translate_angle_to_sector_id(record.angle);
                            if (sectorID != lastReceivedSectorID) {
                                mavlink_DoDistanceSensor(LiDAR_SYSTEM_ID, LiDAR_COMPONENT_ID, minDistanceInSector, LiDAR_SF45_MAX_DISTANCE_cm, LiDAR_SF45_MIN_DISTANCE_cm, lastReceivedSectorID);
                                //cout<<"Distance: "<<minDistanceInSector<<" cm"<<endl;
                                lastReceivedSectorID = sectorID;
                                minDistanceInSector = record.distance;
                                angleWithMinDistance = record.angle;
                            } else {
                                      if (record.distance < minDistanceInSector) {
                                          minDistanceInSector = record.distance;
                                          angleWithMinDistance = record.angle;
                                }
                            }
                        }
                    }
                }
            }
        } else {
            std::cout << "Read Queue is empty." << std::endl;
        }
    }
}

bool LidarSF45Node::SaveDataToSensorStorage(char dat)
{cout<<"inside save data func"<<endl;
  int nDat = lidar_storage.nDat;
  bool endOfMessage = false;

  // If valid data, then save it.
  if (isdigit(dat) == true || dat == '.' || dat == '-')
  {
    lidar_storage.buffer[nDat++] = dat;
  }
  // If the full message is received
  if (dat == LiDAR_MSG_POSTFIX)
  {
    lidar_storage.buffer[nDat++] = 0x0d; // add EOL

    float distance = atof(lidar_storage.buffer) * 100.0f; // m -> cm
    if (distance > 0 && distance <= LiDAR_SF45_MAX_DISTANCE_cm)
    {
      lidar_storage.sum += distance;
      lidar_storage.samples++;
    }
    cout << distance << " cm \t"<<endl;
    RCLCPP_INFO(get_logger(), "ReadDistance: %d", distance);
    mavlink_DoDistanceSensor(LiDAR_SYSTEM_ID, LiDAR_COMPONENT_ID, (float)distance, LiDAR_SF45_MAX_DISTANCE_cm, LiDAR_SF45_MIN_DISTANCE_cm, SENSOR_FORWARD);
    endOfMessage = true;
    nDat = 0;
  }

  if (nDat >= LiDAR_BUFFER_SIZE)
  {
    nDat = 0;
    // request another data in case of failure.
    endOfMessage = true;
  }

  // Update the total number of received data.
  lidar_storage.totalData++;
  lidar_storage.nDat = nDat;

  return endOfMessage;
}

void LidarSF45Node::send_command(const SF45_command& command, bool Write, uint8_t* Data, uint32_t DataSize)
{cout<<"send command function"<<endl;
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

	//serialDriver.write_data(packetBuffer);
 serialDriver.write_data(packetBuffer.data(), packetBuffer.size());
}

void LidarSF45Node::send_scan_start_command(const bool isStart)
{cout<<"scan start func"<<endl;
  if(isStart == true)
		send_command(SF45_command::ScanEnable, true, (uint8_t*)&isStart, 1);
	else
		send_command(SF45_command::ScanEnable, true, (uint8_t*)&isStart, 1);
}

void LidarSF45Node::send_scan_position_command(const float angle)
{
	send_command(SF45_command::ScanPosition, true, (uint8_t*)&angle, 4);
}

void LidarSF45Node::send_update_rate_command(const int targetHz)
{
  // Send update rate option command.
	// For specific format, refer https://support.lightware.co.za/sf45b/#/command_detail/command%20descriptions/66.%20update%20rate

  cout<<"send uprate command"<<endl;
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

void LidarSF45Node::send_output_format_command(const bool& bFirstRaw, const bool& bFirstFilter, const bool& bFirstStrength,
		const bool& bLastRaw, const bool& bLastFilter, const bool& bLastStrength,
		const bool& bBackgroundNoise, const bool& bTemperature, const bool& bYawAngle)
{
cout<<"send output format command"<<endl;
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

void LidarSF45Node::send_stream_command(const bool& isStreamOn)
{
cout<<"send stream command"<<endl;
	uint32_t parameter = 0;
	if (isStreamOn)
		parameter = 5;

	send_command(SF45_command::Stream, true, (uint8_t*)&parameter, 4);
}

void LidarSF45Node::enable_measurement(const bool& isStart)
{
  send_scan_start_command(isStart);
	send_stream_command(isStart);

	if (isStart == false)
		send_scan_position_command(0);
}

int LidarSF45Node::translate_angle_to_sector_id(const float& angle)
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



void LidarSF45Node::mavlink_DoDistanceSensor(int sysid, int componentid, float distance, float max_distance, float min_distance, int orientation)
{
  cout << "[SF45] MSG Sent to FC:\t" << distance <<" cm"<< endl;
  auto publishMsg = interfaces::msg::FcMavlinkCtrl();
  publishMsg.sysid = sysid;
  publishMsg.componentid = componentid;
  publishMsg.sysid = sysid;
  publishMsg.distance = distance;
  publishMsg.max_distance = max_distance;
  publishMsg.min_distance = min_distance;
  publishMsg.orientation = orientation;
  fcMavlinkCtrl->publish(publishMsg);
}




int main(int argc, char *argv[])
{
  //SerialDriver serialDriver; 
  bool watchdog_alarm = serialDriver.initialize(SerialPortType::CH9344, 1, BaudRate::b1152);
  if (true == watchdog_alarm)
{
  cout << "Serial Port ttyCH9344USB1 is opened successfully." << endl;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarSF45Node>();
  serialDriver.write_data('w');
  serialDriver.write_data('w');
  serialDriver.write_data('w');
  serialDriver.write_data('\r');
  serialDriver.write_data('\n');

  thread serialThread(&LidarSF45Node::communicate_serial, node, &serialDriver);
  serialThread.join();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
else
{
  cout << "Cannot open the serial port" << endl;
}
return 0;

  //rclcpp::init(argc, argv);
  //auto node = std::make_shared<LidarSF45Node>();

  //thread serialThread(&LidarSF45Node::communicate_serial, node, &serialDriver);
  //serialThread.join();
  //rclcpp::spin(node);
  //rclcpp::shutdown();
  //return 0;
}



//SF45 Parser implementation:
SF45Parser& SF45Parser::operator=(const SF45Parser& rhs)
{
    if (&rhs != this)
        copy(rhs);

    return *this;
}



void SF45Parser::copy(const SF45Parser& rhs)
{
    parsingState = rhs.parsingState;

    for(int i=0; i<2; i++)
        headerBuffer[i] = rhs.headerBuffer[i];
    
    payloadSize = rhs.payloadSize;
    payloadBuffer = rhs.payloadBuffer;
    calculatedCRC = rhs.calculatedCRC;
}



pair<SF45_command, bool> SF45Parser::pop_parsed_command()
{
    SF45_command command = SF45_command::None;
    bool isValid = false;

    if (m_parsedCommands.empty() == false)
    {
        command = m_parsedCommands.front();
        m_parsedCommands.pop_front();
        isValid = true;
    }

    return make_pair(command, isValid);
}



pair<SF45_distanceRecord, bool> SF45Parser::pop_parsed_distance_record()
{
    SF45_distanceRecord distanceRecord;
    bool isValid = false;

    if (m_parsedDistanceRecords.empty() == false)
    {
        distanceRecord = m_parsedDistanceRecords.front();
        m_parsedDistanceRecords.pop_front();
        isValid = true;
    }

    return make_pair(distanceRecord, isValid);
}



void SF45Parser::initialize_parsing_status()
{
    parsingState = SF45_parsing_state::Idle;
    
    for (int i = 0; i < 2; i++)
        headerBuffer[i] = 0;

    payloadSize = 0;
    payloadBuffer.clear();
    calculatedCRC = 0;
}



bool SF45Parser::push_data_to_parser(const uint8_t& data)
{
    bool isParsingCompleted = false;

    if (parsingState == SF45_parsing_state::Idle ||
        parsingState == SF45_parsing_state::StartCodeReceived ||
        parsingState == SF45_parsing_state::FlagLowReceived)
    {
        // PARSING STATE A: Parsing header.

        parsingState = parse_SF45_header(data, parsingState, headerBuffer);

        if (parsingState == SF45_parsing_state::FlagHighReceived)
        {
            payloadSize = (headerBuffer[0] | (headerBuffer[1] << 8)) >> 6;

            if (payloadSize < 0 || payloadSize > LiDAR_SF45_PAYLOAD_LENGTH)
            {
                initialize_parsing_status();
            }
        }
    }
    else
    {
        // PARSING STATE B: Parsing payload.

        parsingState = save_SF45_packet(data, parsingState, payloadSize, payloadBuffer);

        if (parsingState == SF45_parsing_state::PayloadReceived && calculatedCRC == 0)
        {
            calculatedCRC = calculate_crc(headerBuffer, payloadBuffer);
        }

        if (parsingState == SF45_parsing_state::CRCReceived)
        {
            size_t bufferSize = payloadBuffer.size();

            uint16_t receivedCRC = payloadBuffer.at(bufferSize - 2) | (payloadBuffer.at(bufferSize - 1) << 8);
            if (calculatedCRC == receivedCRC)
            {
                //This is the final state of the parsing.

                SF45_command command = translate_SF45_command_code(payloadBuffer.at(0));
                if (command != SF45_command::None)
                {
                    m_parsedCommands.push_back(command);
                    isParsingCompleted = true;
                }

                if (command == SF45_command::DistanceDataInCm)
                {
                    SF45_distanceRecord record = parse_SF45_distance_record(payloadBuffer);
                    
                    if (record.distance > LiDAR_SF45_MIN_DISTANCE_cm)
                        m_parsedDistanceRecords.push_back(record);
                }
            }

            initialize_parsing_status();
        }
    }

    return isParsingCompleted;
}



SF45_parsing_state SF45Parser::parse_SF45_header(const uint8_t& data, const SF45_parsing_state& parsingState, uint8_t* headerBuffer)
{
    SF45_parsing_state nextState = parsingState;

    switch (parsingState)
    {
    case SF45_parsing_state::Idle:
    {
        if (data == LiDAR_SF45_PACKET_START_BYTE)
            nextState = SF45_parsing_state::StartCodeReceived;
        break;
    }
    case SF45_parsing_state::StartCodeReceived:
        nextState = SF45_parsing_state::FlagLowReceived;
        headerBuffer[0] = data;
        break;
    case SF45_parsing_state::FlagLowReceived:
        nextState = SF45_parsing_state::FlagHighReceived;
        headerBuffer[1] = data;
        break;
    default:
        // For other cases, do nothing
        break;
    }

    return nextState; 
}



SF45_flag SF45Parser::parse_SF45_flag(const uint16_t& flagBytes)
{
    SF45_flag flag;
    flag.dataSize = flagBytes >> 6;
    flag.reserved = (flagBytes >> 1) & 0x001f;
    flag.isWrite = static_cast<bool>(flagBytes & 0x0001);

    return flag;
}



SF45_parsing_state SF45Parser::save_SF45_packet(const uint8_t& data, const SF45_parsing_state& parsingState, const uint16_t& payloadSize, vector<uint8_t>& payloadBuffer)
{
    SF45_parsing_state nextState = parsingState;

    switch (parsingState)
    {
    case SF45_parsing_state::FlagHighReceived:
    {
        payloadBuffer.clear();
        payloadBuffer.push_back(data);

        if (payloadSize > 0)
            nextState = SF45_parsing_state::PayloadReceiving;
        else
            nextState = SF45_parsing_state::PayloadReceived;

        break;
    }
    case SF45_parsing_state::PayloadReceiving:
    {
        payloadBuffer.push_back(data);
        if (payloadBuffer.size() >= payloadSize)
            nextState = SF45_parsing_state::PayloadReceived;
        break;
    }
    case SF45_parsing_state::PayloadReceived:
    {
        payloadBuffer.push_back(data);
        if (payloadBuffer.size() >= payloadSize + 2) // For CRC high/low
            nextState = SF45_parsing_state::CRCReceived; // End of packet reading
        break;
    }
    default:
        // For other cases, do nothing
        break;
    }

    return nextState;
}



SF45_distanceRecord SF45Parser::parse_SF45_distance_record(const vector<uint8_t>& payloadBuffer)
{
    SF45_distanceRecord record;
    record.distance = transform_two_uint8_into_int16(payloadBuffer.at(1), payloadBuffer.at(2));
    int rawAngle = transform_two_uint8_into_int16(payloadBuffer.at(3), payloadBuffer.at(4));
    record.angle = (float)rawAngle / 100;
    return record;
}



SF45_command SF45Parser::translate_SF45_command_code(const uint8_t& commandCode)
{
    SF45_command command = SF45_command::None;

    switch (commandCode)
    {
    case 0:
        command = SF45_command::ProductName;
        break;
    case 10:
        command = SF45_command::Token;
        break;
    case 12:
        command = SF45_command::SaveParameter;
        break;
    case 14:
        command = SF45_command::Reset;
        break;
    case 27:
        command = SF45_command::DistanceOutput;
        break;
    case 30:
        command = SF45_command::Stream;
        break;
    case 44:
        command = SF45_command::DistanceDataInCm;
        break;
    case 45:
        command = SF45_command::DistanceDataInMm;
        break;
    case 66:
        command = SF45_command::UpdateRate;
        break;
    case 79:
        command = SF45_command::BaudRate;
        break;
    case 85:
        command = SF45_command::ScanSpeed;
        break;
    case 96:
        command = SF45_command::ScanEnable;
        break;
    case 97:
        command = SF45_command::ScanPosition;
        break;
    default:
        command = SF45_command::None;
        break;
    }

    return command;
}



int16_t SF45Parser::transform_two_uint8_into_int16(const uint8_t& byte1, const uint8_t& byte2)
{
    int16_t result = 0;
    result = byte1 | (byte2 << 8);
    return result;
}



uint16_t SF45Parser::calculate_crc(const uint8_t* headerBuffer, const vector<uint8_t>& payloadBuffer)
{
    vector<uint8_t> packets(3 + payloadBuffer.size());
    packets.at(0) = 0xaa;
    for (int i = 0; i < 2; i++)
        packets.at(i + 1) = headerBuffer[i];

    int packetPosition = 3;
    for (auto& payloadPacket : payloadBuffer)
        packets.at(packetPosition++) = payloadPacket;

    uint16_t crc = calculate_crc(packets);

    return crc;
}



uint16_t SF45Parser::calculate_crc(const vector<uint8_t>& packetBuffer)
{
    uint16_t crc = 0;

    for (auto& packet : packetBuffer)
    {
        uint16_t code = crc >> 8;
        code ^= packet;
        code ^= code >> 4;
        crc = crc << 8;
        crc ^= code;
        code = code << 5;
        crc ^= code;
        code = code << 7;
        crc ^= code;
    }

    return crc;
}






