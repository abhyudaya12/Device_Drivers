#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/gc_dev_ctrl.hpp"
#include "interfaces/msg/gc_dev_resp.hpp"
#include "SerialDriver.h"

#include <string>
#include <cstring>

#include "ConstForSerialDriver.h"
using namespace std;


#define     CAMERA_TYPE_SIZE					    8
#define     ZT30_TIMEOUT_MILLISEC				    1000
//mail
#define     ZT30_MAIL_EVT_RXSIO				        0x01
#define     ZT30_GIMBAL_MAIL_SIZE			        24
//protocol format
#define     ZT30_PROTOCOL_BUFFER_SIZE		        24

#define     ZT30_PROTOCOL_STX_HIGH_BIT		        0x55
#define     ZT30_PROTOCOL_STX_LOW_BIT		        0x66
#define     ZT30_PROTOCOL_CTRL_NEED_ACK		        0x00
#define     ZT30_PROTOCOL_CTRL_ACK_PACK		        0x01

//receive
#define     ZT30_DATA_HEADER_SIZE			        8
#define     ZT30_WAIT_FOR_STX				        0
#define     ZT30_WAIT_FOR_CTRL				        1
#define     ZT30_WAIT_FOR_DATA_LENGTH		        2
#define     ZT30_WAIT_FOR_SEQ				        3
#define     ZT30_WAIT_FOR_CMD_ID			        4
#define     ZT30_WAIT_FOR_DATA				        5
#define     ZT30_WAIT_FOR_CRC				        6

//command
#define     ZT30_CMD_AUTO_FOCUS						0x04
#define     ZT30_CMD_MANUAL_ZOOM_AUTO_FOCUS			0x05
#define     ZT30_CMD_ABSOLUTE_ZOOM_AUTO_FOCUS		0x0F
#define     ZT30_CMD_GIMBAL_ROTATION				0x07
#define     ZT30_CMD_GIMBAL_CENTER					0x08
#define     ZT30_CMD_ACQUIRE_GIMBAL_INFO			0x0A
#define     ZT30_CMD_FUNCTION_FEEDBACK				0x0B
#define     ZT30_CMD_PHOTO_VIDEO					0x0C
#define     ZT30_CMD_ACQUIRE_GIMBAL_ATTITUDE		0x0D
#define     ZT30_CMD_SET_GIMBAL_CONTROL_ANGLE		0x0E
#define		ZT30_CMD_SET_IMAGE_MODE					0x11
#define		ZT30_CMD_SET_THERNMAL_PSEUDO_COLOR		0x1A

#define EO_ONLY_MODE						1
#define IR_COLOR_MODE						2
#define IR_BLACK_MODE						3
#define EO_IR_MODE							4
#define CUSTOM_MODE             5

//GImbal Mode
#define     ZT30_GIMBAL_MODE_MOUNT			        0
#define     ZT30_GIMBAL_MODE_AUTO_SCAN		        1
#define     ZT30_GIMBAL_MODE_TRACKING		        2

//angle
#define     ZT30_PITCH_MAXIMUM_ANGLE		        -360
//init
#define     ZT30_INIT_PARAM_BUFFER_SIZE		        7
#define     ZT30_INIT_PITCH_ANGLE			        -15


typedef struct {
    int pitch;
    int yaw;
} yawfollow_t;

enum Camera_Command {
    RestartCam,
    FormatCamStorage,
    SetCameraType,
    SetCameraMode,
    SetImageMode,
    SetVideoMode,
    SetImageSurveyMode,
    SetCameraPIP,
    SetCameraNight,
    GetCameraMode,
    ControlCAM,
    ChangeVideoPhoto,
    VideoOn,
    VideoOff,
    TakePhoto,
    StopPhoto,
    StartImageCapture,
    StopImageCapture,
    StartVideoCapture,
    StopVideoCapture,
    ZoomIn,
    ZoomOut,
    ZT30_StartStopRecording

};

enum Gimbal_Command {
    DoMountConfig,
    DoMountControl,
    CamTracking,
    AutoScan
};

/// <summary>
/// Ros node is made below with camera and gimbal functions for ZT30 camera
/// </summary>
class GcSerZT30Node : public rclcpp::Node
{
    SerialDriver serialDriver;

    using GcDevCtrl = interfaces::msg::GcDevCtrl;
    using GcDevResp = interfaces::msg::GcDevResp;

    rclcpp::Publisher<GcDevResp>::SharedPtr gcdevResponse;
    rclcpp::Subscription<GcDevCtrl>::SharedPtr cameraControl;
    rclcpp::Subscription<GcDevCtrl>::SharedPtr gimbalControl;

    int ser_current_roll,
        ser_current_pitch,
        ser_current_yaw,
        ser_current_zoom,
        zt30_packet_seq,
        zt30_current_zoom_level;

    yawfollow_t yaw_follow_value;

    const uint16_t siyi_crc16_tab1[256] = { 0x0,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
 0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
0x1231,0x210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
 0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
 0x2462,0x3443,0x420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
 0x3653,0x2672,0x1611,0x630,0x76d7,0x66f6,0x5695,0x46b4,
 0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
 0x48c4,0x58e5,0x6886,0x78a7,0x840,0x1861,0x2802,0x3823,
 0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
 0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0xa50,0x3a33,0x2a12,
0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0xc60,0x1c41,
0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0xe70,
0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
0x1080,0xa1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
0x2b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
0x34e2,0x24c3,0x14a0,0x481,0x7466,0x6447,0x5424,0x4405,
0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
0x26d3,0x36f2,0x691,0x16b0,0x6657,0x7676,0x4615,0x5634,
0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
0x5844,0x4865,0x7806,0x6827,0x18c0,0x8e1,0x3882,0x28a3,
0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
0x4a75,0x5a54,0x6a37,0x7a16,0xaf1,0x1ad0,0x2ab3,0x3a92,
0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0xcc1,
0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0xed1,0x1ef0
    };

    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
public:
    GcSerZT30Node();

private:
    void timer_callback1()
    {

        //gimbal_mount_control(0x00, 0x05, 0x05, A10T_CAMERA_SPEED, 0, A10T_CAMERA_SPEED, -10, A10T_CAMERA_SPEED, 10);
        //camera_video_record();
        //gimbal_mount_control(0,0);
        //rclcpp::sleep_for(std::chrono::seconds(5));
        //gimbal_mount_control(-0,0);
        //bool test= ZT30_PIP_Mode(IR_COLOR_MODE);
        //ZT30_SetImageMode(2);

        RCLCPP_INFO(get_logger(), "GcSerZT30Node Timer callback 1");
    }

    void process_camera_control_message(const GcDevCtrl::SharedPtr msg);
    void process_gimbal_control_message(const GcDevCtrl::SharedPtr msg);
    void communicate_serial(SerialDriver* pSerialDriverForGc);

    //void device_configure_init();
    //void device_format_sd_card();
    //void device_restart();

    //void camera_zoom_direct_position(int direct_value);
    void camera_take_photo();
    //void camera_video_record(int record_mode);
    void camera_video_record();
    //void switch_pic_n_rec_modes();
    void ZT30_AbsoluteZoomAndAutoFocus(int direct_value);
    void ZT30_ManualZoomAndAutoFocus(int zoom_level);
    void ZT30_TakePhoto();
    void ZT30_VideoRecord();
    void ZT30_PIP_Mode(int mode);
    int ZT30_SetImageMode(int mode);
    int ZT30_SetThermalPseudoColor(int pseudoColor);

    //void gimbal_mount_control(uint8_t roll_mode, uint8_t pitch_mode, uint8_t yaw_mode, int roll_spd, int roll_ang, int pitch_spd, int pitch_ang, int yaw_spd, int yaw_ang);
    void gimbal_mount_control(int pitch_ang, int yaw_ang);
    void gimbal_yaw_follow_handler();
    int gimbal_process_auto_scan(int* param, int idx, int timer);

    //void convert_speed(int speed, uint8_t* lowVal, uint8_t* highVal);
   // void convert_angle(int angle, uint8_t* lowVal, uint8_t* highVal, int state);
    void ZT30_GeneratePacket(char cmd_id, char len, char* data);
    void WORD2WBYTE(uint8_t WBYTE[2], uint16_t Value);
    uint16_t ZT30_CheckCRC(char* data, uint32_t len);
    uint16_t ZT30_CRC16_cal(char* ptr, uint32_t len, uint16_t crc_init);
    //uint8_t calculate_crc8(uint8_t* data, int startIdx, int endIdx);
};

/// <summary>
/// The serial port is initialized here using the constructor
/// </summary>
GcSerZT30Node::GcSerZT30Node() : Node("gcser_zt30_node")
{
    zt30_packet_seq = 0;
    zt30_current_zoom_level = 0;

    serialDriver.initialize(SerialPortType::CH9344, 7, BaudRate::b1152);

    timer1_ = create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&GcSerZT30Node::timer_callback1, this));



    // control:creating subscriptions
    cameraControl = this->create_subscription<GcDevCtrl>(
        "camera_control", 10,
        std::bind(&GcSerZT30Node::process_camera_control_message, this,
            placeholders::_1));
    gimbalControl = this->create_subscription<GcDevCtrl>(
        "gimbal_control", 10,
        std::bind(&GcSerZT30Node::process_gimbal_control_message, this,
            placeholders::_1));
    gcdevResponse = this->create_publisher<GcDevResp>("gcdev_response", 10);

    //device_configure_init();
}

void GcSerZT30Node::process_camera_control_message(const GcDevCtrl::SharedPtr msg)
{
    cout << msg->command << endl;
    switch (msg->command)
    {

    case Camera_Command::SetCameraPIP:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] SetCameraPIP");

        ZT30_PIP_Mode(msg->mode);
        break;
    }

    case Camera_Command::ZoomOut:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] Absolute Zoom: %d", msg->zoom);
        ZT30_AbsoluteZoomAndAutoFocus(msg->zoom);
        break;
    }
    case Camera_Command::ZoomIn:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] Absolute Zoom: %d", msg->zoom);
        ZT30_AbsoluteZoomAndAutoFocus(msg->zoom);
        break;
    }



    case Camera_Command::TakePhoto:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] TakePhoto");
        camera_take_photo();
        break;
    }

    case Camera_Command::StartImageCapture:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] StartImageCapture");
        camera_take_photo();
        break;
    }

    case Camera_Command::StartVideoCapture:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] StartVideoCapture");
        //camera_video_record(A10T_VIDEO_RECORD_START);
        camera_video_record();
        break;
    }
    case Camera_Command::StopVideoCapture:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] StopVideoCapture");
        camera_video_record();
        break;
    }

    case Camera_Command::SetCameraType:
    {
        RCLCPP_INFO(this->get_logger(), "[ViewPro_A10T] SetCameraType"); //keep
        ZT30_PIP_Mode(5);
        break;
    }
    case Camera_Command::SetCameraMode:
    {
        RCLCPP_INFO(this->get_logger(), "[ViewPro_A10T] SetCameraMode");
        break;
    }
    case Camera_Command::SetImageMode:
    {
        RCLCPP_INFO(this->get_logger(), "[ViewPro_A10T] SetImageMode"); //keep

        break;
    }

    case Camera_Command::ControlCAM:
    {
        RCLCPP_INFO(this->get_logger(), "[ViewPro_A10T] ControlCAM Zoom: %d", msg->mode);
        ZT30_AbsoluteZoomAndAutoFocus(msg->mode);
        break;
    }
    case Camera_Command::ChangeVideoPhoto:
    {
        RCLCPP_INFO(this->get_logger(), "[ViewPro_A10T] ChangeVideoPhoto");
        //switch_pic_n_rec_modes();
        break;
    }

    case Camera_Command::StopImageCapture:
    {
        RCLCPP_INFO(this->get_logger(), "[ViewPro_A10T] StopImageCapture"); //keep
        gimbal_mount_control(0, 0);
        break;
    }
    }

}

void GcSerZT30Node::process_gimbal_control_message(const GcDevCtrl::SharedPtr msg)
{
    cout << msg->command << endl;
    ZT30_AbsoluteZoomAndAutoFocus(1);
    switch (msg->command)
    {
    case Gimbal_Command::DoMountConfig:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] DoMountConfig");
        //device_configure_init();
        break;
    }
    case Gimbal_Command::DoMountControl:
    {
        int pitch = msg->in_pitch;
        int yaw = msg->in_yaw;
        yaw_follow_value.pitch = pitch;
        yaw_follow_value.yaw = yaw;

        RCLCPP_INFO(this->get_logger(), "[SIYI] DoMountControl | roll: 0, pitch: %d, yaw: %d", pitch, yaw);
        //gimbal_mount_control(0x00, 0x05, 0x05, A10T_CAMERA_SPEED, 0, A10T_CAMERA_SPEED, (-pitch), A10T_CAMERA_SPEED, yaw);
        gimbal_mount_control(pitch, yaw);
        rclcpp::sleep_for(std::chrono::seconds(1));
        gimbal_yaw_follow_handler();
        break;
    }
    /*case Gimbal_Command::CamTracking:
    {
        RCLCPP_INFO(this->get_logger(), "[ViewPro_A10T] CamTracking");
        break;
    }
    case Gimbal_Command::AutoScan:
    {
        RCLCPP_INFO(this->get_logger(), "[ViewPro_A10T] AutoScan");
        break;
    }*/
    }
}

void GcSerZT30Node::communicate_serial(SerialDriver* pSerialDriverForGc)
{
}


void GcSerZT30Node::gimbal_yaw_follow_handler()
{
    gimbal_mount_control((yaw_follow_value.pitch), yaw_follow_value.yaw);
}


void  GcSerZT30Node::ZT30_AbsoluteZoomAndAutoFocus(int direct_value)
{
    char zoom_direct_cmd[2];

    zoom_direct_cmd[0] = direct_value;
    zoom_direct_cmd[1] = 5;

    ser_current_zoom = direct_value;

    ZT30_GeneratePacket(ZT30_CMD_ABSOLUTE_ZOOM_AUTO_FOCUS, 2, zoom_direct_cmd);



}


void GcSerZT30Node::ZT30_ManualZoomAndAutoFocus(int zoom_level)
{
    char manual_zoom[1];
    int set_zoom_level = 0;

    if (zt30_current_zoom_level < zoom_level)
    {
        set_zoom_level = zoom_level - zt30_current_zoom_level;
        manual_zoom[0] = 0x01;
    }
    else if (zt30_current_zoom_level > zoom_level)
    {
        set_zoom_level = zt30_current_zoom_level - zoom_level;
        manual_zoom[0] = 0xff;
    }
    zt30_current_zoom_level = zoom_level;


    for (int i = 0; i < set_zoom_level; i++)
    {
        ZT30_GeneratePacket(ZT30_CMD_MANUAL_ZOOM_AUTO_FOCUS, 1, manual_zoom);
    }
}

void GcSerZT30Node::camera_take_photo()
{
    char take_photo[1];

    take_photo[0] = TakePhoto;

    ZT30_GeneratePacket(ZT30_CMD_PHOTO_VIDEO, 1, take_photo);
}

void GcSerZT30Node::camera_video_record()
{
    char video_record[1];

    video_record[0] = ZT30_StartStopRecording;

    ZT30_GeneratePacket(ZT30_CMD_PHOTO_VIDEO, 1, video_record);
}

void GcSerZT30Node::gimbal_mount_control(int pitch_ang, int yaw_ang)
{
    unsigned char mount_control[4];

    int convert_yaw = yaw_ang * -10;
    int convert_pitch = pitch_ang * 10;

    mount_control[0] = convert_yaw & 0xFF;
    mount_control[1] = (convert_yaw >> 8) & 0xFF;
    mount_control[2] = convert_pitch & 0xFF;
    mount_control[3] = (convert_pitch >> 8) & 0xFF;

    ser_current_yaw = yaw_ang;
    ser_current_pitch = pitch_ang;

    ZT30_GeneratePacket(ZT30_CMD_SET_GIMBAL_CONTROL_ANGLE, 4, (char*)mount_control);
}


void GcSerZT30Node::ZT30_PIP_Mode(int mode)
{
    bool success = false;
    char chmode = -1, pseudoColor = -1;

    switch (mode)
    {
    case EO_ONLY_MODE:
    {
        chmode = 0;
        success = true;
        break;
    }
    case IR_COLOR_MODE:
    {
        chmode = 7;
        pseudoColor = 4;
        success = true;
        break;
    }
    case IR_BLACK_MODE:
    {
        chmode = 7;
        pseudoColor = 0;
        success = true;
        break;
    }
    case EO_IR_MODE:
    {
        chmode = 2;
        pseudoColor = 4;
        success = true;
        break;
    }
    case CUSTOM_MODE:
    {
        chmode = 0;
        //pseudoColor = 6;
        success = true;
        break;
    }
    }

    if (success == true)
    {
        if (chmode >= 0)
        {
            success = ZT30_SetImageMode(chmode) == 1 ? true : false;
        }
        //rclcpp::sleep_for(std::chrono::seconds(1));
        if (pseudoColor >= 0)
        {
            success = ZT30_SetThermalPseudoColor(pseudoColor) == 1 ? true : false;
        }
    }

    //return success;


}

int GcSerZT30Node::ZT30_SetImageMode(int mode)
{
    int ret = -1;
    char chMode = (char)mode;

    if (chMode >= 0 && chMode <= 8)
    {
        ZT30_GeneratePacket(ZT30_CMD_SET_IMAGE_MODE, 1, (char*)&chMode);
        ret = 1;
    }

    return ret;
}

int GcSerZT30Node::ZT30_SetThermalPseudoColor(int pseudoColor)
{
    int ret = -1;
    char chMode = pseudoColor;

    if (chMode >= 0 && chMode <= 11)
    {
        ZT30_GeneratePacket(ZT30_CMD_SET_THERNMAL_PSEUDO_COLOR, 1, (char*)&chMode);
    }

    return ret;
}

uint16_t GcSerZT30Node::ZT30_CRC16_cal(char* ptr, uint32_t len, uint16_t crc_init)
{
    uint16_t crc, oldcrc16;
    char temp;
    crc = crc_init;
    while (len-- != 0)
    {
        temp = (crc >> 8) & 0xff;
        oldcrc16 = siyi_crc16_tab1[*ptr ^ temp];
        crc = (crc << 8) ^ oldcrc16;
        ptr++;
    }

    return(crc);
}
uint16_t GcSerZT30Node::ZT30_CheckCRC(char* data, uint32_t len)
{
    uint16_t crc_result = 0;
    crc_result = ZT30_CRC16_cal(data, len, 0);

    return crc_result;
}






void GcSerZT30Node::ZT30_GeneratePacket(char cmd_id, char len, char* data)
{
    int idx = 0;
    char packet[ZT30_PROTOCOL_BUFFER_SIZE];

    packet[0] = ZT30_PROTOCOL_STX_HIGH_BIT;
    packet[1] = ZT30_PROTOCOL_STX_LOW_BIT;
    packet[2] = ZT30_PROTOCOL_CTRL_ACK_PACK;
    packet[3] = len;
    packet[4] = 0;
    packet[5] = zt30_packet_seq & 0xFF;
    packet[6] = (zt30_packet_seq & 0xFF00) >> 8;
    packet[7] = cmd_id;

    for (idx = ZT30_DATA_HEADER_SIZE; idx < (len + ZT30_DATA_HEADER_SIZE); idx++)
    {
        packet[idx] = *data++;
    }

    uint16_t crc = ZT30_CheckCRC(packet, ZT30_DATA_HEADER_SIZE + len);

    packet[idx++] = crc & 0xFF;
    packet[idx++] = (crc & 0xFF00) >> 8;

    zt30_packet_seq++;

    //ZT30_SIO_Write(packet, idx);
    serialDriver.write_data(reinterpret_cast<const uint8_t*>(packet), idx);
}





int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GcSerZT30Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}