#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/gc_dev_ctrl.hpp"
#include "interfaces/msg/gc_dev_resp.hpp"
#include <string>
#include <cstring>

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/gimbal/gimbal.h>
#include <mavsdk/plugins/camera/camera.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

#define EO_ONLY_MODE						1
#define IR_COLOR_MODE						2
#define IR_BLACK_MODE						3
#define EO_IR_MODE							4
#define CUSTOM_MODE             5

//Gimbal Mode
#define     R_GIMBAL_MODE_MOUNT			        0
#define     R_GIMBAL_MODE_AUTO_SCAN		        1
#define     R_GIMBAL_MODE_TRACKING		        2

using namespace mavsdk;
using namespace std;
using std::chrono::seconds;
using std::this_thread::sleep_for;

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
    StartStopRecording

};

enum Gimbal_Command {
    DoMountConfig,
    DoMountControl,
    CamTracking,
    AutoScan
};

/// <summary>
/// Ros node is made below with camera and gimbal functions for Rhythm series camera
/// </summary>

class GcRNode : public rclcpp::Node
{

    using GcDevCtrl = interfaces::msg::GcDevCtrl;
    using GcDevResp = interfaces::msg::GcDevResp;

    rclcpp::Publisher<GcDevResp>::SharedPtr gcdevResponse;
    rclcpp::Subscription<GcDevCtrl>::SharedPtr cameraControl;
    rclcpp::Subscription<GcDevCtrl>::SharedPtr gimbalControl;

    int current_roll,
        current_pitch,
        current_yaw,
        current_zoom,
        current_zoom_level;

    yawfollow_t yaw_follow_value;

    

    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
public:
    GcRNode();

private:
    void timer_callback1()
    {
        #if false
        //test functions here
        
        #endif
        
        RCLCPP_INFO(get_logger(), "GcRNode Timer callback 1");  
    }

    void process_camera_control_message(const GcDevCtrl::SharedPtr msg);
    void process_gimbal_control_message(const GcDevCtrl::SharedPtr msg);
    void camera_zoom_direct_position(int direct_value);
    void camera_take_photo();
    //void camera_video_record(int record_mode);
    void camera_video_record();
    //void switch_pic_n_rec_modes();
    void AbsoluteZoomAndAutoFocus(int direct_value);
    void ManualZoomAndAutoFocus(int zoom_level);
    void TakePhoto();
    void VideoRecord();
    //void PIP_Mode(int mode);
    int SetImageMode(int mode);
   
    void gimbal_mount_control(int pitch_ang, int yaw_ang);
    void gimbal_yaw_follow_handler();
    void gimbal_control(float yaw, float pitch);
    
    
    //MavSDK related code:
      Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
     std::shared_ptr<mavsdk::System> system = nullptr;
     std::unique_ptr<mavsdk::Gimbal> gimbal = nullptr;
     std::unique_ptr<mavsdk::Camera> camera = nullptr;
    ConnectionResult connection_result;
 
};

GcRNode::GcRNode() : Node("GcR_node")
{
    current_zoom_level = 0;

    timer1_ = create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&GcRNode::timer_callback1, this));


    // control:creating subscriptions
    cameraControl = this->create_subscription<GcDevCtrl>(
        "camera_control", 10,
        std::bind(&GcRNode::process_camera_control_message, this,
            placeholders::_1));
    gimbalControl = this->create_subscription<GcDevCtrl>(
        "gimbal_control", 10,
        std::bind(&GcRNode::process_gimbal_control_message, this,
            placeholders::_1));
    gcdevResponse = this->create_publisher<GcDevResp>("gcdev_response", 10);
    
    //MavSDK related code:
    connection_result = mavsdk.add_any_connection("udp://192.168.2.119:14551");
    //connection_result = mavsdk.add_any_connection("serial:///dev/ttyCH9344USB7:115200");

        if (connection_result != ConnectionResult::Success) {
            std::cerr << "Connection failed: " << connection_result << '\n';
            return;
        }

        std::cout << "Waiting to discover system...\n";
        std::promise<std::shared_ptr<mavsdk::System>> prom;
        std::future<std::shared_ptr<mavsdk::System>> fut = prom.get_future();

        Mavsdk::NewSystemHandle handle = mavsdk.subscribe_on_new_system([&mavsdk = mavsdk, &prom, &handle]() {
        std::shared_ptr<mavsdk::System> system_discovered = mavsdk.systems().back();

            if (system_discovered->has_gimbal() && system_discovered->has_camera()) {
                std::cout << "Discovered Gimbal and Camera\n";
                mavsdk.unsubscribe_on_new_system(handle);
                prom.set_value(system_discovered);
            }
        });

        if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
            std::cerr << "No gimbal found, exiting.\n";
            return;
        }

        system = fut.get();
        gimbal = std::make_unique<mavsdk::Gimbal>(system);
        camera = std::make_unique<mavsdk::Camera>(system);
}


void GcRNode::process_camera_control_message(const GcDevCtrl::SharedPtr msg)
{
    cout << msg->command << endl;
    switch (msg->command)
    {

    case Camera_Command::SetCameraPIP:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] SetCameraPIP");

        //ZT30_PIP_Mode(msg->mode);
        break;
    }

    case Camera_Command::ZoomOut:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] Absolute Zoom: %d", msg->zoom);
        //AbsoluteZoomAndAutoFocus(msg->zoom);
        break;
    }
    case Camera_Command::ZoomIn:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] Absolute Zoom: %d", msg->zoom);
        //AbsoluteZoomAndAutoFocus(msg->zoom);

        break;
    }

    case Camera_Command::TakePhoto:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] TakePhoto");
        break;
    }

    case Camera_Command::StartImageCapture:
    {
        RCLCPP_INFO(this->get_logger(), "[Rhythm] StartImageCapture");
        
        auto mode_result = camera->set_mode(Camera::Mode::Photo);
    if (mode_result != Camera::Result::Success) {
        std::cerr << "Could not switch to Photo mode: " << mode_result;
        return;
    }
        auto photo_result = camera->take_photo();
    if (photo_result != Camera::Result::Success) {
        std::cerr << "Taking Photo failed: " << photo_result;
        return;
    }

    // Wait a bit to make sure we see capture information.
    sleep_for(seconds(2));
        break;
    }

    case Camera_Command::StartVideoCapture:
    {
        RCLCPP_INFO(this->get_logger(), "[Rhythm] StartVideoCapture");
        
        const auto mode_result = camera->set_mode(Camera::Mode::Video);
    if (mode_result != Camera::Result::Success) {
        std::cerr << "Could not switch to Video mode: " << mode_result;
        return;
    }
        const auto photo_result = camera->start_video();
    if (photo_result != Camera::Result::Success) {
        std::cerr << "Taking video failed: " << photo_result;
        return;
    }
    
        break;
    }
    case Camera_Command::StopVideoCapture:
    {
        RCLCPP_INFO(this->get_logger(), "[Rhythm] StopVideoCapture");
        const auto mode_result = camera->set_mode(Camera::Mode::Video);
    if (mode_result != Camera::Result::Success) {
        std::cerr << "Could not switch to Video mode: " << mode_result;
        return;
    }
        const auto photo_result = camera->stop_video();
    if (photo_result != Camera::Result::Success) {
        std::cerr << "stopping video failed: " << photo_result;
        return;
    }
    // Wait a bit to make sure we see capture information.
    sleep_for(seconds(1));
        break;
    }

    case Camera_Command::SetCameraType:
    {
        RCLCPP_INFO(this->get_logger(), "[SIYI] SetCameraType"); //keep
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
        RCLCPP_INFO(this->get_logger(), "[Rhythm] ControlCAM Zoom: %d", msg->mode);
        auto result = camera->zoom_range(float(msg->mode));
    if (result != Camera::Result::Success) {
        std::cerr << "Zooming failed: " << result << '\n';
        return;
    }   
        
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
        RCLCPP_INFO(this->get_logger(), "[Rhythm] StopImageCapture"); //keep
       
       auto result = camera->zoom_out_start();
    if (result != Camera::Result::Success) {
        std::cerr << "Zooming out failed: " << result << '\n';
        return;
    }
    
    //reset camera settigs:
    //result = camera->reset_settings();
    //result = camera->format_storage(11);
    //std::cout << "Reset camera settings result : " << result << std::endl;
    
        break;
    }
    }

}

void GcRNode::gimbal_control(float pitch, float yaw){

std::cout << "Start controlling gimbal...\n";
 if (!gimbal) {
        std::cerr << "Gimbal not initialized.\n";
        
    }
        Gimbal::Result gimbal_result = gimbal->take_control(Gimbal::ControlMode::Primary);
        if (gimbal_result != Gimbal::Result::Success) {
            std::cerr << "Could not take gimbal control: " << gimbal_result << '\n';
        }

        gimbal_result = gimbal->set_mode(Gimbal::GimbalMode::YawFollow);
        if (gimbal_result != Gimbal::Result::Success) {
            std::cerr << "Could not set to follow mode: " << gimbal_result << '\n';
        }

        gimbal->set_pitch_and_yaw(pitch, yaw);

        std::cout << "Stop controlling gimbal...\n";
        gimbal_result = gimbal->release_control();
        if (gimbal_result != Gimbal::Result::Success) {
            std::cerr << "Could not release gimbal control: " << gimbal_result << '\n';
        }
}


void GcRNode::process_gimbal_control_message(const GcDevCtrl::SharedPtr msg)
{

    cout << msg->command << endl;
    switch (msg->command)
    {
    case Gimbal_Command::DoMountConfig:
    {
        RCLCPP_INFO(this->get_logger(), "[R] DoMountConfig");
        
        break;
    }
    case Gimbal_Command::DoMountControl:
    {
        int pitch = msg->in_pitch;
        int yaw = msg->in_yaw;
        yaw_follow_value.pitch = pitch;
        yaw_follow_value.yaw = yaw;

        RCLCPP_INFO(this->get_logger(), "[Rhythm] DoMountControl | roll: 0, pitch: %d, yaw: %d", pitch, yaw);

        gimbal_control(pitch, yaw);
        
        break;
    }
    
    }
    
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GcRNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




