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
#include <mavsdk/plugins/param/param.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

#include <vector>
#include <sstream>
#include <iomanip>

#define EO_ONLY_MODE						1
#define IR_COLOR_MODE						2
#define IR_BLACK_MODE						3
#define EO_IR_MODE							4
#define CUSTOM_MODE             5
#define DEBUG                   0

using namespace mavsdk;
using namespace std;
using std::chrono::seconds;
using std::chrono::milliseconds;
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
    StartStopRecording,
    SetTrackingModel,
    SetDetectionModel,
    StartTracking,
    StopTracking,
    GetDetection,
    SetBitrate,
    SetResolution,
    SetIRMode
    

};

enum Gimbal_Command {
    DoMountConfig,
    DoMountControl,
    CamTracking,
    AutoScan
};

struct DetectObject {
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    uint16_t score;
    uint16_t type;
};

struct DetectContent {
    uint16_t index;
    uint16_t size;
    uint16_t total;
    std::vector<DetectObject> objects;
};

/// <summary>
/// Ros node below with camera, gimbal and AI functions for Rhythm series camera. ~Abhy 2024.10.25
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
        
        //RCLCPP_INFO(get_logger(), "GcRNode Timer callback 1");  
    }

    void process_camera_control_message(const GcDevCtrl::SharedPtr msg);
    void process_gimbal_control_message(const GcDevCtrl::SharedPtr msg);
    void gimbal_control(float yaw, float pitch); //Gimbal
    void start_detection(std::string model); //AI. This function for command setDetectionModel/= start detection.
    void start_tracking(float top_left_corner_x, float top_left_corner_y, float bottom_right_corner_x, float bottom_right_corner_y); //AI
    void stop_ai(); //Stop AI detection and tracking
    void get_detection_info();//AI
    void set_IR_mode(int view); //type of view for IR cam
    void setBitrate(float rate); //change bitrate of cam
    void setTrackingModel(std::string model);//set tracking model
    void setResolution(int view);//set resolution
    void paramTestFunction(); // For testing
    
    //MavSDK related code:
      Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
     std::shared_ptr<mavsdk::System> system = nullptr;
     std::unique_ptr<mavsdk::Gimbal> gimbal = nullptr;
     std::unique_ptr<mavsdk::Camera> camera = nullptr;
     std::shared_ptr<mavsdk::Param> param = nullptr;
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
            std::cerr << "No camera and gimbal found, exiting.\n";
            return;
        }

        system = fut.get();
        gimbal = std::make_unique<mavsdk::Gimbal>(system);
        camera = std::make_unique<mavsdk::Camera>(system);
        param = std::make_shared<mavsdk::Param>(system);
        
        //Setting bitrate to 1.5
        setBitrate(1.5f);
}

void GcRNode::setBitrate(float rate){
param->select_component(100,Param::ProtocolVersion::Ext); //100 is the component ID of camera.
std::string param_to_set = "EO_BITRATE";
std::string param_to_get = "EO_BITRATE";
float variable = rate;
        auto set_result = param->set_param_float(param_to_set,variable);
        if (set_result == Param::Result::Success) 
        std::cout << "Successfully set parameter: "<< param_to_set <<std::endl;
        
        
        auto get_result = param->get_param_float(param_to_get);
        if (get_result.first == Param::Result::Success) 
        std::cout << "Successfully get parameter: "<< param_to_get << " value: " << get_result.second <<std::endl;


}

void GcRNode::setResolution(int view){
param->select_component(100,Param::ProtocolVersion::Ext); //100 is the component ID of camera
        std::string param_to_set = "EO_VIDEO_QUALITY";
        auto set_result = param->set_param_int(param_to_set,view);
        if (set_result == Param::Result::Success) {
        std::cout << "Successfully set parameter: "<< param_to_set <<std::endl;
  }
}


//testing function:
void GcRNode::paramTestFunction(){
#if DEBUG
param->select_component(100,Param::ProtocolVersion::Ext); //100 is the component ID of camera.
std::string param_to_set = "EO_BITRATE";
std::string param_to_get = "EO_BITRATE";
float variable = 1.0f;
        //auto set_result = param->set_param_float(param_to_set,variable);
        //if (set_result == Param::Result::Success) 
        //std::cout << "Successfully set parameter: "<< param_to_set <<std::endl;
        
        
        auto get_result = param->get_param_float(param_to_get);
        if (get_result.first == Param::Result::Success) 
        std::cout << "Successfully get parameter: "<< param_to_get << " value: " << get_result.second <<std::endl;
#endif
}

//parsing detection raw values
DetectContent parse_detection_data(const std::vector<uint8_t>& data) {
    DetectContent detect_content;
    size_t offset = 0;

    auto read_uint16 = [&data, &offset]() -> uint16_t {
        uint16_t value = data[offset] | (data[offset + 1] << 8);
        offset += 2;
        return value;
    };

    detect_content.index = read_uint16();
    detect_content.size = read_uint16();
    detect_content.total = read_uint16();

    for (size_t i = 0; i < 10 && offset + 12 <= data.size(); ++i) {
        DetectObject obj;
        obj.x = read_uint16();
        obj.y = read_uint16();
        obj.width = read_uint16();
        obj.height = read_uint16();
        obj.score = read_uint16();
        obj.type = read_uint16();
        detect_content.objects.push_back(obj);
    }

    return detect_content;
}

void GcRNode::get_detection_info(){
param->select_component(100,Param::ProtocolVersion::Ext); //100 is the component ID of camera.
std::string param_to_get = "DETECT_OBJECTS";
std::string objects="{\"Detected_Objects\":";
 auto get_result = param->get_param_custom(param_to_get);
        if (get_result.first == Param::Result::Success) {
        std::string result_string(get_result.second.begin(), get_result.second.end());
        
        std::cout << "Successfully got parameter: " << param_to_get << std::endl;
        
        if(!get_result.second.empty()){
        // Convert the std::string to std::vector<uint8_t>
        std::vector<uint8_t> result_vector(get_result.second.begin(), get_result.second.end());
         DetectContent detect_content = parse_detection_data(result_vector);

    std::cout << "Index: " << detect_content.index << std::endl;
    std::cout << "Size: " << detect_content.size-1 << std::endl;
    std::cout << "Total: " << detect_content.total-1 << std::endl;

    for (const auto& obj : detect_content.objects) {  
            
            std::cout << "Object " << " - X: " << obj.x/10000.0f << ", Y: " << obj.y/10000.0f
                      << ", Width: " << obj.width/10000.0f << ", Height: " << obj.height/10000.0f
                      << ", Score: " << obj.score/100.0f << ", Type: " << obj.type << std::endl;
                      
                      objects += std::to_string(obj.x / 10000.0f) + "," +
                         std::to_string(obj.y / 10000.0f) + "," +
                         std::to_string(obj.width / 10000.0f) + "," +
                         std::to_string(obj.height / 10000.0f) + "," +
                         std::to_string(obj.score / 100.0f) + ","+ 
                         std::to_string(obj.type) + ";";
                      
        }
        
        if(detect_content.total-1 == 0){objects.clear();}
        if(objects.empty()==false){
        auto publishMsg = interfaces::msg::GcDevResp();
        publishMsg.result = 1; //1 is for detection result. 2 for tracking result.
        objects+="}";
        publishMsg.response = objects;
        gcdevResponse->publish(publishMsg);
        objects.clear();
        }
        
    }
    
    } else {
        std::cerr << "Failed to get parameter: " << param_to_get << std::endl;
    }
    
}
void GcRNode::start_detection(std::string model){

     std::string param_to_get = "SMART_SELECT";
     std::string param_to_set = "SMART_SELECT";

     std::string detection_algo = model; //Yolov8, Yolov7.


param->select_component(100,Param::ProtocolVersion::Ext); //100 is the component ID of camera.
        
        std::cout << "Successfully set component to Camera" <<std::endl;
        
    
        
        
        auto set_result = param->set_param_custom(param_to_set,detection_algo);
        if (set_result == Param::Result::Success) {
        std::cout << "Successfully set parameter: "<< param_to_set << " to " << detection_algo <<std::endl;
        
    }
        
        
        auto get_result = param->get_param_custom(param_to_get);
        if (get_result.first == Param::Result::Success) {
        std::cout << "Successfully got parameter: " << param_to_get << " value: " << get_result.second << std::endl;
        
        
        
    } else {
        std::cerr << "Failed to get parameter: " << param_to_get << std::endl;
    }

}

void GcRNode::start_tracking(float top_left_corner_x, float top_left_corner_y, float bottom_right_corner_x, float bottom_right_corner_y){
     
param->select_component(100,Param::ProtocolVersion::Ext); //100 is the component ID of camera.
    
    /*
    float point_x = 0.5f; // Middle of the X axis (normalized)
    float point_y = 0.5f; // Middle of the Y axis (normalized)
    float radius = 0.02f;  //radius
    
    //auto result = camera->track_point(point_x, point_y, radius);
    
    
    float top_left_corner_x = 0.323f;
    float top_left_corner_y = 0.25f;
    float bottom_right_corner_x = 0.677f;
    float bottom_right_corner_y = 0.75f;
 

*/    
    auto result = camera->track_rectangle(top_left_corner_x, top_left_corner_y,bottom_right_corner_x, bottom_right_corner_y);


    if (result != Camera::Result::Success) {
        std::cerr << "Failed to start tracking: " << std::endl;
    } else {
        std::cout << "Tracking started successfully" << std::endl;
    }

}

void GcRNode::setTrackingModel(std::string model){

     std::string param_to_get = "TRACK_ALGORITHM";
     std::string param_to_set = "TRACK_ALGORITHM";
    
     std::string tracking_algo = model; //"SiamRPN", "Nano"

param->select_component(100,Param::ProtocolVersion::Ext); //100 is the component ID of camera.
        
           
        auto set_result = param->set_param_custom(param_to_set,tracking_algo);
        if (set_result == Param::Result::Success) {
        std::cout << "Successfully set parameter: "<< param_to_set << " to " << tracking_algo <<std::endl;
        
    }
        
        
        auto get_result = param->get_param_custom(param_to_get);
        if (get_result.first == Param::Result::Success) {
        std::cout << "Successfully got parameter: " << param_to_get << " value: " << get_result.second << std::endl;
        
        
    } else {
        std::cerr << "Failed to get parameter: " << param_to_get << std::endl;
    }
    

}

void GcRNode::stop_ai(){ //stops tracking and resets detection and tracking models to "None".

std::string param_to_set1 = "TRACK_ALGORITHM";
std::string param_to_set2 = "SMART_SELECT";
param->select_component(100,Param::ProtocolVersion::Ext); //100 is the component ID of camera.

auto result = camera->track_stop();
    

    if (result != Camera::Result::Success) {
        std::cerr << "Failed to stop tracking: " << std::endl;
    } else {
        std::cout << "Tracking stopped successfully" << std::endl;
    }
    
    auto set_result = param->set_param_custom(param_to_set1,"None");
        if (set_result == Param::Result::Success) {
        std::cout << "Successfully set parameter: "<< param_to_set1 << " to " << "None" <<std::endl;
        
    }
    
     set_result = param->set_param_custom(param_to_set2,"None");
        if (set_result == Param::Result::Success) {
        std::cout << "Successfully set parameter: "<< param_to_set2 << " to " << "None" <<std::endl;
        
    }
       
    
}

void GcRNode::process_camera_control_message(const GcDevCtrl::SharedPtr msg)
{    
    cout << msg->command << endl;
    switch (msg->command)
    {

    case Camera_Command::SetCameraPIP:
    {
        RCLCPP_INFO(this->get_logger(), "SetCameraPIP");

        //ZT30_PIP_Mode(msg->mode);
        break;
    }

    case Camera_Command::ZoomOut:
    {
        RCLCPP_INFO(this->get_logger(), "Absolute Zoom: %d", msg->zoom);
        //AbsoluteZoomAndAutoFocus(msg->zoom);
        break;
    }
    case Camera_Command::ZoomIn:
    {
        RCLCPP_INFO(this->get_logger(), "Absolute Zoom: %d", msg->zoom);
        //AbsoluteZoomAndAutoFocus(msg->zoom);

        break;
    }

    case Camera_Command::TakePhoto:
    {
        RCLCPP_INFO(this->get_logger(), "TakePhoto");
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
    sleep_for(seconds(1));
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
        RCLCPP_INFO(this->get_logger(), "SetCameraType"); //keep
        break;
    }
    case Camera_Command::SetCameraMode:
    {
        RCLCPP_INFO(this->get_logger(), "SetCameraMode");
        break;
    }
    case Camera_Command::SetImageMode:
    {
        RCLCPP_INFO(this->get_logger(), "SetImageMode"); //when we open cam controller from ALES
        
        start_detection("Yolov8");
        //start_tracking();
        //int t=10;
        while(true){
        sleep_for(seconds(1));
        get_detection_info();
        //start_tracking();
        //t--;
        }
        //paramTestFunction();
        
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
        stop_ai();
        break;
    }
    case Camera_Command::ChangeVideoPhoto:
    {
        RCLCPP_INFO(this->get_logger(), " ChangeVideoPhoto");
        
        break;
    }

    case Camera_Command::StopImageCapture:
    {
        RCLCPP_INFO(this->get_logger(), "[Rhythm] StopImageCapture"); //keep
        break;
    }
    
    case Camera_Command::FormatCamStorage:
    {
        RCLCPP_INFO(this->get_logger(), "[Rhythm] Format camera storage"); //keep
       
       auto result = camera->format_storage(11);
    if (result != Camera::Result::Success) {
        std::cerr << "Formatting failed: " << result << '\n';
        return;
      }
    
        break;
    }
    
    case Camera_Command::SetIRMode:
    {
        RCLCPP_INFO(this->get_logger(), "[Rhythm] IR Mode change"); //keep
       
       
      set_IR_mode(msg->view);
    
        break;
    }
    
    case Camera_Command::SetResolution:
    {
        RCLCPP_INFO(this->get_logger(), "[Rhythm] Resolution change"); //keep
       
       
      setResolution(msg->view);
    
        break;
    }
    
    case Camera_Command::SetBitrate:
    {
        RCLCPP_INFO(this->get_logger(), "[Rhythm] bitrate change"); //keep
       
       
      setBitrate(msg->rate);
    
        break;
    }
    
    case Camera_Command::GetDetection:
    {
    RCLCPP_INFO(this->get_logger(), "[Rhythm] Get Detection"); //keep
    get_detection_info();
    break;
    }
    case Camera_Command::SetDetectionModel:
    {
        RCLCPP_INFO(this->get_logger(), "[Rhythm] Set Detection"); //keep
       
       
        start_detection(msg->model);
    
        break;
    }
    
    case Camera_Command::SetTrackingModel:
    {
        RCLCPP_INFO(this->get_logger(), "[Rhythm] Set tracking"); //keep
       
       
      setTrackingModel(msg->model);
    
        break;
    }
    case Camera_Command::StartTracking:
    {
        RCLCPP_INFO(this->get_logger(), "[Rhythm] start tracking"); //keep
       
       
      start_tracking(msg->top_left_corner_x, msg->top_left_corner_y, msg->bottom_right_corner_x, msg->bottom_right_corner_y);
    
        break;
    }
    case Camera_Command::StopTracking:
    {
        RCLCPP_INFO(this->get_logger(), "[Rhythm] stop tracking"); //keep
       
       
      stop_ai();
    
        break;
    }
    
 }

}

void GcRNode::set_IR_mode(int view){
param->select_component(100,Param::ProtocolVersion::Ext); //100 is the component ID of camera
        std::string param_to_set = "IR_PALETTE";
        auto set_result = param->set_param_int(param_to_set,view);
        if (set_result == Param::Result::Success) {
        std::cout << "Successfully set parameter: "<< param_to_set <<std::endl;

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




