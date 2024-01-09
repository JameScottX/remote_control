/**
 * @file jot_stick.cpp
 * @author Junwen Cui/jun_wencui@126.com
 * @brief 
 * @version 0.1
 * @date 2023-12-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "remote_stick/joy_drive.h"
#include <thread>
#include "signal.h"
#include <mutex> 

#define GAP_CTL 2000

using namespace std::chrono_literals;

std::mutex mtx;
int xbox_fd ;  
xbox_map_t map;  
int len;  
bool joy_get_flag = true;
std_msgs::msg::Float32MultiArray msg_out;

inline double math_limit(double val, double min_, double max_){
    // limited function
    double val_out = val;
    if(val_out > max_){
        return max_;
    }else if(val_out < min_){
        return min_;
    }
    return val;
}

double math_map(double val, double min_, double max_, double min_tar, double max_tar){
    // linear map function
    double val_new = math_limit(val, min_, max_);
    double rat = (max_tar - min_tar) / (max_ - min_);
    return (val_new - min_) * rat + min_tar;
}

void joy_get(void) {
    
    memset(&map, 0, sizeof(xbox_map_t)); 
    // set the initial value
    map.lt = -32767;
    map.rt = -32767; 

    xbox_fd = xbox_open("/dev/input/js0");  
    if(xbox_fd < 0){  
        printf("Xbox open failed!\n");
        raise(SIGINT);
        return ;  
    } 
    
    while(joy_get_flag){  
        len = xbox_map_read(xbox_fd, &map);  
        if (len < 0){  
            usleep(10*1000);  
            continue;  
        }  

        // printf("\rTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d start:%d back:%d home:%d LO:%d RO:%d XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",  
        //         map.time, map.a, map.b, map.x, map.y, map.lb, map.rb, map.start, map.back, map.home, map.lo, map.ro,  
        //         map.xx, map.yy, map.lx, map.ly, map.rx, map.ry, map.lt, map.rt);  
        
        mtx.lock();

        msg_out.data[0] =  math_map((double)-map.lx, -32767.0 + GAP_CTL, 32767.0 - GAP_CTL, -1.0, 1.0);
        msg_out.data[1] =  math_map((double)-map.ly, -32767.0 + GAP_CTL, 32767.0 - GAP_CTL, -1.0, 1.0);
        msg_out.data[2] =  math_map((double)-map.rx, -32767.0 + GAP_CTL, 32767.0 - GAP_CTL, -1.0, 1.0);
        msg_out.data[3] =  math_map((double)-map.ry, -32767.0 + GAP_CTL, 32767.0 - GAP_CTL, -1.0, 1.0);
        msg_out.data[4] =  math_map((double)map.lt, -32767.0 + GAP_CTL, 32767.0 - GAP_CTL, 0.0, 1.0);
        msg_out.data[5] =  math_map((double)map.rt, -32767.0 + GAP_CTL, 32767.0 - GAP_CTL, 0.0, 1.0);
        msg_out.data[6] =  math_map((double)map.lb, 0, 1, 0.0, 1.0);
        msg_out.data[7] =  math_map((double)map.rb, 0, 1, 0.0, 1.0);

        msg_out.data[8] =  math_map((double)map.a, 0, 1, 0.0, 1.0);
        msg_out.data[9] =  math_map((double)map.b, 0, 1, 0.0, 1.0);
        msg_out.data[10] =  math_map((double)map.x, 0, 1, 0.0, 1.0);
        msg_out.data[11] =  math_map((double)map.y, 0, 1, 0.0, 1.0);
        msg_out.data[12] =  math_map((double)map.start, 0, 1, 0.0, 1.0);
        msg_out.data[13] =  math_map((double)map.back, 0, 1, 0.0, 1.0);

        mtx.unlock();
        sleep(0.01);
        fflush(stdout);  
    }  
    xbox_close(xbox_fd);  
}

class JoyStickPublisher : public rclcpp::Node
{
  public:
    JoyStickPublisher()
    : Node("joy_stick_publisher"), count_(0)
    { 
      
      msg_out.data.resize(15);
      std::thread t1(joy_get);
      t1.detach();

      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joy_stick_topic", 100);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&JoyStickPublisher::timer_callback, this));
    }

  private:
    void timer_callback() {
      auto& clk = *this->get_clock();

      RCLCPP_INFO_THROTTLE(this->get_logger(),
                     clk,
                     1000,
                     "joy_stick %f %f", msg_out.data[0], msg_out.data[1]);

      mtx.lock();
      publisher_->publish(msg_out);
      mtx.unlock();
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyStickPublisher>());

  joy_get_flag = false;

  rclcpp::shutdown();
  return 0;
}





