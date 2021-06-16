// @Time : 2021/6/14 下午7:07
// @Author : WenkyJong
// @Site : MianYang SWUST
// @File : car_control.cpp
// @Contact: wenkyjong1996@gmail.com
// @desc:

#include <ros/ros.h>
#include<termios.h>
#include <serial/serial.h>
#include <iostream>
#include <sys/poll.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57


class SmartCarKeyboardTeleopNode{
    double walk_vel_;

    double run_vel_;

    double yaw_rate_;

    double yaw_rate_run_;

public:
    ~SmartCarKeyboardTeleopNode() { }

    void keyboardLoop();
    void stopRobot();
};

SmartCarKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

void SmartCarKeyboardTeleopNode::keyboardLoop(){
    char c;
    double max_tv = walk_vel_;
    double max_rv = yaw_rate_;
    bool dirty = false;
    int speed = 0;
    int turn = 0;
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");

    struct pollfd ufd; //https://baike.baidu.com/item/poll/3643578?fr=aladdin
    ufd.fd = kfd;
    ufd.events = POLLIN;
    for(;;){
        // get the next event from the keyboard
        int num;
        if ((num = poll(&ufd, 1, 250)) < 0){
            perror("poll():");
            return;
        }
        else if(num > 0){
            if(read(kfd, &c, 1) < 0){
                perror("read():");
                return;
            }
        }
        else{
            if (dirty == true){
                stopRobot();
                dirty = false;
            }
            continue;
        }
        switch(c){
            case KEYCODE_W:
                max_tv = walk_vel_;
                speed = 1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_S:
                max_tv = walk_vel_;
                speed = -1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_A:
                max_rv = yaw_rate_;
                speed = 0;
                turn = 1;
                dirty = true;
                break;
            case KEYCODE_D:
                max_rv = yaw_rate_;
                speed = 0;
                turn = -1;
                dirty = true;
                break;

            case KEYCODE_W_CAP:
                max_tv = run_vel_;
                speed = 1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_S_CAP:
                max_tv = run_vel_;
                speed = -1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_A_CAP:
                max_rv = yaw_rate_run_;
                speed = 0;
                turn = 1;
                dirty = true;
                break;
            case KEYCODE_D_CAP:
                max_rv = yaw_rate_run_;
                speed = 0;
                turn = -1;
                dirty = true;
                break;
            default:
                max_tv = walk_vel_;
                max_rv = yaw_rate_;
                speed = 0;
                turn = 0;
                dirty = false;
        }
    }

}

int main(int argc,char** argv){
    ros::init(argc,argv,"car_control");
    ros::NodeHandle n;

    serial::Serial serial1Port;

    serial1Port.setPort("/dev/ttyTHS0");// using in xavier
    serial1Port.setBaudrate(115200);

    try{
        serial1Port.open();
    }catch (serial::IOException& e){
        ROS_ERROR_STREAM("unable to open port");
    }

    if(serial1Port.isOpen()){
        ROS_INFO_STREAM("/dev/ttyTHS0 is open");
    } else{
        return -1;
    }

    int kfd = 0;
    tcsetattr(kfd, TCSANOW, &cooked); //https://baike.baidu.com/item/tcsetattr/3721550?fr=aladdin TCSANOW：不等数据传输完毕就立即改变属性
    ros::Rate loop_rate(500);

    while (ros::ok()){

        if(){
            serial1Port.write("");
        }

        loop_rate.sleep();
    }
    serial1Port.close();
    return 0;
}


