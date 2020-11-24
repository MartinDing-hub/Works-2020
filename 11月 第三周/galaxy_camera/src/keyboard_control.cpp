#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
 
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;
 
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
 
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
 
class SmartCarKeyboardTeleopNode
{
    private:
        double walk_vel_;
        double run_vel_;
        double yaw_rate_;
        double yaw_rate_run_;
        
        geometry_msgs::Twist cmdvel_;
        ros::NodeHandle n_;
        ros::Publisher pub_;
 
    public:
        SmartCarKeyboardTeleopNode()
        {
            pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            
            ros::NodeHandle n_private("~");
            n_private.param("walk_vel", walk_vel_, 0.5);
            n_private.param("run_vel", run_vel_, 1.0);
            n_private.param("yaw_rate", yaw_rate_, 1.0);
            n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);
        }
        
        ~SmartCarKeyboardTeleopNode() { }
        void keyboardLoop();
        
        void stopRobot()
        {
            cmdvel_.linear.x = 0.0;
            cmdvel_.angular.z = 0.0;
            pub_.publish(cmdvel_);
        }
};
 
SmartCarKeyboardTeleopNode* tbk;
int kfd = 0;
int kfs = 0;
struct termios cooked, raw;
struct termios cookedn, rawn;
bool done;
 
int main(int argc, char** argv)
{
    ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    SmartCarKeyboardTeleopNode tbk;
    
    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));
    
    ros::spin();
    
    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);
    tcsetattr(kfs, TCSANOW, &cookedn);
    return(0);
}
 
void SmartCarKeyboardTeleopNode::keyboardLoop()
{
    char c;
    char b;
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

    tcgetattr(kfs, &cookedn);
    memcpy(&rawn, &cookedn, sizeof(struct termios));
    rawn.c_lflag &=~ (ICANON | ECHO);
    rawn.c_cc[VEOL] = 1;
    rawn.c_cc[VEOF] = 2;
    tcsetattr(kfs, TCSANOW, &rawn);
    
    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");
    
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    struct pollfd ufs;
    ufs.fd = kfs;
    ufs.events = POLLIN;
    for(;;)
    {
        boost::this_thread::interruption_point();
        
        // get the next event from the keyboard
        int num;
        int nun;
        if (((num = poll(&ufd, 1, 250)) < 0) && ((nun = poll(&ufs, 1, 250)) < 0))
        {
            perror("poll():");
            return;
        }
        
        else if((num > 0) || (nun > 0))
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
            if(read(kfs, &b, 1) < 0)
            {
                perror("read():");
                return;
            }
            printf("c = %c\n b = %c\n",c,b);
        }
        else
        {
            if (dirty == true)
            {
                stopRobot();
                dirty = false;
            }
            
            continue;
        }
        /*
        if ((c=KEYCODE_W && b=KEYCODE_A) || (b=KEYCODE_W && c=KEYCODE_A))
        {
            max_tv = walk_vel_;
            max_rv = yaw_rate_;
            speed = 1;
            turn = 1;
            dirty = true;
        }
        */

        switch(c)
        {
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
        switch(b)
        {
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
        cmdvel_.linear.x = speed * max_tv;
        cmdvel_.angular.z = turn * max_rv;
        pub_.publish(cmdvel_);
    }
}

