#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

typedef class galaxy_sub{
	//实例化节点
	ros::NodeHandle n;
	//创建两个分别用来subscribe和publish图像的公共接口，其中包含subscribe()来创建image topics的接收，该接口的名称为it
	image_transport::ImageTransport it;
	//创建subscriber
	image_transport::Subscriber sub;

public:
	galaxy_sub():it(n)/*, it_(nh)*/
	{
		//这里我订阅的是原图像，如果想订阅矫正过后的图像，将用以下代码替换掉即可
		//sub = it.subscribe("/galaxy_camera/image_rect", 1, &galaxy_sub::ShowImg_CB, this);
		//因为MER-500-14U3M-L摄像头本身获取的就是黑白图像，所以二者的对比不会太直观。
		sub = it.subscribe("/galaxy_camera/image_raw", 1, &galaxy_sub::ShowImg_CB, this);
		
		//创建图片显示窗口，并命名窗口为“Galaxy camera”
		namedWindow("Galaxy camera", WINDOW_NORMAL); 
	}

	~galaxy_sub()
	{
		//销毁图片显示窗口
		destroyWindow("Galaxy camera");
	}

	void ShowImg_CB(const sensor_msgs::ImageConstPtr& msg)
	{
		Mat img;
		cv_bridge::CvImagePtr cv_ptr;
		//将订阅到的ROS格式图像转换为OpenCV图像格式
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		img = cv_ptr->image;
		//显示图像
		imshow("Galaxy camera", img);
		waitKey(3);
	}
}galaxy_sub;

int main(int argc, char *argv[])
{
	//初始化节点
	ros::init(argc, argv, "galaxy_subscriber");
		
	galaxy_sub galsub;

	ros::spin();
	
	return 0;
}