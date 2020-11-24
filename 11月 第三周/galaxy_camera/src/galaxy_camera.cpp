/*==============================
*pluginlib软件包提供了用于使用ROS构建基础结构编写和动态加载插件的工具，这些插件需要
*在package.xml中注册才能正常工作，包含这个头文件是因为用到了PLUGINLIB_EXPORT_CLASS
*这个宏
==============================*/
#include <pluginlib/class_list_macros.h>
#include <galaxy_camera.h>
/*==============================
*nodelet软件包提供了在同一进程中运行多个算法的方法，该软件包提供了实现节点集所需的nodelet基类，
*以及用于实例化节点集的Nodeletloader类
==============================*/
#include <nodelet/loader.h>
#include <utility>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

namespace galaxy_camera 
{
	/*==============================
	*为了动态加载类，必须将其标记为导出的类，这是通过特殊的宏PLUGINLIB_EXPORT_CLASS来
	*完成的，这里是将galaxy_camera命名空间里的GalaxyCameraNodelet类声明为nodelet命名
	*空间里的Nodelet类
	*----------
	*注意：这个宏语句使用时，最后就是没有分号的，不要刻意加上去
	==============================*/
	PLUGINLIB_EXPORT_CLASS(galaxy_camera::GalaxyCameraNodelet, nodelet::Nodelet)
	GalaxyCameraNodelet::GalaxyCameraNodelet()
	{
	}

	void GalaxyCameraNodelet::onInit()		//初始化galaxy_camera的节点
	{
		/*==============================
		*nh_已经在galaxy_camera.h头文件里定义了
		*----------
		*getPrivateNodeHandle()函数是nodelet::Nodelet类中的成员函数，而类的成员函数可
		*以通过this指针来访问自己类的地址，而前面已经将GalaxyCameraNodelet类声明为Nodelet类
		*了，因此可以通过this指针来进行调用该函数
		==============================*/
		nh_ = this->getPrivateNodeHandle();
		ros::Publisher pub = nh_.advertise<sensor_msgs::CameraInfo>("/galaxy_camera/camera_info", 10);
		/*创建一个用来publish和subscribe图像的公共接口，其中包含advertise()和subscribe()来创建image topics的发布和接收，该接口的名称为it*/
		image_transport::ImageTransport it(nh_);
		/*==============================
		*发布"camera raw image + info topic"的配对在一起的信息
		*----------
		*it.advertiseCamera()函数的两个参数
		*	1、topic的名称
		*	2、队列长度
		==============================*/
		pub_ = it.advertiseCamera("image_raw", 1);
		
		/*==============================
		*image_已经在galaxy_camera.h头文件中定义了，是static sensor_msgs::Image类型，其中
		*包括一个Header类型的成员变量header，而Header类型中又包括了一个string类型的frame_id
		==============================*/
		nh_.param("camera_frame_id",image_.header.frame_id,std::string("pitch_camera"));
		/*camera_name_已经在galaxy_camera.h头文件中定义了，是std::string类型*/
		nh_.param("camera_name", camera_name_, std::string("pitch_camera"));
		/*camera_info_url_已经在galaxy_camera.h头文件中定义了，是std::string类型*/
		nh_.param("camera_info_url", camera_info_url_, std::string(""));
		/*image_width_和image_height_都在galaxy_camera.h头文件中定义了，是int类型，这里将图像二维大小预设为1280x1024*/
		nh_.param("image_width", image_width_, 2592);
		nh_.param("image_height", image_height_, 1944);
		/*image_offset_x_和image_offset_y_都在galaxy_camera.h头文件中定义了，是int类型，这两个变量通常都设置为0，表示垂直轴和水平轴的缩放比*/
		nh_.param("image_offset_x", image_offset_x_, 0);
		nh_.param("image_offset_y", image_offset_y_, 0);
		/*pixel_format_已经在galaxy_camera.h头文件中定义了，是std::string类型，这里将图像预设格式为BGR8*/
		nh_.param("pixel_format", pixel_format_, std::string("bgr8"));
		/*==============================
		*info_manager_已经在galaxy_camera.h头文件中定义了，是camera_info_manager::CameraInfoManager类型,
		*且有共享所有权的智能指针boost::shared_ptr的特性，这里.reset()就是共享指针类的成员函数，虽然智能指针的
		*.reset()主要是起到由新对象接管指针所指向的原对象，但这里就只是给info_manager_进行初始化。
		*----------
		*camera_info_manager::CameraInfoManager()是构造函数，其功能是注册相机信息校准服务service请求的回调函数
		*三个参数：
		*		1、ros::NodeHandle类型的节点句柄
		*		2、const std::string &的字符串指针，默认的相机名称
		*		3、const std::string &的字符串指针，默认的统一资源定位器，用于加载和保存数据，通常为空字符串。
		==============================*/
		info_manager_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));

		/*==============================
		*检测相机默认信息是否设置成功了
		*----------
		*isCalibrated()函数是camera_info_manager::CameraInfoManager类的成员函数，其功能是判断当前相机的信息
		*时候校准成功，其返回值的类型是bool类型，如果返回true，则表明当前相机的信息全部校准成功。
		==============================*/
		if (!info_manager_->isCalibrated())
		{
			/*==============================
			*setCameraName()函数是camera_info_manager::CameraInfoManager类的成员函数，其功能是给相机设置名称，
			*一个参数：
			*		1、const std::string &的字符串指针，相机名称
			==============================*/
			info_manager_->setCameraName(camera_name_);
			/*==============================
			*sensor_msgs头文件包中有专门设置相机信息的类CameraInfo，其中包括了指定图像frame_id、width、height
			*的成员变量
			==============================*/
			sensor_msgs::CameraInfo camera_info;
			camera_info.header.frame_id = image_.header.frame_id;		//图像的frame_id
			camera_info.width = image_width_;			//图像的宽
			camera_info.height = image_height_;			//图像的高
			ROS_INFO("Camera_info has been published successfully.");
			/*==============================
			*setCameraInfo()函数是camera_info_manager::CameraInfoManager类的成员函数，其功能是手动设置相机信息
			*一个参数：
			*		1、const sensor_msgs::CameraInfo &类型指针，相机信息
			==============================*/
			info_manager_->setCameraInfo(camera_info);
		}
		/*显示相机名称、图像宽度、图像高度，用来表明相机信息校准成功*/
		ROS_INFO("Starting '%s' at %dx%d", camera_name_.c_str(),image_width_, image_height_);

		/*==============================
		*std::move()函数的功能是告诉编译器，我们有一个左值，但我们希望像一个右值一样处理它，即将info_manager_中的
		*相机信息赋给info_，info_已经在galaxy_camera.h头文件中定义了，是static sensor_msgs::CameraInfo类型
		*----------
		*getCameraInfo()函数是camera_info_manager::CameraInfoManager类的成员函数，其功能是获取当前相机信息的数据
		==============================*/
		info_ = std::move(info_manager_->getCameraInfo());
		/*image_已经在galaxy_camera.h头文件中定义了，是static sensor_msgs::Image类型*/
		image_.height = image_height_;		//图像的高，uint32类型
		image_.width = image_width_;		//图像的宽，uint32类型
		image_.step = image_width_ * 3;		//图像步进，即全行长度，以字节为单位
		image_.data.resize(image_.step * image_.height);		//image_.data实际图像矩阵大小，大小是(step * rows)，这里是将图像进行大小改变
		image_.encoding = pixel_format_;		//像素编码-通道含义，顺序，大小，这里是BGR8图像处理格式
		/*==============================
		*image_的类型是uint8[]，而uint8其实就是char类型，所以img在galaxy_camera.h头文件中定义被定义为了static char *
		*类型，这里并不是将image_变量中的图像信息赋值给img，而是单纯分配一个和image_实际图像矩阵大小相同的内存空间，
		*而img就是这个内存空间的首地址
		==============================*/
		img = new char[image_.step * image_.height];

		/*至此，相机所有的参数等初始化准备工作已经结束，下面开始正式进入相机的工作流程和控制流程*/
		/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

		/*==============================
		*GXInitLib()函数的功能是初始化galaxy_camera的设备库，这是整个相机工作流程的第一步
		*其没有参数，有两个返回值
		*		1、GX_STATUS_SUCCESS：操作成功，没有发生错误，该宏对应的整数值是0
		*		2、GX_STATUS_NOT_FOUND_TL ：找不到TL库，该宏对应的整数值是-2
		*----------
		*assert()函数的功能是判断括号你的表达式结果是否为假，即计算结果是否为0，若为假，则先向stderr打印一条出错信息，
		*然后通过调用 abort 来终止程序运行。说白了assert()在这里就是起到if()条件判断的作用，判断设备库是否初始化成功。
		==============================*/
		assert(GXInitLib() == GX_STATUS_SUCCESS); 
		uint32_t device_num = 0;		//定义存放当前可用的设备数量的变量，并初始化为0
		/*==============================
		*GXUpdateDeviceList()函数的功能是枚举当前可用的所有设备，并且获取可用设备的个数，这是整个相机工作流程的第二步
		*其有两个参数：
		*		1、[out] (uint32_t *)punNumDevices：用来返回设备个数的地址指针，不能为 NULL 指针
		*		2、[in]  (uint32_t) nTimeOut：枚举的超时时间(单位ms)，如果在用户指定超时时间内成功枚举到设备，则立即返回；如果在用户
		*指定超时时间内没有枚举到设备，则一直等待，直到达到用户指定的超时时间返回。
		*其有三个常见的返回值：
		*		1、GX_STATUS_SUCCESS：操作成功，没有发生错误，该宏对应的整数值是0
		*		2、GX_STATUS_NOT_INIT_API：没有调用GXInitLib初始化库，该宏对应的整数值是-13
		*		3、GX_STATUS_INVALID_PARAMETER：用户输入的指针为NULL，该宏对应的整数值是-5
		*上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST
		==============================*/
		GXUpdateDeviceList(&device_num, 1000);
		assert(device_num == 1); 		// (TODO add multi camera support.)支持增加多个相机
		
		/*GX_OPEN_PARAM是相机打开设备接口专用的结构体，后面三个参数该结构体的成员变量，使用户可以规定打开设备的方式*/
		GX_OPEN_PARAM open_param;
		open_param.accessMode = GX_ACCESS_EXCLUSIVE;//GX_ACCESS_EXCLUSIVE是相机预设的宏，表示以独占的方式打开设备
		open_param.openMode = GX_OPEN_INDEX;		//GX_OPEN_INDEX是相机预设的宏，表示以序列号的方式打开设备
		open_param.pszContent = (char *) "1";		//设置相机的序列号为1
		/*==============================
		*GXOpenDevice()函数的功能是通过指定唯一标识打开设备，例如指定 SN、IP、MAC、Index等，这是整个相机工作流程的第三步，却是相机控制流程的第一步
		*其有两个参数：
		*		1、[in] (GX_OPEN_PARAM *)pOpenParam：用户配置的打开设备参数，参见GX_OPEN_PARAM结构体定义
		*		2、[out] (GX_DEV_HANDLE *)phDevice：接口返回的设备句柄，这里的dev_handle_已经在galaxy_camera.h头文件中定义过了
		*其有五个常见的返回值：
		*		1、GX_STATUS_SUCCESS：操作成功，没有发生错误，该宏对应的整数值是0
		*		2、GX_STATUS_NOT_INIT_API：没有调用GXInitLib初始化库，该宏对应的整数值是-13
		*		3、GX_STATUS_INVALID_PARAMETER：用户输入的指针为NULL，该宏对应的整数值是-5
		*		4、GX_STATUS_NOT_FOUND_DEVICE：没有找到与指定信息匹配的设备，该宏对应的整数值是-3
		*		5、GX_STATUS_INVALID_ACCESS：设备无法在当前访问方式下打开，该宏对应的整数值是-8
		*上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST
		==============================*/
		assert(GXOpenDevice(&open_param, &dev_handle_) == GX_STATUS_SUCCESS);
		/*显示"Camera Opened"，表明相机设备已经成功打开*/
		ROS_INFO("Camera Opened");

		/*==============================
		*定义用来存放像素格式的变量format，因为在GxlAPI.h头文件中，枚举类型的GX_PIXEL_FORMAT_ENTRY常量
		*结构体中的每一个宏都对应一个8位的十六进制数，所以定义format为int64_t类型
		==============================*/
		int64_t format = 0;
		if (pixel_format_ == "mono8")		//判断像素格式是否为mono8
		{
			format = GX_PIXEL_FORMAT_MONO8;		//将mono8对应的常量宏的数值赋给format
		}
		if (pixel_format_ == "mono16")		//判断像素格式是否为mono16
		{
			format = GX_PIXEL_FORMAT_MONO16;		//将mono16对应的常量宏的数值赋给format
		}
		if (pixel_format_ == "bgr8")		//判断像素格式是否为bgr8
		{
			format = GX_PIXEL_FORMAT_BAYER_GB8;		//将bgr8对应的常量宏的数值赋给format
		}
		if (pixel_format_ == "rgb8")		//判断像素格式是否为rgb8
		{
			format = GX_PIXEL_FORMAT_BAYER_RG8;		//将rgb8对应的常量宏的数值赋给format
		}
		if (pixel_format_ == "bgra8")		//判断像素格式是否为bgra8
		{
			format = GX_PIXEL_FORMAT_BAYER_BG8;		//将bgra8对应的常量宏的数值赋给format
		}
		/*判断像素格式是否没有匹配成功，因为只有与上述像素格式全部匹配失败，format才会依旧是初始化的0*/
		if (format == 0)
		{
			static_assert(true, "Illegal format");		//这里static_assert()函数就类似于ROS_ERROR()的功能，在屏幕上答应错误信息提示
		}

		//assert(GXSetEnum(dev_handle_, GX_ENUM_PIXEL_FORMAT, format) == GX_STATUS_SUCCESS);
		/*==============================
		*GXSetInt()函数的功能是设置 Int 类型值，这四句语句都是在设置相机图像参数
		*其有三个参数：
		*		1、[in] (GX_DEV_HANDLE) hDevice：设备句柄
		*		2、[in] (GX_FEATURE_ID) featureID：功能码 ID
		*		3、[in] (int64_t) nValue：用户将要设置的值
		*其有七个常见的返回值：
		*		1、GX_STATUS_SUCCESS：操作成功,没有发生错误，该宏对应的整数值是0
		*		2、GX_STATUS_NOT_INIT_API：没有调用GXInitLib初始化库，该宏对应的整数值是-13
		*		3、GX_STATUS_INVALID_HANDLE：用户传入非法的句柄，该宏对应的整数值是-6
		*		4、GX_STATUS_NOT_IMPLEMENTED：当前不支持的功能，该宏对应的整数值是-12
		*		5、GX_STATUS_ERROR_TYPE：用户传入的featureID类型错误，该宏对应的整数值是-10
		*		6、GX_STATUS_OUT_OF_RANGE：用户传入值越界，比最小值小，或者比最大值大，或者不是步长整数倍，该宏对应的整数值是-11
		*		7、GX_STATUS_INVALID_ACCESS：当前不可访问，不能写，该宏对应的整数值是-8
		*上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST
		*----------
		*GX_INT_WIDTH：Width of the image provided by the device (in pixels).
		*GX_INT_HEIGHT：Height of the image provided by the device (in pixels).
		*GX_INT_OFFSET_X：Horizontal offset from the origin to the region of interest (in pixels).
		*GX_INT_OFFSET_Y：Vertical offset from the origin to the region of interest (in pixels).
		==============================*/
		assert(GXSetInt(dev_handle_, GX_INT_WIDTH, image_width_) == GX_STATUS_SUCCESS);
		assert(GXSetInt(dev_handle_, GX_INT_HEIGHT, image_height_) == GX_STATUS_SUCCESS);
		assert(GXSetInt(dev_handle_, GX_INT_OFFSET_X, image_offset_x_) == GX_STATUS_SUCCESS);
		assert(GXSetInt(dev_handle_, GX_INT_OFFSET_Y, image_offset_y_) == GX_STATUS_SUCCESS);

		/*==============================
		*GXRegisterCaptureCallback()函数的功能是注册采集回调函数，与 GXUnregisterCaptureCallback 接口对应，这是回调采集流程的第二步
		*其有三个参数：
		*		1、[in] (GX_DEV_HANDLE) hDevice：设备句柄
		*		2、[in] (void *)pUserParam：指向用户将在回调处理函数中使用的私有数据指针，这里nullptr等同于空指针
		*		3、[in] (GXCaptureCallBack) callBackFun：用户将要注册的回调处理函数，函数类型参见 GXCaptureCallBack
		*其有五个常见的返回值：
		*		1、GX_STATUS_SUCCESS：操作成功，没有发生错误，该宏对应的整数值是0
		*		2、GX_STATUS_NOT_INIT_API：没有调用GXInitLib初始化库，该宏对应的整数值是-13
		*		3、GX_STATUS_INVALID_HANDLE：用户传入非法的句柄，该宏对应的整数值是-6
		*		4、GX_STATUS_INVALID_PARAMETER：用户传入指针为NULL，该宏对应的整数值是-5
		*		5、GX_STATUS_INVALID_CALL：发送开采命令后，不能注册采集回调函数，该宏对应的整数值是-7
		*上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST
		==============================*/
		GXRegisterCaptureCallback(dev_handle_, nullptr, onFrameCB);
		/*==============================
		*GXStreamOn()函数的功能是开采，包括流开采和设备开采，DQBuf 采集流程仅Linux操作系统，这是DQBuf 采集流程的第二步
		*其有一个参数：
		*		1、[in] (GX_DEV_HANDLE) hDevice：设备句柄
		*其有五个常见的返回值：
		*		1、GX_STATUS_SUCCESS：操作成功,没有发生错误，该宏对应的整数值是0
		*		2、GX_STATUS_NOT_INIT_API：没有调用GXInitLib初始化库，该宏对应的整数值是-13
		*		3、GX_STATUS_INVALID_HANDLE：用户传入非法的句柄，该宏对应的整数值是-6
		*		4、GX_STATUS_INVALID_ACCESS 设备访问模式错误，该宏对应的整数值是-8
		*		5、GX_STATUS_ERROR 不期望发生的未明确指明的内部错误，该宏对应的整数值是-1
		*上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST
		==============================*/
		GXStreamOn(dev_handle_);
		/*打印"Stream On."，表明开采步骤已完成，可以创建采集线程了*/
		ROS_INFO("Stream On.");
		/*==============================
		*GXSetEnum()函数的参数与返回值与GXSetInt()函数是完全一样的，而这唯一的不同就是GXSetEnum()设置的是枚举型变量
		*----------
		*GX_ENUM_BALANCE_WHITE_AUTO：表示自动白平衡使能
		*GX_BALANCE_WHITE_AUTO_CONTINUOUS：表示连续自动白平衡
		==============================*/
		GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);

		ros::NodeHandle p_nh(nh_, "camera");		//父级构造函数
		/*dynamic_reconfigure主要实现的是节点的动态更新*/
		srv_ = new dynamic_reconfigure::Server<CameraConfig>(p_nh);
		dynamic_reconfigure::Server<CameraConfig>::CallbackType
		cb = boost::bind(&GalaxyCameraNodelet::reconfigCB, this, _1, _2);
		srv_->setCallback(cb);
	}

	void GalaxyCameraNodelet::onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrame)
	{
		Mat img_DIY;
		cv_bridge::CvImagePtr cv_ptr;
		
		if (pFrame->status == GX_FRAME_STATUS_SUCCESS)
		{
			DxRaw8toRGB24((void *) pFrame->pImgBuf, img, pFrame->nWidth, pFrame->nHeight, RAW2RGB_NEIGHBOUR, BAYERBG, false);
			memcpy((char *) (&image_.data[0]), img, image_.step * image_.height);
			ros::Time now = ros::Time().now();
			image_.header.stamp = now;
			info_.header.stamp = now;
			pub_.publish(image_, info_);
		}
	}

	void GalaxyCameraNodelet::reconfigCB(CameraConfig &config, uint32_t level)
	{
		(void) level;
		// Exposure
		if (config.exposure_auto)
		{
			double value;
			GXSetFloat(dev_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, config.exposure_max);
			GXSetFloat(dev_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, config.exposure_min);
			GXSetEnum(dev_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
			GXGetFloat(dev_handle_, GX_FLOAT_EXPOSURE_TIME, &value);
			config.exposure_value = value;
		}
		else
		{
			GXSetEnum(dev_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
			GXSetFloat(dev_handle_, GX_FLOAT_EXPOSURE_TIME, config.exposure_value);
		}

		// Gain
		if (config.gain_auto)
		{
			GXSetFloat(dev_handle_, GX_FLOAT_AUTO_GAIN_MIN, config.gain_min);
			GXSetFloat(dev_handle_, GX_FLOAT_AUTO_GAIN_MAX, config.gain_max);
			GXSetEnum(dev_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
			GXGetFloat(dev_handle_, GX_FLOAT_GAIN, &config.gain_value);
		}
		else
		{
			GXSetEnum(dev_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
			GXSetFloat(dev_handle_, GX_FLOAT_GAIN, config.gain_value);
		}

		// Black level
		if (config.black_auto)
		{
			GXSetEnum(dev_handle_, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_CONTINUOUS);
			GXGetFloat(dev_handle_, GX_FLOAT_BLACKLEVEL, &config.black_value);
		}
		else
		{
			GXSetEnum(dev_handle_, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_OFF);
			GXSetFloat(dev_handle_, GX_FLOAT_BLACKLEVEL, config.black_value);
		}
		// Balance White
		switch (config.white_selector){
			case 0:
				GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
				break;
			case 1:
				GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
				break;
			case 2:
				GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
				break;
		}
		if (last_channel_ != config.white_selector)
		{
			GXGetFloat(dev_handle_, GX_FLOAT_BALANCE_RATIO, &config.white_value);
			last_channel_ = config.white_selector;
		}
		if (config.white_auto)
		{
			GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
			GXGetFloat(dev_handle_, GX_FLOAT_BALANCE_RATIO, &config.white_value);
		}
		else
		{
			GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
			GXSetFloat(dev_handle_, GX_FLOAT_BALANCE_RATIO, config.white_value);
		}
	}

	GalaxyCameraNodelet::~GalaxyCameraNodelet()
	{
		GXStreamOff(dev_handle_);
		GXUnregisterCaptureCallback(dev_handle_);
		GXCloseDevice(dev_handle_);
		GXCloseLib();
	}

	char *GalaxyCameraNodelet::img;
	sensor_msgs::Image GalaxyCameraNodelet::image_;
	image_transport::CameraPublisher GalaxyCameraNodelet::pub_;
	sensor_msgs::CameraInfo GalaxyCameraNodelet::info_;
}