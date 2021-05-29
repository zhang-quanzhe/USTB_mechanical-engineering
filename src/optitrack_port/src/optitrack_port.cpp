#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Point.h>
#include "/home/ustb814/car/devel/include/ultra_serial_port/Ultrasound.h"

	// #include "/home/ustb814/car/src/optitrack_port/include/MAVLINK/common/common.h"
	// #include "/home/ustb814/car/src/optitrack_port/include/MAVLINK/common/mavlink.h"
	// #include "/home/ustb814/car/src/optitrack_port/include/MAVLINK/protocol.h"




 // mavlink_vision_position_estimate_t  vision_position_estimate;
 // mavlink_vision_speed_estimate_t vision_speed_estimate;

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "DoorInPos_publisher");

    // 创建节点句柄
    ros::NodeHandle np;

    // 创建一个Publisher，发布名为/person_info的topic，消息类型为ultra_serial_port::Ultrasound，队列长度10
    ros::Publisher DoorInPos_pub = np.advertise<geometry_msgs::Point>("/DoorInPos_info", 1);

    std::string uart_port;
    int uart_baud_rate;
    np.param<std::string>("uart_port", uart_port, "/dev/ttyUSB0");
    np.param<int>("uart_baud_rate", uart_baud_rate, 115200);

    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort(uart_port.c_str());
    //设置串口通信的波特率
    sp.setBaudrate(uart_baud_rate);
    //串口设置timeout
    sp.setTimeout(to);

    ROS_INFO("_baud_rate is %d; uart_port is %s",uart_baud_rate,uart_port.c_str());

    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open ultra port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("ultra port is opened.");
    }
    else
    {
        return -1;
    }
    


    // 设置循环的频率10=10Hz
    ros::Rate loop_rate(10);

    int dis=0;


    //unsigned char rcv_buf[24];


    while (ros::ok())
    {

        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t rcv_buf[128];
            //读出数据
            n = sp.read(rcv_buf, n);
			// for (i=0;i<n;i++)
			// {
			//   // MavlinkDecode(rcv_buf[i]);

			// }
            /*		for(int i=0; i<n; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (rcv_buf[i] & 0xff) << " ";
            }
            */
			// printf("x->%f,y->%f \n", vision_position_estimate.x, vision_position_estimate.y); 
    		
        }
  

        // 初始化learning_topic::Person类型的消息
  		// ultra_serial_port::Ultrasound Ultrasound_msg;
		// Ultrasound_msg.dis = dis;  

        // 发布消息
		// ultrasound_pub.publish(Ultrasound_msg);

       	// ROS_INFO("Ultrasound_info: ++++++dis=%d", Ultrasound_msg.dis);

        // 按照循环频率延时
        loop_rate.sleep();
    }
    //关闭串口
    sp.close();
    return 0;
}





// void MavlinkDecode(uint8_t data)
// {
//     static mavlink_message_t msg;  
//     static mavlink_status_t  status;
	  
    
	  
//     //对接收到的字节数据进行帧解析，接收完一帧时再继续对帧数据进行解析
	
// 	mavlinkcheck=mavlink_parse_char(0, data, &msg, &status);
	
//     if(mavlinkcheck == false)
// 		{
// 		 return;
// 		}else
// 		{
// 		  mavlink_startsend=true;
		
// 		}
//     switch(msg.msgid)
//     {
    	
// 	    /*vision_position*/
//     case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
        
//         mavlink_msg_vision_position_estimate_decode(&msg, &vision_position_estimate);
//         break;	
		
// 		 /*vision_speed*/
// 		case MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE:
        
//         mavlink_msg_vision_speed_estimate_decode(&msg, &vision_speed_estimate);
//         break;	
		
		
		
		

//     default:
//         break;
//     }
// }