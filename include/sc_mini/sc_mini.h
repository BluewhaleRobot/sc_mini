#ifndef NODE_LIDAR_H
#define NODE_LIDAR_H


#include <string>
#include <sensor_msgs/LaserScan.h>

#define ANGLE_MIN_NUMBERS 365

#define ANGLE_MAX_NUMBERS 16000                                                                                                                 
#define ANGLE_MAX_RADIUS 18000
#define AngelJumpDel 50
#define M_UART_BUF_SIZE 10000                                                                 
#define POINTS_OF_HEAD 3
#define POINTS_PER_ROUND 1440
#define B_HEAD0 0x55
#define B_HEAD1 0xAA
#define B_TAIL 0xFA
#define AngelJumpDel 50
#define AngleZoneNumber 15
#define AngleSizePerZone (360.0F/AngleZoneNumber)
#define AngleZoneSampleNo 30
#define COM_FRAME_LEN (2*AngleZoneSampleNo + 9)


#define ET_LIDAR_CALI_SUCCESS   301         // 雷达校准成功
#define ET_LIDAR_CALI_FAILED    302         // 雷达校准失败
#define ET_LIDAR_CALI_START     303         // 雷达校准开始
#define ET_LIDAR_NO_CALI        123         // 雷达未校准
#define ET_LIDAR_NODATA         313         // 雷达没有数据
#define DEG2RAD(x) ((x)*M_PI / 180.)
#define RAD2DEG(x) ((x)*180 / M_PI)
#define HEADER_REPLY_LEN 7
#define LIDAR_FILE  "/mnt/lidar_degree.txt"

#define M1C1_Mini_HEAD_LEN 10
#define M1C1_Mini_FRAME_LEN_MIN (M1C1_Mini_HEAD_LEN + 2)
#define M1C1_Mini_Si_iMAX 100
#define M1C1_Mini_FRAME_LEN_MAX (M1C1_Mini_HEAD_LEN + 2*M1C1_Mini_Si_iMAX)

typedef struct{
	union{
		unsigned char Buffer[M1C1_Mini_FRAME_LEN_MAX];
		struct{
			unsigned char PHL;
			unsigned char PHH;
			unsigned char CT;
			unsigned char LSN;
			unsigned char FSAL;
			unsigned char FSAH;
			unsigned char LSAL;
			unsigned char LSAH;
			unsigned char CSL;
			unsigned char CSH;
			unsigned char Si[2*M1C1_Mini_Si_iMAX];
		}Frame;
	};
	int BufferStart;
	int BufferLen;
}M1C1_MiniDataStr;

typedef struct {
	uint64_t timeStamp;
	float GyroYaw;
}GyroYawStr;

typedef struct {
	float f_begin; // ��ʼ�Ƕ�(0~360)
	float f_end; // ĩβ�Ƕ�(0~360)���������f_begin
}LidarCoverAngleStr;
#define LIDAR_COVER_BAR_NUM 3

namespace sc_m_c
{
	class SCLaser
	{
	public:

		SCLaser(void);

		/**
		* @brief Default destructor
		*/
		~SCLaser();

		/** 
		* @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
		* @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
		*/
		int poll(sensor_msgs::LaserScan& Scan,int fd);
		void PointCloudFilter(sensor_msgs::LaserScan& Scan);
		void angle_insert(sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);
                void start();
		void stop();
		/**
		* @brief Close the driver down and prevent the polling loop from advancing
		*/
		void close() { shutting_down_ = true; }

	private:
		std::string port_; ///< @brief The serial port the driver is attached to
		uint32_t baud_rate_; ///< @brief The baud rate for the serial connection
		bool shutting_down_; ///< @brief Flag for whether the driver is supposed to be shutting down or not
		uint16_t motor_speed_; ///< @brief current motor speed as reported by the LFCD.
	};

}
int open_port(const char *port, int baud_rate);
int read_port(unsigned char *pbuf, unsigned int len);
int write_port(unsigned char *pbuf, unsigned int len);
int close_port();
#endif
