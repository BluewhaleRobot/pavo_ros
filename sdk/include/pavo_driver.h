#pragma once

#include <string>

#include <boost/thread/thread.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <time.h>

#include "sys_types.h"
#include "pavo_types.h"
#include "pavo_cmd.h"

#include "utils.h"
#include "data_filters.h"
#include "pavo_exception.h"

#define SDK_VER "20191108 2.0.2"
//VS2015 does not support exception declaration
#if defined(_WIN32)
#pragma warning (disable:4290)
#endif



namespace pavo
{
	class pavo_driver
	{
	public:
		pavo_driver() throw(pavo_exception);
		pavo_driver(std::string dest_ip, uint16_t dest_port) throw(pavo_exception);
			
		pavo_driver(std::string device_ip, uint16_t device_port, std::string dest_ip, uint16_t dest_port) throw(pavo_exception);

		~pavo_driver();

		bool pavo_open(std::string device_ip, uint16_t device_port) throw(pavo_exception);
		void pavo_close();
		
		bool get_scanned_data(pavo_response_scan_t* data_buffer, int& count, int timeout=0);
		bool get_scanned_data(std::vector<pavo_response_scan_t>& vec_buff, int timeout = 0);

		bool get_scanned_data(pavo_response_pcd_t* data_buffer, int& count, int timeout=0);
		bool get_scanned_data(std::vector<pavo_response_pcd_t>& vec_buff, int timeout = 0);
		
		bool is_lidar_connected();
		
		int get_device_type();
		
		bool get_device_sn(uint32_t &sn);
		
		bool get_device_pn(uint32_t &pn);
		
		void enable_data(bool en);
		
		bool get_dest_ip(std::string& dest_ip);
		bool set_dest_ip(const std::string& dest_ip);
		
		bool get_dest_port(uint16_t& dest_port);
		bool set_dest_port(uint16_t dest_port);
		
		bool get_lidar_ip(std::string& lidar_ip);
		bool set_lidar_ip(const std::string& lidar_ip);
		
		bool get_lidar_port(uint16_t& port);
		bool set_lidar_port(uint16_t port);
		
		bool apply_net_config();
		
		bool get_motor_speed(int& motor_spped);
		bool set_motor_speed(int motor_speed);
		
		bool get_merge_coef(int& merge_coef);
		bool set_merge_coef(int merge_coef);

		bool get_degree_shift(int &degree_shift);
		bool set_degree_shift(int degree_shift);
		
		void get_degree_scope(int& min, int& max);
		bool set_degree_scope(int min, int max);
		
		void enable_motor(bool en);
		
		bool reset();
		bool reset(std::string device_ip, uint16_t device_port, std::string dest_ip, uint16_t dest_port);
		
		void enable_tail_filter(int method);

	private:
		void StartThread();

		void StopIOService();

		void StartReceiveService();

		void HandReceive(const boost::system::error_code& error, std::size_t rxBytes);

		void ProcessPavoFiring(pavo_firing_data_t* firingData, int firingBlock, int azimuthDiff);
		
		void ProcessPavoPacket(unsigned char *data, std::size_t bytesReceived);
		
		void IncreaseFrameCount();

		void DecreaseFrameCount();

		void UpdateInterval(unsigned int tohTime, int laserPerFiring);

		int SendData(const unsigned char* buffer, int size);

		void DisableDataTransfer();

		void EnableDataTransfer();
		
		void EnableMotor();
		
		void DisableMotor();

		int PopFrame(int n = 1);  //ret: the actual number deleted
		
		void UnInitialize();
		
		bool Initialize();

		void EnablePassiveMode();

	
	private:
		bool ConfigRegSimp(PAVO_REG_INDEX reg_set, const char* cmd);  //simplex
		bool ConfigRegDual(PAVO_REG_INDEX reg_set, PAVO_REG_INDEX reg_get, const char* cmd);  //dualplex
		bool ReadReg(PAVO_REG_INDEX reg_get, char* read_buff);
		
	private:
		boost::asio::io_service IOService;
		boost::scoped_ptr<boost::thread> Thread;
		boost::asio::ip::udp::socket* ReceiveSocket;
		boost::asio::ip::udp::socket* SendSocket;
		
		boost::asio::ip::udp::endpoint LIDAREndpoint;  // Endpoint data to
		boost::asio::ip::udp::endpoint RemoteEndpoint;  //The Endpoint data from
		bool IsReceiving;
		bool ShouldStop;

		boost::mutex IsReceivingMtx;
		boost::condition_variable IsReceivingCond;

		boost::mutex DataOpMtx;  //protect: IsResponseReceived, FrameCount, IsDataMode
		boost::condition_variable DataOpCond;
		int FrameCount;
		bool IsDataMode;
		bool IsResponseReceived;
		
		unsigned char ReceiveBuffer[1500];
		unsigned char SendBuffer[1500];
		
		boost::scoped_ptr<boost::asio::io_service::work> DummyWork;

		SynchronizedQueue<pavo_response_scan_t*> PacketQue;
		int FrameCapacity;

		int LastAzimuth;
		unsigned int LastTimestamp;
		
		unsigned int AzimuthDiff;
		unsigned int ResponseCount;
		
		std::string DestIp;
		uint16_t DestPort;
		std::string LidarIp;
		uint16_t LidarPort;

		int DegreeScopeMin;
		int DegreeScopeMax;
		bool DegreeCrop;
		boost::system_time PacketTouch;
        MADataFilter* Filter;
		bool passive_mode;
		bool ReceiveSocketStatus;
		
	};
}
