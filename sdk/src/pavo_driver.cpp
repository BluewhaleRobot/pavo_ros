#include <string>

#include <boost/thread/thread.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <time.h>

#include "../include/utils.h"
#include "../include/pavo_driver.h"
#include "../include/byte_order.h"


#define PAVO_DEVICE_IP "169.254.1.101"
//#define PAVO_DEVICE_IP "192.168.1.101"
#define PAVO_SERVER_IP "169.254.1.100"
//#define PAVO_SERVER_IP "192.168.1.100"
#define PAVO_DEVICE_PORT (2368)
#define PAVO_SERVER_PORT (2368)
#define PAVO_MAX_FRAME_COUNT (1)


namespace pavo
{
	typedef boost::error_info<struct tag_errmsg, std::string> errmsg_info;
	const int COMMAND_WAIT_MS = 100; //100ms

	int PavoAdjustTimeStamp(int laserPerFiring,
							int firingblock, //one of the 12 blocks
							int dsr, //laser id 0, 0~1, 0~2, 0~4, 0~8
							unsigned int laserInterval, //in terval between lasers
							int firingwithinblock)
	{
		return ( firingblock*laserPerFiring + dsr + firingwithinblock* laserPerFiring/2) * laserInterval;
	}

	int PavoAdjustAzimuth(int laserPerFiring, 
				int firingblock, //one of the 12 blocks
				int dsr, //laser id 0, 0~1, 0~2, 0~4, 0~8
				unsigned int azimuthDiff, //in terval between lasers
				int firingwithinblock)
	{
		return (firingwithinblock * azimuthDiff/2 + dsr * azimuthDiff / laserPerFiring);
	}

	
	
	pavo_driver::pavo_driver()
		try: 
		IOService(),
		Thread(),
		ReceiveSocket(new boost::asio::ip::udp::socket(IOService, 
				boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(PAVO_SERVER_IP), PAVO_SERVER_PORT))), //any ip
		SendSocket(new boost::asio::ip::udp::socket(IOService, boost::asio::ip::udp::v4())),  //the socket Send to Pavo
		LIDAREndpoint(boost::asio::ip::address::from_string(PAVO_DEVICE_IP), PAVO_DEVICE_PORT),
		IsReceiving(false),
		ShouldStop(false),
		IsDataMode(true),
		IsResponseReceived(false),
		FrameCount(0),
		LastAzimuth(0),
		LastTimestamp(std::numeric_limits<unsigned int>::max()),
		DegreeScopeMin(0),
		DegreeScopeMax(35999),
		DegreeCrop(false),
		//LaserInterval(0),
		AzimuthDiff(0),
		FrameCapacity(PAVO_MAX_FRAME_COUNT),
		ResponseCount(0),
		DummyWork(new boost::asio::io_service::work(IOService))
	{
		LidarIp = PAVO_DEVICE_IP;
		LidarPort = PAVO_DEVICE_PORT;
		DestIp = PAVO_SERVER_IP;
		DestPort = PAVO_SERVER_PORT;
		StartThread();
		StartReceiveService();
                Filter = new MADataFilter(5, 25); //scale:5, distace:15units = 3cm/0.2cm 
	}
	catch (boost::system::system_error &e)
	{
		boost::system::error_condition ecnd = e.code().default_error_condition();
		if (ecnd.value() == boost::system::errc::errc_t::address_not_available)
		{
			std::stringstream ss;
			ss << "The address could not be binded, IP:Port " << PAVO_SERVER_IP << ":" << PAVO_SERVER_PORT;
			throw std::runtime_error(ss.str());
		}
		else
			throw;
	}
	
	pavo_driver::pavo_driver(std::string device_ip, uint16_t device_port, std::string dest_ip, uint16_t dest_port)
		try:
		IOService(),
		Thread(),
		SendSocket(new boost::asio::ip::udp::socket(IOService, boost::asio::ip::udp::v4())),  //the socket Send to Pavo
		IsReceiving(false),
		ShouldStop(false),
		IsDataMode(true),
		IsResponseReceived(false),
		FrameCount(0),
		LastAzimuth(0),
		LastTimestamp(std::numeric_limits<unsigned int>::max()),
		AzimuthDiff(0),
		FrameCapacity(PAVO_MAX_FRAME_COUNT),
		ResponseCount(0),
		DummyWork(new boost::asio::io_service::work(IOService))
	{
		char ip_bytes[4];
		if(!NetCoverter::String2Bytes(device_ip, ip_bytes) ||
			!NetCoverter::String2Bytes(dest_ip, ip_bytes))
			throw std::invalid_argument("invalid ip format!");
			
		LidarIp = device_ip;
		LidarPort = device_port;
		DestIp = dest_ip;
		DestPort = dest_port;
		
		ReceiveSocket = new boost::asio::ip::udp::socket(IOService, 
							boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), dest_port));
		LIDAREndpoint.address(boost::asio::ip::address::from_string(device_ip));
		LIDAREndpoint.port(device_port);
		
		StartThread();
		StartReceiveService();	
	}
	catch (boost::system::system_error &e)
	{
		boost::system::error_condition ecnd = e.code().default_error_condition();
		if (ecnd.value() == boost::system::errc::errc_t::address_not_available)
		{
			std::stringstream ss;
			ss << "The specified address could not be binded, IP:Port " << dest_ip << ":" << PAVO_SERVER_PORT;
			throw std::runtime_error(ss.str());
		}
		else
			throw;
	}

	pavo_driver::~pavo_driver()
	{
		UnInitialize();
	}
	
	bool pavo_driver::Initialize()
	{
		
		if(LidarIp.empty())
			return false;
		IOService.reset();

		DummyWork = new boost::asio::io_service::work(IOService);

		ReceiveSocket = new boost::asio::ip::udp::socket(IOService, 
							boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), DestPort));


		SendSocket = new boost::asio::ip::udp::socket(IOService, boost::asio::ip::udp::v4());
							
		LIDAREndpoint.address(boost::asio::ip::address::from_string(LidarIp));
		LIDAREndpoint.port(LidarPort);
		
		IsReceiving = false;
		ShouldStop = false;
		IsDataMode = true;
		IsResponseReceived =false;
		FrameCount = 0;
		LastAzimuth = 0;
		LastTimestamp = std::numeric_limits<unsigned int>::max();
		AzimuthDiff = 0;
		FrameCapacity = PAVO_MAX_FRAME_COUNT;
		ResponseCount = 0;
		EnableDataTransfer();
		
		StartThread();
		StartReceiveService();	
		
		return true;
	}

	void pavo_driver::UnInitialize()
	{

		if (this->ReceiveSocket->is_open())
		{
			{
				boost::unique_lock<boost::mutex> guard(this->IsReceivingMtx);
				//StopIOService();
				this->ShouldStop = true;
				this->ReceiveSocket->shutdown(boost::asio::ip::udp::socket::shutdown_receive);
				this->ReceiveSocket->close();
				
				while (this->IsReceiving)
				{
					this->IsReceivingCond.wait(guard);
				}
			}
		}

		delete this->ReceiveSocket;
		this->ReceiveSocket = nullptr;

		delete this->DummyWork;  //call destructor
		this->DummyWork = nullptr;


		if (this->Thread)
		{
			this->Thread->join();
			this->Thread.reset();
		}

		if (this->SendSocket->is_open())
		{
			this->SendSocket->shutdown(boost::asio::ip::tcp::socket::shutdown_send);
			this->SendSocket->close();
			delete this->SendSocket;
			this->SendSocket = nullptr;
		}
	}

	void pavo_driver::StopIOService()
	{
		IOService.stop();
		while (!IOService.stopped())
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
			IOService.stop();
		}
		//IOService.reset();
	}

	void pavo_driver::StartThread()
	{
		if (this->Thread)
			return;

		this->Thread.reset(new boost::thread(
			boost::bind(&boost::asio::io_service::run, &this->IOService)
		));
	}

	void pavo_driver::StartReceiveService()
	{
		{
			boost::lock_guard<boost::mutex> guard(this->IsReceivingMtx);
			this->IsReceiving = true;
		}
		ReceiveSocket->async_receive_from(boost::asio::buffer(this->ReceiveBuffer), RemoteEndpoint,
			boost::bind(&pavo_driver::HandReceive, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));

	}

	void pavo_driver::HandReceive(const boost::system::error_code& error, std::size_t rxBytes)
	{

		if (error || this->ShouldStop)
		{
			{
				boost::lock_guard<boost::mutex> guard(this->IsReceivingMtx);
				this->IsReceiving = false;
			}
			this->IsReceivingCond.notify_one();
			return;
		}

		{
			boost::lock_guard<boost::mutex> guard(this->DataOpMtx);  //protect "IsDataMode"

			if (!IsDataMode)  //������ģʽ
			{
				if (IsCMDResponse(ReceiveBuffer))  //ȷ������������ģʽ
				{
					this->IsResponseReceived = true;
					this->DataOpCond.notify_one();

				}
				//else :drop this packet, continue for next
				this->StartReceiveService();
				return;
			}

		}

		if (!IsCMDResponse(ReceiveBuffer)) 
		{
			this->ProcessPavoPacket(this->ReceiveBuffer, rxBytes);
		}

		//drop this packet, continue for next
		this->StartReceiveService();
	}

	
	
	bool pavo_driver::reset()
	{
		UnInitialize();
		return Initialize();
	}
	
	bool pavo_driver::reset(std::string device_ip, uint16_t device_port, std::string dest_ip, uint16_t dest_port)
	{
		char ip_bytes[4];
		memset(ip_bytes, 0, 4);
		if(!NetCoverter::String2Bytes(device_ip, ip_bytes) ||
			!NetCoverter::String2Bytes(dest_ip, ip_bytes))
		{
			return false;
		}
		
		if(!set_dest_ip(dest_ip) || !set_dest_port(dest_port) ||
			!set_lidar_ip(device_ip) || !set_lidar_port(device_port) )
			return false;
		
		return Initialize();	
	}

	bool pavo_driver::get_scanned_data(pavo_response_scan_t* data_buffer, int& count, int timeout)
	{
		if(timeout < 0)
			return false;

		{
			boost::unique_lock<boost::mutex> guard(this->DataOpMtx);
			if(timeout == 0)
			{
				while(this->FrameCount==0)
				{
					this->DataOpCond.wait(guard);
				}
			}
			else //timeout > 0
			{
				boost::system_time const sys_timeout = boost::get_system_time()+ boost::posix_time::milliseconds(timeout);
				while(this->FrameCount==0)
				{
					if(!this->DataOpCond.timed_wait(guard, sys_timeout))
						return false;
				}
			}

			pavo_response_scan_t* response_ptr ;
			if( !this->PacketQue.dequeue(response_ptr))
				return false;

			int count_cap = count;
			count = 0;
			while(!IsWedgeData(response_ptr))
			{
				if (count < count_cap)
					memcpy(&data_buffer[count++], response_ptr, sizeof(pavo_response_scan_t));
				else
					count++;

				delete response_ptr;
				if( !this->PacketQue.dequeue(response_ptr))
					return false;
			}

			FrameCount--;
		}
		bool m_first = true;
		//bool m_first = false;
		Filter->Filter(data_buffer, count, m_first);
		return true;
	}

	bool pavo_driver::get_scanned_data(std::vector<pavo_response_scan_t>& vec_buff, int timeout)
	{
		if (timeout < 0)
			return false;

		{
			boost::unique_lock<boost::mutex> guard(this->DataOpMtx);
			if (timeout == 0)
			{
				while (this->FrameCount == 0)
				{
					this->DataOpCond.wait(guard);
				}
			}
			else //timeout > 0
			{
				boost::system_time const sys_timeout = boost::get_system_time() + boost::posix_time::milliseconds(timeout);
				while (this->FrameCount == 0)
				{
					if (!this->DataOpCond.timed_wait(guard, sys_timeout))
						return false;
				}
			}

			vec_buff.clear();

			pavo_response_scan_t* response_ptr;
			if (!this->PacketQue.dequeue(response_ptr))
				return false;
            
			while (!IsWedgeData(response_ptr))
			{
				//memcpy(&vec_buff[count++], response_ptr, sizeof(pavo_response_scan_t));
				vec_buff.push_back(*response_ptr);
				delete response_ptr;
				if (!this->PacketQue.dequeue(response_ptr))
					return false;

			}

			FrameCount--;
		}
		bool m_first = true;
		//bool m_first = false;
		Filter->Filter(&vec_buff[0], vec_buff.size(), m_first);
		return true;
	}


	bool pavo_driver::get_scanned_data(pavo_response_pcd_t* data_buffer, int& count, int timeout)
	{
		if(timeout < 0)
			return false;
		{
			boost::unique_lock<boost::mutex> guard(this->DataOpMtx);
			SinuLookupTable* sinu_ptr = SinuLookupTable::Instance();
			if(timeout==0)
			{
				while(this->FrameCount==0)
				{
					this->DataOpCond.wait(guard);
				}
			}
			else //timeout > 0
			{
				boost::system_time const sys_timeout = boost::get_system_time()+ boost::posix_time::milliseconds(timeout);
				while(this->FrameCount==0)
				{
					if(!this->DataOpCond.timed_wait(guard, sys_timeout))
						return false;
				}
			}
			pavo_response_scan_t* response_ptr ;
			if( !this->PacketQue.dequeue(response_ptr))
				return false;

			int count_cap = count;
			count = 0;
			while(!IsWedgeData(response_ptr))
			{
				if (count < count_cap)
				{
					pavo_response_pcd_t* pcd_ptr = &data_buffer[count++];
					pcd_ptr->x = sinu_ptr->CosValue(response_ptr->angle) * response_ptr->distance;
					pcd_ptr->y = sinu_ptr->SinValue(response_ptr->angle) * response_ptr->distance;
					pcd_ptr->z = 0;
					pcd_ptr->intensity = response_ptr->intensity;
				}
				else
					count++;

				delete response_ptr;
				if( !this->PacketQue.dequeue(response_ptr))
					return false;

			}

			FrameCount--;
		}
		return true;
	}

	bool pavo_driver::get_scanned_data(std::vector<pavo_response_pcd_t>& vec_buff, int timeout)
	{
		if (timeout < 0)
			return false;
		{
			boost::unique_lock<boost::mutex> guard(this->DataOpMtx);
			SinuLookupTable* sinu_ptr = SinuLookupTable::Instance();
			if (timeout == 0)
			{
				while (this->FrameCount == 0)
				{
					this->DataOpCond.wait(guard);
				}
			}
			else //timeout > 0
			{
				boost::system_time const sys_timeout = boost::get_system_time() + boost::posix_time::milliseconds(timeout);
				while (this->FrameCount == 0)
				{
					if (!this->DataOpCond.timed_wait(guard, sys_timeout))
						return false;
				}
			}

			vec_buff.clear();

			pavo_response_scan_t* response_ptr;
			if (!this->PacketQue.dequeue(response_ptr))
				return false;

			
			while (!IsWedgeData(response_ptr))
			{
				
				pavo_response_pcd_t pcd_item;
				pcd_item.x = sinu_ptr->CosValue(response_ptr->angle) * response_ptr->distance;
				pcd_item.y = sinu_ptr->SinValue(response_ptr->angle) * response_ptr->distance;
				pcd_item.z = 0;
				pcd_item.intensity = response_ptr->intensity;
				vec_buff.push_back(pcd_item);
				
				delete response_ptr;
				if (!this->PacketQue.dequeue(response_ptr))
					return false;

			}

			FrameCount--;
		}
		return true;
	}

	bool pavo_driver::is_lidar_connected()
	{

		char conn_bytes[PAVO_RESPONSE_LEN];
		memset(conn_bytes, 0, PAVO_RESPONSE_LEN);
		if (!ReadReg(REG_DEVICE_CONNECTION_GET, conn_bytes))
			return false;

		if ( memcmp(PAVO_RESPONSE_CONNECTED, conn_bytes, PAVO_RESPONSE_LEN)==0)
		{
			return true;
		}
		
		return false;

	}

	int pavo_driver::get_device_type()
	{
		char type_bytes[PAVO_RESPONSE_LEN+1];
		memset(type_bytes, '\0', PAVO_RESPONSE_LEN + 1);
		if(!ReadReg(REG_DEVICE_TYPE_GET, type_bytes))
			return -1;

		return static_cast<int>(type_bytes[3]);
	}

	void pavo_driver::enable_data(bool en)
	{
		if(en)
			this->EnableDataTransfer();
		else
			this->DisableDataTransfer();
	}

	bool pavo_driver::get_dest_ip(std::string& dest_ip)
	{
		char ip_bytes[PAVO_RESPONSE_LEN];
		memset(ip_bytes, 0, PAVO_RESPONSE_LEN);
		if(!ReadReg(REG_DEST_IP_GET, ip_bytes))
			return false;

		return NetCoverter::Bytes2String(dest_ip, ip_bytes);
	}
	
	bool pavo_driver::set_dest_ip(const std::string& dest_ip)
	{	
		char ip_bytes[PAVO_RESPONSE_LEN];
		memset(ip_bytes, 0, PAVO_RESPONSE_LEN);
		if(!NetCoverter::String2Bytes(dest_ip, ip_bytes))
			return false;
		
		DestIp = dest_ip;
		
		return ConfigRegDual(REG_DEST_IP_CONFIG_SET, REG_DEST_IP_CONFIG_GET, ip_bytes);  //dualplex
	}
		
	bool pavo_driver::get_dest_port(uint16_t& dest_port)
	{
		char port_bytes[PAVO_RESPONSE_LEN];
		memset(port_bytes, 0, PAVO_RESPONSE_LEN);
		if(!ReadReg(REG_DEST_PORT_GET, port_bytes))
			return false;

		return NetCoverter::Bytes2Port(dest_port, port_bytes);
	}
	
	bool pavo_driver::set_dest_port(uint16_t dest_port)
	{
		char port_bytes[PAVO_RESPONSE_LEN];
		memset(port_bytes, 0, PAVO_RESPONSE_LEN);
		if(!NetCoverter::Port2Bytes(dest_port, port_bytes))
			return false;
		
		DestPort = dest_port;
		return ConfigRegDual(REG_DEST_PORT_CONFIG_SET, REG_DEST_PORT_CONFIG_GET, port_bytes);  //dualplex
	}
	
	bool pavo_driver::get_lidar_ip(std::string& lidar_ip)
	{
		char ip_bytes[PAVO_RESPONSE_LEN];
		memset(ip_bytes, 0, PAVO_RESPONSE_LEN);
		if(!ReadReg(REG_DEVICE_IP_GET, ip_bytes))
			return false;

		return NetCoverter::Bytes2String(lidar_ip, ip_bytes);		
	}
	
	bool pavo_driver::set_lidar_ip(const std::string& lidar_ip)
	{
		char ip_bytes[PAVO_RESPONSE_LEN];
		memset(ip_bytes, 0, PAVO_RESPONSE_LEN);
		if(!NetCoverter::String2Bytes(lidar_ip, ip_bytes))
			return false;
		LidarIp = lidar_ip;
		return ConfigRegDual(REG_DEVICE_IP_CONFIG_SET, REG_DEVICE_IP_CONFIG_GET, ip_bytes);  //dualplex
	}
	
	bool pavo_driver::get_lidar_port(uint16_t& lidar_port)
	{
		char port_bytes[PAVO_RESPONSE_LEN];
		memset(port_bytes, 0, PAVO_RESPONSE_LEN);
		if(!ReadReg(REG_DEVICE_PORT_GET, port_bytes))
			return false;

		return NetCoverter::Bytes2Port(lidar_port, port_bytes);
	}
	
	bool pavo_driver::set_lidar_port(uint16_t lidar_port)
	{
		char port_bytes[PAVO_RESPONSE_LEN];
		memset(port_bytes, 0, PAVO_RESPONSE_LEN);
		if(!NetCoverter::Port2Bytes(lidar_port, port_bytes))
			return false;
		LidarPort = lidar_port;
		return ConfigRegDual(REG_DEVICE_PORT_CONFIG_SET, REG_DEVICE_PORT_CONFIG_GET, port_bytes);  //dualplex
	}
		
	bool pavo_driver::apply_net_config()
	{
		if(!ConfigRegSimp(REG_NETCONFIG_APPLY, reinterpret_cast<const char*> (PAVO_CMD[CMD_NETCONFIG_APPLY])))
			return false;
		
		return true;
	}
		
	bool pavo_driver::get_motor_speed(int& motor_speed)
	{
		char speed_bytes[PAVO_RESPONSE_LEN];
		memset(speed_bytes, 0, PAVO_RESPONSE_LEN);
		if(!ReadReg(REG_MOTOR_SPEED_GET, speed_bytes))
			return false;
		
		motor_speed = speed_bytes[3];
		
		return true;
	}
	
	bool pavo_driver::set_motor_speed(int motor_speed)
	{
		if(!IsValidMotorSpeed(motor_speed))
			return false;
		
		char speed_bytes[PAVO_RESPONSE_LEN];
		memset(speed_bytes, 0, PAVO_RESPONSE_LEN);
		
		speed_bytes[3] = static_cast<unsigned char>(motor_speed);
		
		return ConfigRegDual(REG_MOTOR_CONFIG_SET, REG_MOTOR_CONFIG_GET, speed_bytes);  //dualplex
	}
	
	
	bool pavo_driver::get_merge_coef(int& merge_coef)
	{
		char merge_bytes[PAVO_RESPONSE_LEN];
		memset(merge_bytes, 0, PAVO_RESPONSE_LEN);
		if(!ReadReg(REG_POINTCLOUD_MERGE_GET, merge_bytes))
			return false;
			
		merge_coef = merge_bytes[3];
			
		return true;
	}
	
	bool pavo_driver::set_merge_coef(int merge_coef)
	{
		if(!IsValidPointCloudMergeCoef(merge_coef))
			return false;
		
		char coef_bytes[PAVO_RESPONSE_LEN];
		memset(coef_bytes, 0, PAVO_RESPONSE_LEN);
		
		coef_bytes[3] = static_cast<unsigned char>(merge_coef);
		
		return ConfigRegDual(REG_POINTCLOUD_MERGE_SET, REG_POINTCLOUD_MERGE_GET, coef_bytes);  
	}
	
	bool pavo_driver::get_degree_shift(int &degree_shift)
	{
		char shift_bytes[PAVO_RESPONSE_LEN];
		memset(shift_bytes, 0, PAVO_RESPONSE_LEN);
		if(!ReadReg(REG_DEGREE_SHIFT_GET, shift_bytes))
			return false;
			
		degree_shift = shift_bytes[3];
			
		return true;
	}
	
	
	bool pavo_driver::set_degree_shift(int degree_shift)
	{
		if((degree_shift<DEGREE_SHIFT_MIN) || (degree_shift>DEGREE_SHIFT_MAX))
			return false;
		
		char shift_bytes[PAVO_RESPONSE_LEN];
		memset(shift_bytes, 0, PAVO_RESPONSE_LEN);
		
		shift_bytes[3] = static_cast<unsigned char>(degree_shift);
		
		return ConfigRegDual(REG_DEGREE_SHIFT_SET, REG_DEGREE_SHIFT_GET, shift_bytes);  
	}
		
	bool pavo_driver::pavo_driver::set_degree_scope(int min, int max)
	{
		if((min<DEGREE_SCOPE_MIN) || (max>DEGREE_SCOPE_MAX) || (min>=max) )
			return false;
		
		if(min==DEGREE_SCOPE_MIN || max==DEGREE_SCOPE_MAX)
			DegreeCrop = false;
		else
			DegreeCrop = true;
		
		DegreeScopeMin = min;
		DegreeScopeMax = max;
		return true;
	}
	
	void pavo_driver::pavo_driver::get_degree_scope(int& min, int& max)
	{
		min = DegreeScopeMin;
		max = DegreeScopeMax;
	}


	void pavo_driver::IncreaseFrameCount()
	{
		boost::lock_guard<boost::mutex> guard(this->DataOpMtx);
		FrameCount++;
	}

	void pavo_driver::DecreaseFrameCount()
	{
		boost::lock_guard<boost::mutex> guard(this->DataOpMtx);
		FrameCount--;
	}

	void pavo_driver::ProcessPavoFiring(pavo_firing_data_t* firingData,
					int firingBlock,
					int azimuthDiff)
	{	
		for (int dsr = 0; dsr < DATA_PER_FIRING; dsr++)
		{
			unsigned char laserId = static_cast<unsigned char>(dsr);
			unsigned short azimuth = __bytes2uint16(firingData->rotationalPosition);
			
			int firingWithinBlock = 0; 
				
			if(laserId >= NUM_LASERS)
			{
				laserId -= NUM_LASERS;
				firingWithinBlock = 1;
			}
				
			int azimuthadjustment = PavoAdjustAzimuth(DATA_PER_FIRING, firingBlock, laserId,azimuthDiff, firingWithinBlock);
			
			{
				pavo_response_scan_t* response_ptr = new pavo_response_scan_t();
				response_ptr->angle = azimuth + azimuthadjustment;
				if(DegreeCrop)  
				{
					if((response_ptr->angle >= DegreeScopeMin) && (response_ptr->angle <= DegreeScopeMax) )
						response_ptr->distance = __bytes2uint16(firingData->laserReturns[dsr].distance);
					else
						response_ptr->distance = 0;
				}
				else
					response_ptr->distance = __bytes2uint16(firingData->laserReturns[dsr].distance);

				response_ptr->intensity = firingData->laserReturns[dsr].intensity;
				this->PacketQue.enqueue(response_ptr);
				this->ResponseCount++;
			}
		}
	}

        
	void pavo_driver::ProcessPavoPacket(unsigned char *data, std::size_t bytesReceived)
	{
		pavo_data_packet_t* dataPacket = reinterpret_cast<pavo_data_packet_t *>(data);
			
		//Skip the first packet for no time interpolation could be implemented
		unsigned long rawtime = __bytes2uint32(dataPacket->timestamp);
		if(this->LastTimestamp == std::numeric_limits<unsigned int>::max())
		{
			this->LastTimestamp = rawtime;
			return;
		}
			
		this->UpdateInterval(rawtime, DATA_PER_FIRING); //microseconds
		
		
		//Get the median of azimuthDiff
		std::vector<int> diffs (FIRING_PER_PKT - 1);
		for(int i = 0; i < FIRING_PER_PKT - 1; ++i)
		{
			int localDiff = (36000 + __bytes2uint16(dataPacket->firingData[i+1].rotationalPosition) -
						 __bytes2uint16(dataPacket->firingData[i].rotationalPosition)) % 36000;
			if(localDiff > 200)
			{
				std::cout << "position diff greater than 100: " << localDiff << std::endl;
				std::cout << "Previous position: " << (dataPacket->firingData[i].rotationalPosition %36000) << std::endl;
			}
			diffs[i] = localDiff;
		}
		std::nth_element(diffs.begin(),
				   diffs.begin() + FIRING_PER_PKT/2,
				   diffs.end());
		int azimuthDiff = diffs[FIRING_PER_PKT/2];
		this->AzimuthDiff = azimuthDiff;

		assert(azimuthDiff >= 0);

		for ( int firingBlock=0; firingBlock < FIRING_PER_PKT; ++firingBlock)
		{
			pavo_firing_data_t * firingData = &(dataPacket->firingData[firingBlock]);

			//after each circle, data be re-initialized by enque a wedge
			uint16_t rotation = __bytes2uint16(firingData->rotationalPosition);

			if (rotation < this->LastAzimuth)
			{
				if(this->FrameCount >= this->FrameCapacity)
					this->PopFrame();

				pavo_response_scan_t* response_ptr = GenWedgeData();
				this->PacketQue.enqueue(response_ptr);
				this->ResponseCount = 0;
				boost::lock_guard<boost::mutex> guard(this->DataOpMtx);
				FrameCount++;
				DataOpCond.notify_one();
			}
				
				
			this->ProcessPavoFiring(firingData, firingBlock, azimuthDiff);
			
			this->LastAzimuth = rotation;
		}
		 
	}


	void pavo_driver::UpdateInterval(unsigned int tohTime, int dataPerFiring)
	{
		long long timeDiff;// = tohTime - this->LastTimestamp;
		
		if(tohTime < this->LastTimestamp)
		{
			timeDiff = tohTime +  3600LL * 1000000 - this->LastTimestamp;
		}
		else
		{
			timeDiff = tohTime - this->LastTimestamp;
		}
	}

	int pavo_driver::SendData(const unsigned char* buffer, int size)
	{
		return this->SendSocket->send_to(boost::asio::buffer(buffer, size), LIDAREndpoint); 
	}
	
	
	bool pavo_driver::ReadReg(PAVO_REG_INDEX reg_get, char* read_buff)
	{
		if(read_buff == nullptr)
			return false;
		
		this->DisableDataTransfer();

		char tmp_buff[PAVO_REG_LEN + PAVO_CMD_LEN];
		
		pavo_cmd_packet_single_t data_request_packet;
		memcpy(data_request_packet.reg, PAVO_REG[reg_get], PAVO_REG_LEN);
		unsigned char* packet_ptr = reinterpret_cast<unsigned char*>(&data_request_packet); 		//check by reading back
		bool time_out = false;
		{
			boost::unique_lock<boost::mutex> guard(this->DataOpMtx);
			this->IsResponseReceived = false;
			this->SendData(packet_ptr, PAVO_REG_LEN);

			boost::system_time const sys_timeout = boost::get_system_time() + boost::posix_time::milliseconds(COMMAND_WAIT_MS);
			while(!this->IsResponseReceived)
			{
				//this->DataOpCond.wait(guard);
				if (!this->DataOpCond.timed_wait(guard, sys_timeout))//Network error
				{
					time_out = true;
					break;
				}
			}
			memcpy(tmp_buff, ReceiveBuffer, PAVO_REG_LEN + PAVO_CMD_LEN);
		}
		this->EnableDataTransfer();

		if (time_out)
			return false;

		if (memcmp(tmp_buff, PAVO_REG[reg_get], PAVO_REG_LEN) != 0)
			return false;

		memcpy(read_buff, tmp_buff + PAVO_REG_LEN, PAVO_RESPONSE_LEN);
		return true;
	}
	
	bool pavo_driver::ConfigRegSimp(PAVO_REG_INDEX reg_set, const char* cmd)
	{
		if(cmd==nullptr)
			return false;
		
		pavo_cmd_packet_dual_t cmd_packet;
		memcpy(cmd_packet.reg, PAVO_REG[reg_set], PAVO_REG_LEN);
		memcpy(cmd_packet.cmd, cmd, PAVO_CMD_LEN);

		unsigned char* packet_ptr = reinterpret_cast<unsigned char*>(&cmd_packet);
		
		this->SendData(packet_ptr, PAVO_REG_LEN + PAVO_CMD_LEN);
		
		return true;

	}
	
	bool pavo_driver::ConfigRegDual(PAVO_REG_INDEX reg_set, PAVO_REG_INDEX reg_get, const char* cmd)
	{
		if(cmd==nullptr)
			return false;
		
		
		this->DisableDataTransfer();
		
		bool ret_bool = false;
		unsigned char recved_bytes[PAVO_REG_LEN + PAVO_CMD_LEN];
		memset(recved_bytes, 0, PAVO_REG_LEN + PAVO_CMD_LEN);
		
		pavo_cmd_packet_dual_t set_packet;
		memcpy(set_packet.reg, PAVO_REG[reg_set], PAVO_REG_LEN);
		memcpy(set_packet.cmd, cmd, PAVO_CMD_LEN);
		
		unsigned char* packet_ptr = reinterpret_cast<unsigned char*>(&set_packet);  //config
		this->SendData(packet_ptr, PAVO_REG_LEN + PAVO_CMD_LEN);
		
		pavo_cmd_packet_single_t data_request_packet;
		memcpy(data_request_packet.reg, PAVO_REG[reg_get], PAVO_REG_LEN);
		packet_ptr = reinterpret_cast<unsigned char*>(&data_request_packet); 		//check by reading back
		{
			boost::unique_lock<boost::mutex> guard(this->DataOpMtx);
			this->IsResponseReceived = false;
			this->SendData(packet_ptr, PAVO_REG_LEN);
			while(!this->IsResponseReceived)
			{
				this->DataOpCond.wait(guard);
			}
			
			memcpy(recved_bytes, ReceiveBuffer, PAVO_REG_LEN + PAVO_CMD_LEN);
			
		}
		
		this->EnableDataTransfer();
		
		if((memcmp(PAVO_REG[reg_get], recved_bytes, PAVO_REG_LEN)!=0) || 
			(memcmp(cmd, recved_bytes + PAVO_REG_LEN, PAVO_RESPONSE_LEN)!=0))
		{
			return false;
		}
				
		return true;	
	}


	void pavo_driver::EnableDataTransfer()
	{
		pavo_cmd_packet_dual_t cmd_packet;
		memcpy(cmd_packet.reg, PAVO_REG[REG_DATA_EN_SET], PAVO_REG_LEN);
		memcpy(cmd_packet.cmd, PAVO_CMD[CMD_DATA_EN_SET_ENABLE], PAVO_CMD_LEN);

		unsigned char* packet_ptr = reinterpret_cast<unsigned char*>(&cmd_packet);

		this->SendData(packet_ptr, PAVO_REG_LEN + PAVO_CMD_LEN);

		boost::lock_guard<boost::mutex> guard(this->DataOpMtx);
		IsDataMode = true;
	}

	void pavo_driver::DisableDataTransfer()
	{
		pavo_cmd_packet_dual_t cmd_packet;
		memcpy(cmd_packet.reg, PAVO_REG[REG_DATA_EN_SET], PAVO_REG_LEN);
		memcpy(cmd_packet.cmd, PAVO_CMD[CMD_DATA_EN_SET_DISABLE], PAVO_CMD_LEN);

		unsigned char* packet_ptr = reinterpret_cast<unsigned char*>(&cmd_packet);

		this->SendData(packet_ptr, PAVO_REG_LEN + PAVO_CMD_LEN);
		//Sleep(10);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));;

		boost::lock_guard<boost::mutex> guard(this->DataOpMtx);
		IsDataMode = false;

	}

	int pavo_driver::PopFrame(int n)
	{
		boost::unique_lock<boost::mutex> guard(this->DataOpMtx);

		int del_number = std::min(n, this->FrameCount);
		
		pavo_response_scan_t* response_ptr ;
		int deleted = 0;

		while(deleted<del_number)
		{
			if( !this->PacketQue.dequeue(response_ptr))
				return deleted;
			if(IsWedgeData(response_ptr))
			{
				deleted++;
				this->FrameCount--;
			}
			delete response_ptr;
		}

		return deleted;
	}
	
}
