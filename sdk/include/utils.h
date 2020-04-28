#pragma once

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>

#include <queue>
#include <deque>
#include <algorithm>
#include <string>

//#define Degree2Radians(x) ((x) * 3.1415926525 / 180.0)
const double pi = std::acos(-1);

#define Degree2Radians(x) ((x) * pi / 180.0)

const int NUM_ROT_ANGLES = 36001;

namespace pavo
{

	template<typename T>
	class SynchronizedQueue
	{
	public:
		
		SynchronizedQueue () :
		  queue_(), mutex_(), cond_(), request_to_end_(false), enqueue_data_(true) 
		{ 
		}
		  
		void enqueue (const T& data)
		{
			boost::unique_lock<boost::mutex> lock (mutex_);
			
			if (enqueue_data_)
			{
				queue_.push (data);
				cond_.notify_one ();
			}
		}
		
		bool dequeue (T& result)
		{
			boost::unique_lock<boost::mutex> lock (mutex_);
			
			while (queue_.empty () && (!request_to_end_))
			{
				cond_.wait (lock);
			}
			
			if (request_to_end_)
			{
				doEndActions ();
				return false;
			}
			
			result = queue_.front ();
			queue_.pop ();
			
			return true;
		}
		
		void stopQueue ()
		{
			boost::unique_lock<boost::mutex> lock (mutex_);
			request_to_end_ = true;
			cond_.notify_one ();
		}
		
		unsigned int size ()
		{
			boost::unique_lock<boost::mutex> lock (mutex_);
			return static_cast<unsigned int> (queue_.size ());
		}
		
		bool isEmpty () const
		{
			boost::unique_lock<boost::mutex> lock (mutex_);
			return (queue_.empty ());
		}
	
	private:
      
		void doEndActions ()
		{
			enqueue_data_ = false;
			
			while (!queue_.empty ())
			{
				queue_.pop ();
			}
		}

		std::queue<T> queue_;              // Use STL queue to store data
		mutable boost::mutex mutex_;       // The mutex to synchronise on
		boost::condition_variable cond_;   // The condition to wait for
		bool request_to_end_;
		bool enqueue_data_;
	};



	class SinuLookupTable
	{
	public:
		static SinuLookupTable* Instance()
		{
			if(instance_== NULL)
				instance_ = new SinuLookupTable();
			return instance_;
		}
		
		double CosValue(int angle) //1/100 degree
		{
			int angle_mod = angle % (NUM_ROT_ANGLES - 1);
			return cos_lookup_table_[angle_mod];
		}
		double SinValue(int angle) //1/100 degree
		{
			int angle_mod = angle % (NUM_ROT_ANGLES - 1);
			return sin_lookup_table_[angle_mod];
		}

	private:
		SinuLookupTable(int size = NUM_ROT_ANGLES)
		{
			InitTables(size);
		}

		void InitTables(int size)
		{
			if (cos_lookup_table_.size() == 0 || sin_lookup_table_.size() == 0)
			{
				cos_lookup_table_.resize(size);
				sin_lookup_table_.resize(size);
				for (int i = 0; i < size; i++)
				{
					double rad = Degree2Radians(i / 100.0);
					cos_lookup_table_[i] = std::cos(rad);
					sin_lookup_table_[i] = std::sin(rad);
				}
			}
		}
	private:
		static SinuLookupTable* instance_;
		std::vector<double> cos_lookup_table_;
		std::vector<double> sin_lookup_table_;

	};
	
	
	class NetCoverter
	{
		
	public:
		static bool String2Bytes(const std::string& ip_str, char* ip_bytes)
		{
			if(ip_str.empty())  //empty string
				return false;
			
			if (count(ip_str.begin(), ip_str.end(), '.') != 3)
				return false;

			std::string ip_str_copy(ip_str);
			TrimSpace(ip_str_copy); //remove leading and tail space
			
			std::string str_tmp;
			std::size_t prev_pos = 0;
			
			int dot_num = 0;
			int byte_single = 0;
			char tmp_bytes[4];
			
			for(std::size_t i=0; i<ip_str_copy.length(); i++)
			{
				if(ip_str_copy[i]=='.')
				{
					if(dot_num++ > 3)
						return false;
					
					str_tmp = ip_str_copy.substr(prev_pos, i- prev_pos);
					prev_pos = i+1;
					
					byte_single = atoi(str_tmp.c_str());  //invalid value  
					if(byte_single > 255)
						return false;
					tmp_bytes[dot_num-1] = static_cast<char>(byte_single);         
					
					continue;
				}
				if( (ip_str_copy[i]<'0') || (ip_str_copy[i] > '9') ) //invalid character
					return false;
				
			}
			
			str_tmp = ip_str_copy.substr(prev_pos);
			byte_single = atoi(str_tmp.c_str());   
			tmp_bytes[dot_num] = static_cast<char>(byte_single);
			
			memcpy(ip_bytes, tmp_bytes, 4);
			
			return true;
		}
		
		static bool Bytes2String(std::string& ip_str, const char* ip_bytes)
		{	
			if(ip_bytes == 0)
				return false;
			
			//for(int i=0; i<4; i++)
			//{
			//	int t_int = static_cast<unsigned int>(ip_bytes[i]);
			//	if(t_int < 0)
			//		return false;
			//}
			
			ip_str = std::to_string(static_cast<unsigned char>(ip_bytes[0])) + "." +
						std::to_string(static_cast<unsigned char>(ip_bytes[1])) + "." +
						std::to_string(static_cast<unsigned char>(ip_bytes[2])) + "." +
						std::to_string(static_cast<unsigned char>(ip_bytes[3]));
			return true;
		}
		
		static bool Port2Bytes(const uint16_t& port_int, char* port_bytes)  //from uint16_t to char[4]
		{
			if(port_bytes == nullptr)
				return false;
			
			memset(port_bytes, 0, 4);
			port_bytes[2] = (port_int & 0xFF00) >> 8;
			port_bytes[3] = (port_int & 0x00FF);
			
			return true;
		}
		
		static bool Bytes2Port(uint16_t& port_int, const char* port_bytes)
		{
			if(port_bytes == nullptr)
				return false;
			
			port_int = (static_cast<unsigned char>(port_bytes[2]) << 8)  + static_cast<unsigned char>( port_bytes[3]);
			
			return true;
		}
		
		
	private:
		static std::string& TrimSpace(std::string &str)
		{
			if (str.empty()) 
			{
				return str;
			}
			str.erase(0, str.find_first_not_of(" "));  //trim white space
			str.erase(str.find_last_not_of(" ") + 1);

			str.erase(0, str.find_first_not_of("\t"));  //trim tab
			str.erase(str.find_last_not_of("\t") + 1);

			return str;
		}
		
	};
}


