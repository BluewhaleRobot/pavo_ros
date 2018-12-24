#include <string.h>
#include <limits>
#include "../include/data_filters.h"



pavo_response_scan_t* DataFilter::Filter(pavo_response_scan_t* data_buffer, int count)
 {return data_buffer;}

pavo_response_pcd_t* DataFilter::Filter(pavo_response_pcd_t* data_buffer, int count)
{return data_buffer;}

DataFilter::~DataFilter() {}


MADataFilter::MADataFilter(int scale, int distance):
      _point_scale(scale),
      _delta_distance(distance)
    {}


pavo_response_scan_t* MADataFilter::Filter(pavo_response_scan_t* data_buffer, int count, bool m_first)
{
    if(count < this->_point_scale)
        return data_buffer;


    pavo_response_scan_t* copy_ptr = new pavo_response_scan_t[count];
    memcpy(copy_ptr, data_buffer, sizeof(pavo_response_scan_t)*count);
    

    for(int i=0; i<count; i++)
    {
        uint16_t left_lpp = LeftLpp(data_buffer, i, count);
	uint16_t middle_lpp = MiddleLpp(data_buffer, i, count);
        uint16_t right_lpp = RightLpp(data_buffer, i, count);
	
        int choice = 0;
	if(m_first)
		choice = ChooseLppM(left_lpp, middle_lpp, right_lpp );
	else
		choice = ChooseLpp(left_lpp, middle_lpp, right_lpp );
        switch(choice)
        {
        case 1:
	    copy_ptr[i].distance = this->_avg_left;
            break;
        case 2:
            copy_ptr[i].distance = this->_avg_middle;
            break;
        case 3:
            copy_ptr[i].distance = this->_avg_right;
            break;
        }
    }
    memcpy(data_buffer, copy_ptr, sizeof(pavo_response_scan_t)*count);
    delete[] copy_ptr;
    return data_buffer;
}

pavo_response_pcd_t* MADataFilter::Filter(pavo_response_pcd_t* data_buffer, int count, bool m_first)
{
    return data_buffer;
}


int MADataFilter::GetScale()
{
    return _point_scale;
}
void MADataFilter::SetScale(int scale)
{
    _point_scale = scale;
}

uint16_t MADataFilter::GetDistance()
{
    return _delta_distance;
}

void MADataFilter::SetDistance(uint16_t distance)
{
   _delta_distance = distance;
}


int MADataFilter::ChooseLpp(uint16_t left_lpp, uint16_t middle_lpp, uint16_t right_lpp)
{
    int choice = 0;
    uint16_t min = _delta_distance;
    if(left_lpp < min)
    {
        choice = 1;
        min = left_lpp;
    }

    if(middle_lpp < min)
    {
        choice = 2;
        min = middle_lpp;
    }

    if(right_lpp < min)
    {
        choice = 3;
        min = right_lpp;
    }

    return choice;
         
}

int MADataFilter::ChooseLppM(uint16_t left_lpp, uint16_t middle_lpp, uint16_t right_lpp)
{
    int choice = 0;
    uint16_t min = _delta_distance;

    if(middle_lpp < min)
    {
        choice = 2;
        min = middle_lpp;
	return choice;
    }

    if(left_lpp < min)
    {
        choice = 1;
        min = left_lpp;
    }



    if(right_lpp < min)
    {
        choice = 3;
        min = right_lpp;
    }

    return choice;
         
}

uint16_t MADataFilter::LeftLpp(pavo_response_scan_t* data_buffer, int idx, int capacity)
{
    uint16_t min = std::numeric_limits<uint16_t>::max();
    uint16_t max = 0;
    if(idx < _point_scale - 1)
        return _delta_distance + 1;

    this->_avg_left = 0;

    for(int i=0; i<_point_scale;i++)
    {
        if(data_buffer[idx-i].distance==0)
            return _delta_distance + 1;
            
        this->_avg_left += data_buffer[idx-i].distance;
        if(data_buffer[idx-i].distance < min)
        {
            min = data_buffer[idx-i].distance;
        }
        if(data_buffer[idx-i].distance > max)
        {
            max = data_buffer[idx-i].distance;
        }
    } 
    this->_avg_left /= _point_scale; 
    return max - min;
       
}

uint16_t MADataFilter::MiddleLpp(pavo_response_scan_t* data_buffer, int idx, int capacity)
{
    uint16_t min = std::numeric_limits<uint16_t>::max();
    uint16_t max = 0;
    if(idx < _point_scale/2 || idx + _point_scale/2 > capacity )
        return _delta_distance + 1;

    int start = idx - _point_scale/2;
    this->_avg_middle = 0;
    for(int i=0; i<_point_scale;i++)
    {
        if(data_buffer[start+i].distance==0)
            return _delta_distance + 1;
        this->_avg_middle += data_buffer[start+i].distance;
        if(data_buffer[start+i].distance < min)
        {
            min = data_buffer[start+i].distance;
        }
        if(data_buffer[start+i].distance > max)
        {
            max = data_buffer[start+i].distance;
        }
    } 
    this->_avg_middle /= _point_scale;
    return max - min;
}

uint16_t MADataFilter::RightLpp(pavo_response_scan_t* data_buffer, int idx, int capacity)
{
    uint16_t min = std::numeric_limits<uint16_t>::max();
    uint16_t max = 0;
    if(idx + _point_scale > capacity - 1 )
        return _delta_distance + 1;

    this->_avg_right = 0;
    for(int i=0; i<_point_scale;i++)
    {
        if(data_buffer[idx+i].distance==0)
            return _delta_distance + 1;
         this->_avg_right += data_buffer[idx+i].distance;
        if(data_buffer[idx+i].distance < min)
        {
            min = data_buffer[idx+i].distance;
        }
        if(data_buffer[idx+i].distance > max)
        {
            max = data_buffer[idx+i].distance;
        }
    } 
    this->_avg_right /= _point_scale;
    return max - min;
}
