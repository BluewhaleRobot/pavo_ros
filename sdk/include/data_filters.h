#include "pavo_types.h"
#include "sys_types.h"

class DataFilter
{
public:
    virtual pavo_response_scan_t* Filter(pavo_response_scan_t* data_buffer, int count);

    virtual pavo_response_pcd_t* Filter(pavo_response_pcd_t* data_buffer, int count);

    virtual ~DataFilter();
};


class MADataFilter : public DataFilter
{
public:
    pavo_response_scan_t* Filter(pavo_response_scan_t* data_buffer, int count, bool m_first);
    pavo_response_pcd_t* Filter(pavo_response_pcd_t* data_buffer, int count, bool m_first);

public:
    MADataFilter(int scale, int distance);

    int GetScale();
    void SetScale(int scale);

    uint16_t GetDistance();
    void SetDistance(uint16_t distance); 

private:
    //0: no Lpp qualified, 1:left, 2:middle, 3:right
    int ChooseLpp(uint16_t left_lpp, uint16_t middle_lpp, uint16_t right_lpp);
    int ChooseLppM(uint16_t left_lpp, uint16_t middle_lpp, uint16_t right_lpp);
    

    uint16_t LeftLpp(pavo_response_scan_t* data_buffer, int idx, int capacity);
   

    uint16_t MiddleLpp(pavo_response_scan_t* data_buffer, int idx, int capacity);
   

    uint16_t RightLpp(pavo_response_scan_t* data_buffer, int idx, int capacity);
    

private:
    int _point_scale;
    uint16_t _delta_distance;
    uint16_t _avg_left;
    uint16_t _avg_middle;
    uint16_t _avg_right;

};



