#include "pavo_types.h"
#include "sys_types.h"
#include <vector>

class MADataFilter
{
public:
    MADataFilter();
    pavo_response_scan_t* Filter(pavo_response_scan_t* data_buffer, int count, bool m_first);
    pavo_response_pcd_t* Filter(pavo_response_pcd_t* data_buffer, int count, bool m_first);
    void EnableTailFilter(int method);

private:  
    pavo_response_scan_t*  HandlTialFilter(pavo_response_scan_t* data_buffer, int count);

    pavo_response_scan_t*  HandlTialFilterWithRatio(pavo_response_scan_t* data_buffer, int count);

    pavo_response_scan_t*  HandlTialFilterWithDistance_1(pavo_response_scan_t* data_buffer, int count);

    pavo_response_scan_t*  HandlTialFilterWithDistance_2(pavo_response_scan_t* data_buffer, int count);

    pavo_response_scan_t*  HandlTialFilterWithDistanceForContinuity(pavo_response_scan_t* data_buffer, int count);

    pavo_response_scan_t*  HandlTialFilterWithAngle(pavo_response_scan_t* data_buffer, int count);

    uint16_t SearchSkipPoint(pavo_response_scan_t* data_buffer, int idx, int capacity);

    uint16_t SearchContinuityPoint(pavo_response_scan_t* data_buffer, int idx, int capacity);

    uint16_t HandlePoint(pavo_response_scan_t* data_buffer, int start,int end, int capacity);

    void SetWindow(int size);

    void SetDistance(double size);

    void SetIntensity(double size);

private:
    uint16_t _window_size;
    double   _distance_diff;
    double   _intensity_diff;
    std::vector<int> _window_vector;
    int  tailfilter_method;
    int  tailfilter_method_angle;
};



