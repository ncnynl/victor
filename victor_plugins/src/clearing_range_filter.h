#ifndef CLEARING_LASER_SCAN_RANGE_FILTER_H
#define CLEARING_LASER_SCAN_RANGE_FILTER_H

#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

namespace laser_filters
{

class ClearingLaserScanRangeFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:

  double lower_threshold_ ;
  double upper_threshold_ ;

  bool configure()
  {
    lower_threshold_ = 0.0;
    upper_threshold_ = 100000.0;
    getParam("lower_threshold", lower_threshold_);
    getParam("upper_threshold", upper_threshold_) ;
    return true;
  }

  virtual ~ClearingLaserScanRangeFilter()
  {

  }

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
  {
    filtered_scan = input_scan;
    for (unsigned int i=0;
         i < input_scan.ranges.size();
         i++) // Need to check ever reading in the current scan
    {
      {
        if (filtered_scan.ranges[i] <= lower_threshold_ || filtered_scan.ranges[i] >= upper_threshold_)
        {
          filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
        }
      }
    }

    return true;
  }
} ;

}

#endif // CLEARING_LASER_SCAN_RANGE_FILTER_H