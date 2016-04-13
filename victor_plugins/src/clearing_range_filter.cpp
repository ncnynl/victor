#include "clearing_range_filter.h"
#include "pluginlib/class_list_macros.h"

using namespace laser_filters;

PLUGINLIB_REGISTER_CLASS(ClearingLaserScanRangeFilter, laser_filters::ClearingLaserScanRangeFilter, filters::FilterBase<sensor_msgs::LaserScan>)