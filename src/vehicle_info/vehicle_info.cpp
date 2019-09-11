#include "lattice_planner/vehicle_info/vehicle_info.h"

VehicleInfo::VehicleInfo(const double length,
                         const double width,
                         const double wheelBase,
                         const double safetyDistance) : wheelBase_(wheelBase), safetyDistance_(safetyDistance)
{
    if(length>width)
    {
        length_ = length;
        width_ = width;
    }
    else
    {
        length_ = width;
        width_ = length;
    }

    middlecircleCenters_.resize(2);
    footprintcircleCenters_.resize(4);
    middlecircleCenters_[0](0)=0.0;
    middlecircleCenters_[0](1)=length_/4.0;
    middlecircleCenters_[1](0)=0.0;
    middlecircleCenters_[1](1)=-length_/4.0;
    footprintcircleCenters_[0](0) = width_/4.0;
    footprintcircleCenters_[0](1) = 3*length_/8.0;
    footprintcircleCenters_[1](0) = -width_/4.0;
    footprintcircleCenters_[1](1) = 3*length_/8.0;
    footprintcircleCenters_[2](0) = -width_/4.0;
    footprintcircleCenters_[2](1) = -3*length_/8.0;
    footprintcircleCenters_[3](0) = width_/4.0;
    footprintcircleCenters_[3](1) = -3*length_/8.0;

    circumcircleRadius_ = length_/2.0;    // circumcircle radius
    middlecircleRadius_ = std::sqrt((width_/4)*(width_/4)+(length_/8)*(length_/8));

    if(length_/4.0>width_/2.0)
        footprintcircleRadius_ = length_/8.0;
    else
        footprintcircleRadius_ = width_/4.0;
}
