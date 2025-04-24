#include "docking_utils.h"

void printDeque(const std::deque<double>& dq) {
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < dq.size(); ++i) {
        ss << dq[i];
        if (i < dq.size() - 1) {
            ss << ",";
        }
    }
    ss << "]";
    ROS_INFO("%s", ss.str().c_str());
}


double radiansToDegrees360(double radians) {
    double degrees = radians * (180.0 / M_PI);  
    if (degrees < 0) {
        degrees += 360.0;  
    }
    return degrees;
}


Eigen::Matrix2d Rot2D(double yaw) {
    Eigen::Matrix2d rot;
    rot <<  cos(yaw), -sin(yaw),
            sin(yaw),  cos(yaw);
    return rot;
}


double wrapToPi(double angle) {
    angle = fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0) 
        angle += 2.0 * M_PI;
    return angle - M_PI;
}

double wrapTo2Pi(double angle) {
    angle = fmod(angle, 2.0 * M_PI);
    if (angle < 0) 
        angle += 2.0 * M_PI;
    return angle;
}

double sigma_e(double input) {
    if (input > 1)
        return 1;
    else if (input < -1)
        return -1;
    return input;
}
