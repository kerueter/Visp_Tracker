/****************************************************************
 *****                                                      *****
 ***** Name: Keypoint.hpp                                   *****
 ***** Ver.: 1.00                                           *****
 ***** Date: 2016-03-17                                     *****
 ***** Auth: Kevin Rueter                                   *****
 ***** University of Osnabrueck                             *****
 ***** Germany                                              *****
 ***** Func: representaion of a keypoint from visp tracker  *****
 *****                                                      *****
 ******************************************************************/

#ifndef VISP_EXAMPLES_KEYPOINT_H
#define VISP_EXAMPLES_KEYPOINT_H


#include <visp3/core/vpPoint.h>

class Keypoint {
public:
    Keypoint(int id, vpPoint point);
    Keypoint(int id, double x, double y, double z);
    ~Keypoint();
    vpPoint getPoint();
    double getX();
    double getY();
    double getZ();
    int getID();
    void setPoint(vpPoint point);
    void setX(const double x);
    void setY(const double x);
    void setZ(const double x);

private:
    vpPoint point;
    int id;
};


#endif //VISP_EXAMPLES_KEYPOINT_H
