/****************************************************************
 *****                                                      *****
 ***** Name: Vector.hpp                                     *****
 ***** Ver.: 1.00                                           *****
 ***** Date: 2016-03-11                                     *****
 ***** Auth: Kevin Rueter                                   *****
 ***** University of Osnabrueck                             *****
 ***** Germany                                              *****
 ***** Func: Representation of a vector                     *****
 *****                                                      *****
 ****************************************************************/

#ifndef VISP_EXAMPLES_QUARTERNION_H
#define VISP_EXAMPLES_QUARTERNION_H

class Vector {
public:
    Vector(float x, float y, float z);
    ~Vector();
    float getX();
    float getY();
    float getZ();
private:
    float x;
    float y;
    float z;
};
#endif //VISP_EXAMPLES_QUARTERNION_H
