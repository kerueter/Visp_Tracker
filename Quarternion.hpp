/****************************************************************
 *****                                                      *****
 ***** Name: Quarternion.hpp                                *****
 ***** Ver.: 1.00                                           *****
 ***** Date: 2016-03-11                                     *****
 ***** Auth: Kevin Rueter                                   *****
 ***** University of Osnabrueck                             *****
 ***** Germany                                              *****
 ***** Func: Representation of a quarternion                *****
 *****                                                      *****
 ****************************************************************/

#ifndef VISP_EXAMPLES_QUARTERNION_H
#define VISP_EXAMPLES_QUARTERNION_H

class Quarternion {
public:
    Quarternion(float x, float y, float z, float w);
    ~Quarternion();
    float getX();
    float getY();
    float getZ();
    float getW();
private:
    float x;
    float y;
    float z;
    float w;
};
#endif //VISP_EXAMPLES_QUARTERNION_H
