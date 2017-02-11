/*
    MPU9255 - AK8963C - COMPASS    
    Full scale measurement range is ±4800µT
    Y axis runs front to back
    X axis runs left to right
    z axis runs bottom to top 
    
    
    X (ROLL)
        longitudinal axis runs from front to back
        positive value raises the left side and lowers the right
    Y (PITCH)
        lateral axis runs left to right
        positive value raises the nose and lowers the tail
    Z (YAW)
        vertical axis runs top to bottom
        positive value moves the nose right

*/

#ifndef COMPASSANGLE_H
#define COMPASSANGLE_H

#include "math.h"

class CompassAngle{
    float radToDegree = 180.0 / 3.1415926535897932384626433832795;
    
public:
    float heading;
    float offset;
    float upX = 0.0;
    float upY = 0.0;
    float upZ = 1.0;
    
    void vectorReject(float ax, float ay, float az, float bx, float by, float bz, float &_x, float &_y, float &_z){
        // project vector a onto plane b
        // a - ((a dot b)/(b dot b))*b
        float AdotB = dot(ax,ay,az,bx,by,bz);
        float BdotB = dot(bx,by,bz,bx,by,bz);
        float temp = AdotB / BdotB;
        float abx = bx * temp;
        float aby = by * temp;
        float abz = bz * temp;
        _x = ax - abx;
        _y = ay - aby;
        _z = az - abz;
    };
    float dot(float ax, float ay, float az, float bx, float by, float bz){
        return ax*bx + ay*by + az*bz;
    };
    void cross(float ax, float ay, float az, float bx, float by, float bz, float &_x, float &_y, float &_z){
        _x = ay*bz - az*by;
        _y = az*bx - ax*bz;
        _z = ax*by - ay*bx;
    };
    float vecLength(float &_x, float &_y, float &_z){
        return sqrt(_x*_x + _y*_y + _z*_z);
    };
    void vecNormalize(float &_x, float &_y, float &_z) {
        float m = vecLength(_x,_y,_z);
        _x /= m;
        _y /= m;
        _z /= m;
    };
    
    void update(short _x, short _y, short _z){
        float xf = float(_x);//-ox;
        float yf = float(_y);//-oy;
        float zf = float(_z);//-oz;   

        vecNormalize(xf,yf,zf);
        float rX, rY, rZ;
        // reject compass vector onto up vector
        vectorReject(xf, yf, zf, upX, upY, upZ, rX, rY, rZ);

        heading = atan2f(rX, rY) * radToDegree;
        //heading -= offset;
        //heading = wrapAngle(heading);

        //heading = atan2f(yf, sqrt(xf*xf + zf*zf)) * radToDegree;
    };
    
    float wrapAngle(float v){
        if(v > 180.0){
            return v - 360.0;
        }
        if(v < -180.0){
            return v + 360.0;
        }
        return v;
    };
};


#endif

/*
//get Quaternion between two vectors
Quaternion q;
vector a = crossproduct(v1, v2)
q.xyz = a;
q.w = sqrt((v1.Length ^ 2) * (v2.Length ^ 2)) + dotproduct(v1, v2)

rotating
around the Z-axis
    |cos ?   -sin ?   0| |x|   |x cos ? - y sin ?|   |x'|
    |sin ?    cos ?   0| |y| = |x sin ? + y cos ?| = |y'|
    |  0       0      1| |z|   |        z        |   |z'|

around the Y-axis
    | cos ?    0   sin ?| |x|   | x cos ? + z sin ?|   |x'|
    |   0      1       0| |y| = |         y        | = |y'|
    |-sin ?    0   cos ?| |z|   |-x sin ? + z cos ?|   |z'|

around the X-axis
    |1     0           0| |x|   |        x        |   |x'|
    |0   cos ?    -sin ?| |y| = |y cos ? - z sin ?| = |y'|
    |0   sin ?     cos ?| |z|   |y sin ? + z cos ?|   |z'|

*/