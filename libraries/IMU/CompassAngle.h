/*
    MPU9255 - AK8963C - COMPASS    
    Full scale measurement range is ±4800µT
    Y axis runs front to back
    X axis runs left to right
    z axis runs bottom to top 
    
    (YAW)
        positive value moves the nose right

*/

#ifndef COMPASSANGLE_H
#define COMPASSANGLE_H

#include "math.h"

class CompassAngle{
    float radToDegree = 180.0 / 3.1415926535897932384626433832795;
    float Xoffset, Yoffset, Zoffset;
    float Xscale, Yscale, Zscale;
    float yawOffset = 0.0;
    
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
    float wrapAngle(float v){
        if(v > 180.0){
            return v - 360.0;
        }
        if(v < -180.0){
            return v + 360.0;
        }
        return v;
    };
    
public:
    float yaw;
    //float mx, my, mz; //expose for debug purposes
    //float rx, ry, rz; //expose for debug purposes
    
    void config(float xo, float yo, float zo, float xs, float ys, float zs){
        Xoffset = xo;
        Yoffset = yo;
        Zoffset = zo;
        Xscale = xs;
        Yscale = ys;
        Zscale = zs;
    };
    void init(float offset){
        yawOffset = offset;
    };
    
    void update(short _x, short _y, short _z, short ux, short uy, short uz){
        _x = (_x - Xoffset) * Xscale;
        _y = (_y - Yoffset) * Yscale;
        _z = (_z - Zoffset) * Zscale;
        // transform compass to align with acc on mpu-9255
        float mx, my, mz;
        mx = _y*-1;
        my = _x*-1;
        mz = _z;        
        vecNormalize(mx, my, mz);
        
        float upX = float(ux);
        float upY = float(uy);
        float upZ = float(uz);
        vecNormalize(upX, upY, upZ);

        // reject compass vector onto up vector plane
        float rx, ry, rz;
        vectorReject(mx, my, mz, upX, upY, upZ, rx, ry, rz);
        yaw = atan2f(rx, ry) * radToDegree;
        yaw = yaw * -1; //invert output to get clockwise behavior, complies with common flight scheme
        yaw = wrapAngle(yaw - yawOffset);
        
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
