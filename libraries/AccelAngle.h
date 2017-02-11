/*
    MPU9255 - ACCELEROMETER
    AFS_SEL | Full Scale Range | LSB Sensitivity
    --------+------------------+----------------
    0       | +/-  2g          | 16384 LSB/g
    1       | +/-  4g          |  8192 LSB/g
    2       | +/-  8g          |  4096 LSB/g
    3       | +/- 16g          |  2048 LSB/g

    Sensitivity Change vs. Temperature
    -40°C to +85°C AFS_SEL:0  is ±0.026 %/°C

    Zero-G Level Change vs. Temperature

    X (ROLL)
        longitudinal axis runs from front to back
        positive value raises the left side and lowers the right
    Y (PITCH)
        lateral axis runs left to right
        positive value raises the nose and lowers the tail
    Z (YAW)
        vertical axis runs top to bottom
        positive value moves the nose right

    MPU9255 - AK8963C - COMPASS    
    Full scale measurement range is ±4800µT
    Y axis runs front to back
    X axis runs left to right
    z axis runs bottom to top 
    
*/

#ifndef ACCELANGLE_H
#define ACCELANGLE_H

#include "math.h"

class AccelAngle{
    float radToDegree = 180.0 / 3.1415926535897932384626433832795;
    // true inverts axis
    bool invX;
    bool invY;
    bool invZ; 
    
public:
    // angular position
    float x;
    float y;
    float z;
    // gravity total
    float g = 0;
    // resting position
    float ox;
    float oy;
    float oz;

    float vecLength(float &_x, float &_y, float &_z){
        return sqrt(_x*_x + _y*_y + _z*_z);
    };
    void vecNormalize(float &_x, float &_y, float &_z) {
        float m = vecLength(_x,_y,_z);
        _x /= m;
        _y /= m;
        _z /= m;
    };
    
    void config(bool _x, bool _y, bool _z){
        invX = _x;
        invY = _y;
        invZ = _z;
    };
    
    void setRestingPosition(float _x, float _y, float _z){
        ox = _x;
        oy = _y;
        oz = _z;
    };
    
    void update(short _x, short _y, short _z){
        float xf = float(_x)-ox;
        float yf = float(_y)-oy;
        float zf = float(_z);//-oz;
        
        // get total gravity and normalize
        //float g = vecLength(xf,yf,zf);        
        
        // roll relative to zf
        x = atan2f(yf, sqrt(xf*xf + zf*zf)) * radToDegree;

        // pitch relative to zf
        y = atan2f(-xf, sqrt(yf*yf + zf*zf)) * radToDegree;

        // yaw, need compass for this
        //http://phucly-blog.blogspot.com/2016/02/arduino-working-with-gy-9255-mpu-9255.html
        z = 0.0;//atan2f(yf, xf) * radToDegree;
        
        // attempt to provide range past 90, mostly good till close to 180 in either axis
        if(zf < 0){
            if(abs(x) > abs(y)){
                if(x > 0){
                    x = 180 - x;
                }else{
                    x = -180 - x;
                }
            }else{
                if(y > 0){
                    y = 180 - y ;
                }else{
                    y = -180 - y;
                }
            }
        }
        x = invX ? -x : x;
        y = invY ? -y : y;
        z = invX ? -z : z;
    };
};


#endif // ACCELANGLE_H

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