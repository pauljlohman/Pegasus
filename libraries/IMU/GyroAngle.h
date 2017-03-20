/*
    MPU9255

    GYROSCOPE
    FS_SEL | Full Scale Range   | LSB Sensitivity
    -------+--------------------+----------------
    0      | +/-  250 degrees/s | 131.072 LSB/deg/s
    1      | +/-  500 degrees/s |  65.536 LSB/deg/s
    2      | +/- 1000 degrees/s |  32.768 LSB/deg/s
    3      | +/- 2000 degrees/s |  16.384 LSB/deg/s

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

#ifndef GYROANGLE_H
#define GYROANGLE_H
#define DEBUG
class GyroAngle{
    float radToDegree = 57.295779513082320876798154814105;// 180 / PI
    float degreeToRad = 0.01745329251994329576923690768489;// PI / 180
    unsigned long dpu; // degrees per time unit
    float deltaDPU;
    float correctionWeight = 0.1;
    float gyroWeight = 1.0-correctionWeight;
    // x:57.60, y:35.87, z:29.05 offset value, tested at @ 2000
    float offsetX, offsetY, offsetZ;
    // true inverts axis
    bool invX, invY, invZ;
    
    float wrapAngle(float v){
        if(v > 180.0){
            return v - 360.0;
        }
        if(v < -180.0){
            return v + 360.0;
        }
        return v;
    };
    
    float angleDelta(float a, float b){
        a *= degreeToRad;
        b *= degreeToRad;
        float v = atan2f(sinf(a-b), cosf(a-b));
        return v * radToDegree;     
    };
    
    float angleInterpolate(float a, float b, float w){
        //http://stackoverflow.com/a/30129248
        a *= degreeToRad;
        b *= degreeToRad;
        float CS = (1.0-w) * cosf(a) + w * cosf(b);
        float SN = (1.0-w) * sinf(a) + w * sinf(b);
        float c = atan2f(SN,CS);
        return c * radToDegree;
    };
    
public:
    // angle rate
    float rateX, rateY, rateZ;
    // integrated angle rate and corrected using accelerometer
    float posX, posY, posZ;
    #ifdef DEBUG
        // integrated angle rate
        float rsumX, rsumY, rsumZ;
    #endif
    void config(unsigned short FullScaleRangeDPS,
                unsigned long deltaTime_UnitsPerSecond,
                float correction_weight,
                bool ix, bool iy, bool iz
                )
    {
        dpu = (1.0 / (FullScaleRangeDPS / 32768.0)) * deltaTime_UnitsPerSecond;
        correctionWeight = constrain(correction_weight, 0.0, 1.0);
        gyroWeight = 1.0 - correctionWeight;
        invX = ix;
        invY = iy;
        invZ = iz;
    };

    void init(float ox, float oy, float oz, float ax, float ay, float az){
        offsetX = invX ? -ox : ox;
        offsetY = invY ? -oy : oy;
        offsetZ = invZ ? -oz : oz;
        posX = ax;
        posY = ay;
        posZ = az;
    };

    void update(short x, short y, short z, unsigned long deltaTime){
        x = invX ? -x : x;
        y = invY ? -y : y;
        z = invZ ? -z : z;
        deltaDPU = dpu / float(deltaTime);
        rateX = (x - offsetX) / deltaDPU;
        rateY = (y - offsetY) / deltaDPU;
        rateZ = (z - offsetZ) / deltaDPU;
        #ifdef DEBUG
            // sum angle using rate only
            rsumX += rateX;
            rsumY += rateY;
            rsumZ += rateZ;
            rsumX = wrapAngle(rsumX);
            rsumY = wrapAngle(rsumY);
            rsumZ = wrapAngle(rsumZ);
        #endif
        posX += rateX;
        posY += rateY;
        posZ += rateZ;
        posX = wrapAngle(posX);
        posY = wrapAngle(posY);
        posZ = wrapAngle(posZ);        
    };

    void correct(float cx, float cy, float cz){
        /*
        posX = (posX * gyroWeight) + (cx * correctionWeight);
        posY = (posY * gyroWeight) + (cy * correctionWeight);
        posZ = (posZ * gyroWeight) + (cz * correctionWeight);
        */
        posX = angleInterpolate(posX, cx, correctionWeight);
        posY = angleInterpolate(posY, cy, correctionWeight);
        posZ = angleInterpolate(posZ, cz, correctionWeight);
    };
    void correctXY(float cx, float cy){
        /*
        posX = (posX * gyroWeight) + (cx * correctionWeight);
        posY = (posY * gyroWeight) + (cy * correctionWeight);
        */
        posX = angleInterpolate(posX, cx, correctionWeight);
        posY = angleInterpolate(posY, cy, correctionWeight);
    };
    void correctZ(float cz){
        //posZ = (posZ * gyroWeight) + (cz * correctionWeight);
        posZ = angleInterpolate(posZ, cz, correctionWeight);
    };
    
    void zeroZ(){
            posZ = 0.0;
        #ifdef DEBUG
            rsumZ = 0.0;
        #endif
    };
};

#endif // GYROANGLE_H
