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

class GyroAngle{
    unsigned long dpu; // degrees per time unit
    float deltaDPU;
    float correctionWeight = 0.1;
    float gyroWeight = 1.0-correctionWeight;

    public:
        // angle rate
        float rateX;
        float rateY;
        float rateZ;
        // accumulated angle
        float accX;
        float accY;
        float accZ;
        // corrected angle
        float posX;
        float posY;
        float posZ;
        // x:57.60, y:35.87, z:29.05 offset value, tested at @ 2000
        float offsetX;
        float offsetY;
        float offsetZ;
        // true inverts axis
        bool invX;
        bool invY;
        bool invZ;

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
            offsetZ = invX ? -oz : oz;
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

            accX += rateX;
            accY += rateY;
            accZ += rateZ;
            accX = wrapAngle(accX);
            accY = wrapAngle(accY);
            accZ = wrapAngle(accZ);

            posX += rateX;
            posY += rateY;
            posZ += rateZ;
            posX = wrapAngle(posX);
            posY = wrapAngle(posY);
            posZ = wrapAngle(posZ);
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

        void correct(float cx, float cy, float cz){
            //not used but long reference: http://www.starlino.com/imu_guide.html
            posX = (posX * gyroWeight) + (cx * correctionWeight);
            posY = (posY * gyroWeight) + (cy * correctionWeight);
            posZ = (posZ * gyroWeight) + (cz * correctionWeight);
        };
        void correctXY(float cx, float cy){
            posX = (posX * gyroWeight) + (cx * correctionWeight);
            posY = (posY * gyroWeight) + (cy * correctionWeight);
        };
        void correctZ(float cz){
            posZ = (posZ * gyroWeight) + (cz * correctionWeight);
        };


};

#endif // GYROANGLE_H
