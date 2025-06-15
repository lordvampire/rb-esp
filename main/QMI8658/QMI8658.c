#include "QMI8658.h"
#include "lvgl.h"
#include "madgwick.h"

extern uint8_t EnableAttitudeMadgwick;

IMUdata Accel;
IMUdata Gyro;

IMUdata AccelFiltered;
IMUdata AccelFilteredMax;
IMUdata GyroFiltered;
IMUdata GyroBias;
IMUdata GyroCalibration;

float GPSAccelerationForAttitudeCompensation = 0.0;
float GFactor = 1.0;
float GFactorMax = 0;
float GFactorMin = 1.0;
uint8_t GFactorDirty = 0;
float AttitudePitch = 0;
float AttitudeRoll = 0;
float AttitudeYaw = 0;
float AttitudeYawDegreePerSecond = 0;
// 1.1.8 Improve reliabilty
float FilterMoltiplier = 3.0;
float FilterMoltiplierGyro = 10.0;
uint16_t lv_atan2(int x, int y);
// Define complementary filter constant (adjust as needed)
float AttitudeBalanceAlpha = 0.99;
#define DT 0.1 // Time step HZ

uint8_t Device_addr; // default for SD0/SA0 low, 0x6A if high
acc_scale_t acc_scale = ACC_RANGE_4G;
gyro_scale_t gyro_scale = GYR_RANGE_64DPS;
acc_odr_t acc_odr = acc_odr_norm_60;
gyro_odr_t gyro_odr = gyro_odr_norm_60;
sensor_state_t sensor_state = sensor_default;
lpf_t acc_lpf=LPF_MODE_0;
lpf_t gyro_lpf=LPF_MODE_0;


float accelScales, gyroScales;
float accelScales = 0;
uint8_t readings[12];
uint32_t reading_timestamp_us; // timestamp in arduino micros() time
/**
 * Inialize Wire and send default configs
 * @param addr I2C address of sensor, typically 0x6A or 0x6B
 */
void QMI8658_Init(void)
{
    uint8_t buf[1];
    Device_addr = QMI8658_L_SLAVE_ADDRESS;
    I2C_Read(Device_addr, QMI8658_REVISION_ID, buf, 1);
    printf("QMI8658 Device ID: %x\r\n", buf[0]); // Get chip id
    setState(sensor_running);

    setAccScale(acc_scale);
    setAccODR(acc_odr);
    setAccLPF(acc_lpf);
    switch (acc_scale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case ACC_RANGE_2G:
        accelScales = 2.0 / 32768.0;
        break;
    case ACC_RANGE_4G:
        accelScales = 4.0 / 32768.0;
        break;
    case ACC_RANGE_8G:
        accelScales = 8.0 / 32768.0;
        break;
    case ACC_RANGE_16G:
        accelScales = 16.0 / 32768.0;
        break;
    }

    setGyroScale(gyro_scale);
    setGyroODR(gyro_odr);
    setGyroLPF(gyro_lpf);
    switch (gyro_scale)
    {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case GYR_RANGE_16DPS:
        gyroScales = 16.0 / 32768.0;
        break;
    case GYR_RANGE_32DPS:
        gyroScales = 32.0 / 32768.0;
        break;
    case GYR_RANGE_64DPS:
        gyroScales = 64.0 / 32768.0;
        break;
    case GYR_RANGE_128DPS:
        gyroScales = 128.0 / 32768.0;
        break;
    case GYR_RANGE_256DPS:
        gyroScales = 256.0 / 32768.0;
        break;
    case GYR_RANGE_512DPS:
        gyroScales = 512.0 / 32768.0;
        break;
    case GYR_RANGE_1024DPS:
        gyroScales = 1024.0 / 32768.0;
        break;
    }

    GyroBias.x = 0;
    GyroBias.y = 0;
    GyroBias.z = 0;

    Madgwick_Init(10.0f, 0.1f);
}
void QMI8658_Loop(void)
{
    getAccelerometer();
    getGyroscope();
    getAttitude();
    getGFactor();
}

/**
 * Transmit one uint8_t of data to QMI8658.
 * @param addr address of data to be written
 * @param data the data to be written
 */
void QMI8658_transmit(uint8_t addr, uint8_t data)
{
    I2C_Write(Device_addr, addr, &data, 1);
}

/**
 * Receive one uint8_t of data from QMI8658.
 * @param addr address of data to be read
 * @return the uint8_t of data that was read
 */
uint8_t QMI8658_receive(uint8_t addr)
{
    uint8_t retval;
    I2C_Read(Device_addr, addr, &retval, 1);
    return retval;
}

/**
 * Writes data to CTRL9 (command register) and waits for ACK.
 * @param command the command to be executed
 */
void QMI8658_CTRL9_Write(uint8_t command)
{
    // transmit command
    QMI8658_transmit(QMI8658_CTRL9, command);

    // wait for command to be done
    while (((QMI8658_receive(QMI8658_STATUSINT)) & 0x80) == 0x00)
        ;
}

/**
 * Set output data rate (ODR) of accelerometer.
 * @param odr acc_odr_t variable representing new data rate
 */
void setAccODR(acc_odr_t odr)
{
    if (sensor_state != sensor_default) // If the device is not in the default state
    {
        uint8_t ctrl2 = QMI8658_receive(QMI8658_CTRL2);
        ctrl2 &= ~QMI8658_AODR_MASK; // clear previous setting
        ctrl2 |= odr;                // OR in new setting
        QMI8658_transmit(QMI8658_CTRL2, ctrl2);
    }
    acc_odr = odr;
}

/**
 * Set output data rate (ODR) of gyro.
 * @param odr gyro_odr_t variable representing new data rate
 */
void setGyroODR(gyro_odr_t odr)
{
    if (sensor_state != sensor_default)
    {
        uint8_t ctrl3 = QMI8658_receive(QMI8658_CTRL3);
        ctrl3 &= ~QMI8658_GODR_MASK; // clear previous setting
        ctrl3 |= odr;                // OR in new setting
        QMI8658_transmit(QMI8658_CTRL3, ctrl3);
    }
    gyro_odr = odr;
}

/**
 * Set scale of accelerometer output.
 * @param scale acc_scale_t variable representing new scale
 */
void setAccScale(acc_scale_t scale)
{
    if (sensor_state != sensor_default)
    {
        uint8_t ctrl2 = QMI8658_receive(QMI8658_CTRL2);
        ctrl2 &= ~QMI8658_ASCALE_MASK;           // clear previous setting
        ctrl2 |= scale << QMI8658_ASCALE_OFFSET; // OR in new setting
        QMI8658_transmit(QMI8658_CTRL2, ctrl2);
    }
    acc_scale = scale;
}

/**
 * Set scale of gyro output.
 * @param scale gyro_scale_t variable representing new scale
 */
void setGyroScale(gyro_scale_t scale)
{
    if (sensor_state != sensor_default)
    {
        uint8_t ctrl3 = QMI8658_receive(QMI8658_CTRL3);
        ctrl3 &= ~QMI8658_GSCALE_MASK;           // clear previous setting
        ctrl3 |= scale << QMI8658_GSCALE_OFFSET; // OR in new setting
        QMI8658_transmit(QMI8658_CTRL3, ctrl3);
    }
    gyro_scale = scale;
}

/**
 * Set new low-pass filter value for accelerometer
 * @param lp lpf_t variable representing new low-pass filter value
 */
void setAccLPF(lpf_t lpf)
{
    if (sensor_state != sensor_default)
    {
        uint8_t ctrl5 = QMI8658_receive(QMI8658_CTRL5);
        ctrl5 &= !QMI8658_ALPF_MASK;
        ctrl5 |= lpf << QMI8658_ALPF_OFFSET;
        ctrl5 |= 0x01; // turn on acc low pass filter
        QMI8658_transmit(QMI8658_CTRL5, ctrl5);
    }
    acc_lpf = lpf;
}

/**
 * Set new low-pass filter value for gyro
 * @param lp lpf_t variable representing new low-pass filter value
 */
void setGyroLPF(lpf_t lpf)
{
    if (sensor_state != sensor_default)
    {
        uint8_t ctrl5 = QMI8658_receive(QMI8658_CTRL5);
        ctrl5 &= !QMI8658_GLPF_MASK;
        ctrl5 |= lpf << QMI8658_GLPF_OFFSET;
        ctrl5 |= 0x10; // turn on gyro low pass filter
        QMI8658_transmit(QMI8658_CTRL5, ctrl5);
    }
}

/**
 * Set new state of QMI8658.
 * @param state new state to transition to
 */
void setState(sensor_state_t state)
{
    uint8_t ctrl1;
    switch (state)
    {
    case sensor_running:
        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        // enable 2MHz oscillator
        ctrl1 &= 0xFE;
        // enable auto address increment for fast block reads
        ctrl1 |= 0x40;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);

        // enable high speed internal clock,
        // acc and gyro in full mode, and
        // disable syncSample mode
        QMI8658_transmit(QMI8658_CTRL7, 0x43);

        // disable AttitudeEngine Motion On Demand
        QMI8658_transmit(QMI8658_CTRL6, 0x00);
        break;
    case sensor_power_down:
        // disable high speed internal clock,
        // acc and gyro powered down
        QMI8658_transmit(QMI8658_CTRL7, 0x00);

        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        // disable 2MHz oscillator
        ctrl1 |= 0x01;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);
        break;
    case sensor_locking:
        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        // enable 2MHz oscillator
        ctrl1 &= 0xFE;
        // enable auto address increment for fast block reads
        ctrl1 |= 0x40;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);

        // enable high speed internal clock,
        // acc and gyro in full mode, and
        // enable syncSample mode
        QMI8658_transmit(QMI8658_CTRL7, 0x83);

        // disable AttitudeEngine Motion On Demand
        QMI8658_transmit(QMI8658_CTRL6, 0x00);

        // disable internal AHB clock gating:
        QMI8658_transmit(QMI8658_CAL1_L, 0x01);
        QMI8658_CTRL9_Write(0x12);
        // re-enable clock gating
        QMI8658_transmit(QMI8658_CAL1_L, 0x00);
        QMI8658_CTRL9_Write(0x12);
        break;
    default:
        break;
    }
    sensor_state = state;
}

void getAccelerometer(void)
{

    uint8_t buf[6];
    I2C_Read(Device_addr, QMI8658_AX_L, buf, 6);
    Accel.x = (float)((int16_t)((buf[1] << 8) | (buf[0])));
    Accel.y = (float)((int16_t)((buf[3] << 8) | (buf[2])));
    Accel.z = (float)((int16_t)((buf[5] << 8) | (buf[4])));
    Accel.x = Accel.x * accelScales;
    Accel.y = Accel.y * accelScales;
    Accel.z = Accel.z * accelScales;

    AccelFiltered.x = (FilterMoltiplier * AccelFiltered.x + Accel.x) / (FilterMoltiplier + 1.0);
    AccelFiltered.y = (FilterMoltiplier * AccelFiltered.y + Accel.y) / (FilterMoltiplier + 1.0);
    AccelFiltered.z = (FilterMoltiplier * AccelFiltered.z + Accel.z) / (FilterMoltiplier + 1.0) - GPSAccelerationForAttitudeCompensation;
}

// Function to calculate roll (bank) from accelerometer
uint16_t getRollFromAccel(float axNotUsed, float ay, float az)
{

    int alpha = ay * 10.0;
    int beta = az * 10.0;
    if (alpha == 0 && beta == 0)
    {
        // Exception:
        return 0;
    }

    return lv_atan2(alpha, beta);
}

// Function to calculate pitch from accelerometer
int16_t getPitchFromAccel(float ax, float ay, float az)
{

    lv_sqrt_res_t res;
    lv_sqrt(ay * ay * 100.0 + az * az * 100.0, &res, 0x8000);

    int alpha = -ax * 10.0;
    if (alpha == 0 && res.i == 0)
    {
        // Exception:
        return 0;
    }

    uint16_t pitchReference = lv_atan2(alpha, res.i);
    if (pitchReference >= 90)
    {
        pitchReference = pitchReference - 360;
    }
    return pitchReference;
}

// Complementary filter for roll and pitch estimation
void updateAttitude(float ax, float ay, float az, float gx, float gy, float *roll, float *pitch)
{
    // Convert gyroscope readings from degrees per second to angle change
    float gyroRoll = *roll + gx * DT;
    float gyroPitch = *pitch + gy * DT;

    // Compute accelerometer roll and pitch
    float accelRoll = getRollFromAccel(ax, ay, az);
    float accelPitch = getPitchFromAccel(ax, ay, az);

    // Apply complementary filter
    *roll = AttitudeBalanceAlpha * gyroRoll + (1.0 - AttitudeBalanceAlpha) * accelRoll;
    *pitch = AttitudeBalanceAlpha * gyroPitch + (1.0 - AttitudeBalanceAlpha) * accelPitch;
}

// Guru Meditation Error: Core  0 panic'ed (IntegerDivideByZero). Exception was unhandled.
void getGyroscope(void)
{
    uint8_t buf[6];
    I2C_Read(Device_addr, QMI8658_GX_L, buf, 6);
    Gyro.x = (float)((int16_t)((buf[1] << 8) | (buf[0])));
    Gyro.y = (float)((int16_t)((buf[3] << 8) | (buf[2])));
    Gyro.z = (float)((int16_t)((buf[5] << 8) | (buf[4])));
    Gyro.x = Gyro.x * gyroScales;
    Gyro.y = Gyro.y * gyroScales;
    Gyro.z = Gyro.z * gyroScales;

    GyroFiltered.x = (FilterMoltiplierGyro * GyroFiltered.x + Gyro.x) / (FilterMoltiplierGyro + 1.0);
    GyroFiltered.y = (FilterMoltiplierGyro * GyroFiltered.y + Gyro.y) / (FilterMoltiplierGyro + 1.0);
    GyroFiltered.z = (FilterMoltiplierGyro * GyroFiltered.z + Gyro.z) / (FilterMoltiplierGyro + 1.0);
}
void getGFactor(void)
{
    lv_sqrt_res_t res;
    lv_sqrt(AccelFiltered.x * AccelFiltered.x * 100.0 + AccelFiltered.y * AccelFiltered.y * 100.0 + AccelFiltered.z * AccelFiltered.z * 100.0, &res, 0x8000);
    GFactor = res.i / 10.0;
    if (AccelFiltered.x < 0)
    {
        GFactor = GFactor * -1.0;
    }
    if (GFactor > GFactorMax)
    {
        printf("GMeter new Max %f %f", GFactor, GFactorMax);
        GFactorMax = GFactor;
        AccelFilteredMax.x = AccelFiltered.x;
        AccelFilteredMax.y = AccelFiltered.y;
        GFactorDirty = 1;
    }
    if (GFactor < GFactorMin)
    {
        printf("GMeter new min %f %f", GFactor, GFactorMin);
        GFactorMin = GFactor;
        GFactorDirty = 1;
    }
} 
int64_t esp_timer_get_time(void);
extern float invSampleFreq;

int16_t YawSamples[20];
int16_t YawSum;
uint8_t YawSampleCircle=0;

void getAttitude(void)
{
    /*
    static int64_t last = 0;
    int64_t now = esp_timer_get_time();
    int32_t elapsed = now - last;
    last = now;
    //invSampleFreq = 1.0/elapsed;
    printf("Elapsed: %ld\n",elapsed);
    */

    if (EnableAttitudeMadgwick == 0)
    {
        updateAttitude(AccelFiltered.z, AccelFiltered.y, -AccelFiltered.x, GyroFiltered.z + GyroBias.z + GyroCalibration.z, -(GyroFiltered.y + GyroBias.y + GyroCalibration.y), &AttitudeRoll, &AttitudePitch);
        AttitudeYawDegreePerSecond = GyroFiltered.x + GyroBias.x + GyroCalibration.x;
    }
    else
    {
        // AY <= -Z
        // AX <= Y
        // AZ <= -X
        // GX <= GY
        // GY <= GZ
        // GZ <= -GX
        float DEG2RAD = 3.14159265359f / 180.0f;

        // Example: convert gyro values
        float gz_rad = (GyroFiltered.z + GyroBias.z + GyroCalibration.z) * DEG2RAD /2.0;
        float gy_rad = (GyroFiltered.y + GyroBias.y + GyroCalibration.y) * DEG2RAD /2.0;
        float gx_rad = (GyroFiltered.x + GyroBias.x + GyroCalibration.x) * DEG2RAD/2.0;

        Madgwick_UpdateIMU(gz_rad, gy_rad, -gx_rad, -AccelFiltered.z/*PITCH*/, -AccelFiltered.y/*ROLL*/, AccelFiltered.x /*YAW*/);
        //Madgwick_UpdateIMU(gz_rad, gy_rad, -gx_rad, AccelFiltered.z, AccelFiltered.y, -AccelFiltered.x);
        // 3. Get Euler angles
        Madgwick_GetEuler(&AttitudeRoll, &AttitudePitch, &AttitudeYaw);


        /*
        static float lastAttitudeYaw = 0;
        AttitudeYawDegreePerSecond = (2*AttitudeYawDegreePerSecond+(AttitudeYaw-lastAttitudeYaw)*60)/3.0;
        lastAttitudeYaw=AttitudeYaw;
*/
        static float lastAttitudeYaw = 0;
        float Yaw10 = (AttitudeYaw-lastAttitudeYaw)*10.0;
        int16_t oldestYawSample = YawSamples[YawSampleCircle];
        YawSum = YawSum - oldestYawSample+Yaw10;
        YawSamples[YawSampleCircle]=Yaw10;
        YawSampleCircle = (YawSampleCircle+1)%20;
        AttitudeYawDegreePerSecond = -YawSum/2.5;
        lastAttitudeYaw=AttitudeYaw;
        /*
        printf("%3d ",YawSum);
        for(int i=0;i<20;i++){
printf("%3d",YawSamples[i]);
        }
        printf("\n");
        
        

        printf("AX:%3.1f AY:%3.1f AZ:%3.1f GX:%3.1f GY:%3.1f GZ:%3.1f ROLL:%3.1f PITCH:%3.1f YAW:%3.1f V:%3.1f\n",
         AccelFiltered.z, AccelFiltered.y, -AccelFiltered.x,
         gz_rad, gy_rad, -gx_rad,
        AttitudeRoll,AttitudePitch,AttitudeYaw,
        AttitudeYawDegreePerSecond
        );

*/
        
        AttitudeRoll = AttitudeRoll-180;
        AttitudePitch = -AttitudePitch;

    }
}