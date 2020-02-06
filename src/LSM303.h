#ifndef LSM303_H
#define LSM303_H

#include <Arduino.h> // for byte data type

class LSM303
{
  public:
    template <typename T> struct vector
    {
      T x, y, z;
    };

    enum deviceType { device_DLH, device_DLM, device_DLHC, device_D, device_auto };
    enum sa0State { sa0_low, sa0_high, sa0_auto };

    vector<int16_t> a; // accelerometer readings
    vector<int16_t> m; // magnetometer readings
    vector<int16_t> m_max; // maximum magnetometer values, used for calibration
    vector<int16_t> m_min; // minimum magnetometer values, used for calibration

    byte last_status; // status of last I2C transmission

    LSM303(void);

    bool init(deviceType device = device_auto, sa0State sa0 = sa0_auto);
    deviceType getDeviceType(void) { return _device; }

    void enableDefault(void);

    void writeAccReg(byte reg, byte value);
    byte readAccReg(byte reg);
    void writeMagReg(byte reg, byte value);
    byte readMagReg(int reg);

    void writeReg(byte reg, byte value);
    byte readReg(int reg);

    void readAcc(void);
    void readMag(void);
    void read(void);

    void setTimeout(unsigned int timeout);
    unsigned int getTimeout(void);
    bool timeoutOccurred(void);

    float heading(void);
    template <typename T> float heading(vector<T> from);

    // vector functions
    template <typename Ta, typename Tb, typename To> static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
    template <typename Ta, typename Tb> static float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
    static void vector_normalize(vector<float> *a);

  private:
    deviceType _device; // chip type (D, DLHC, DLM, or DLH)
    byte acc_address;
    byte mag_address;

    static const int dummy_reg_count = 6;
    regAddr translated_regs[dummy_reg_count + 1]; // index 0 not used

    unsigned int io_timeout;
    bool did_timeout;

    int testReg(byte address, regAddr reg);
};

/*
Returns the angular difference in the horizontal plane between the
"from" vector and north, in degrees.

Description of heading algorithm:
Shift and scale the magnetic reading based on calibration data to find
the North vector. Use the acceleration readings to determine the Up
vector (gravity is measured as an upward acceleration). The cross
product of North and Up vectors is East. The vectors East and North
form a basis for the horizontal plane. The From vector is projected
into the horizontal plane and the angle between the projected vector
and horizontal north is returned.
*/
template <typename T> float LSM303::heading(vector<T> from)
{
    vector<int32_t> temp_m = {m.x, m.y, m.z};

    // subtract offset (average of min and max) from magnetometer readings
    temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
    temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
    temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

    // compute E and N
    vector<float> E;
    vector<float> N;
    vector_cross(&temp_m, &a, &E);
    vector_normalize(&E);
    vector_cross(&a, &E, &N);
    vector_normalize(&N);

    // compute heading
    float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / PI;
    if (heading < 0) heading += 360;
    return heading;
}

template <typename Ta, typename Tb, typename To> void LSM303::vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float LSM303::vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

#endif
