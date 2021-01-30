#include <lps25h.h>

Lps25h::Lps25h(I2cConn& i2c_conn) : PololuAltImu(i2c_conn)
{
    _setup(LPS25H_CONFIG_MAP, LPS25H_READ_MAP);
}

// Returns (atmospheric) pressure [Pa]
double Lps25h::get_pressure()
{
    uint8_t* buf = _get_buffer(LPS25H_REG_PRESS_OUT_XL);

    int32_t raw_pres = ((buf[LPS25H_BUF_PRESS_OUT_H] << 16) | (buf[LPS25H_BUF_PRESS_OUT_L] << 8)) | buf[LPS25H_BUF_PRESS_OUT_XL];
    double pres = double(raw_pres) * LPS25H_PRES_SCALE / LPS25H_PRES_RESOLUTION;

    return pres;
}

// Returns temperature [C]
double Lps25h::get_temperature()
{
    uint8_t* buf = _get_buffer(LPS25H_REG_TEMP_OUT_P_L);

    int16_t raw_temp = (buf[LPS25H_BUF_TEMP_OUT_P_H] << 8) | buf[LPS25H_BUF_TEMP_OUT_P_L];
    double temp = double(raw_temp) / LPS25H_TEMP_RESOLUTION + LPS25H_TEMP_OFFSET;

    return temp;
}
