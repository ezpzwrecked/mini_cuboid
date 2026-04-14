#include "IO_handler.h"

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f // pi
#endif

// constructor
IO_handler::IO_handler(float Ts)
    : m_encoder(PA_8, PA_9, 4 * 2048)
    , m_a_out(PA_4)
    , m_d_out(PB_1)
    , m_button(PA_10)
    , m_spi(PA_12, PA_11, PA_1)
    , m_imu(m_spi, PB_0)
{
    // attach debounced key press function
    m_button.rise(callback(this, &IO_handler::button_pressed));
    m_button_pressed = false;

    m_d_out = 0;

    // reset encoder
    m_encoder.reset();

    // initialize imu
    m_imu.init_inav();
    m_imu.configuration();

    // initialize linear characteristics
    m_lc_gz2gz.init(-32768.0f, 32767.0f, -1000.0f * M_PIf / 180.0f, 1000.0f * M_PIf / 180.0f);
    m_lc_ax2ax.init(-16170.0f, 16729.0f, -9.81f, 9.81f);
    m_lc_ay2ay.init(-17034.0f, 15860.0f, -9.81f, 9.81f);
    m_lc_i2u.init(-15.0f, 15.0f, 0.0f, 1.0f);

    // differentiating low pass filter
    m_fil_diff.differentiatingLowPass1Init(1.0f / (2.0f * M_PIf * 40.0f), Ts);

    // low pass filters for complementary filter
    float m_tau = 1.0f;
    m_fil_ax.lowPass1Init(m_tau, Ts);
    m_fil_ay.lowPass1Init(m_tau, Ts);
    m_fil_gz.lowPass1Init(m_tau, Ts, 1.0f);
    //m_fil_phi_bd.integratorInit(Ts);
}

IO_handler::~IO_handler() {}

void IO_handler::update(void)
{
    // update encoder readings and calculate speed
    m_phi_fw = m_encoder.getAngleRad();
    m_phi_fw_vel = m_fil_diff(m_phi_fw);

    // apply linear characteristics to imu readings
    m_ax = m_lc_ax2ax(m_imu.readAcc_raw(1));
    m_ay = m_lc_ay2ay(-m_imu.readAcc_raw(0));
    m_gz = m_lc_gz2gz(m_imu.readGyro_raw(2));

    // calculate complementary filter for wheel angle
    m_phi_bd = atan2(m_fil_ax(m_ax), m_fil_ay(m_ay)) + m_fil_gz(m_gz) - M_PI / 4.0f;
}

float IO_handler::get_phi_fw(void) { return m_phi_fw; }

float IO_handler::get_phi_bd(void) { return m_phi_bd; }

float IO_handler::get_phi_fw_vel(void) { return m_phi_fw_vel; }

float IO_handler::get_ax(void) { return m_ax; }

float IO_handler::get_ay(void) { return m_ay; }

float IO_handler::get_gz(void) { return m_gz; }

void IO_handler::write_current(float i_des) { m_a_out = m_lc_i2u(i_des); }

void IO_handler::enable_escon(void) { m_d_out = 1; }

void IO_handler::disable_escon(void) { m_d_out = 0; }

bool IO_handler::get_and_reset_button_state(void)
{
    bool button_pressed = m_button_pressed;
    m_button_pressed = false;
    return button_pressed;
}

void IO_handler::button_pressed() { m_button_pressed = true; }
