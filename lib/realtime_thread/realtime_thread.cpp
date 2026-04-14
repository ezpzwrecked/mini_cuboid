#include "realtime_thread.h"

#include <chrono>
#include <cstdint>

#include "DataLogger.h"
#include "GPA.h"

extern DataLogger myDataLogger;
extern GPA myGPA;

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f // pi
#endif

using namespace Eigen;
using namespace std::chrono;

// contructor for realtime_thread loop
realtime_thread::realtime_thread(IO_handler *io_handler, float Ts)
    : m_Thread(osPriorityHigh, 4096)
{
    m_Ts = Ts;                 // the sampling time
    m_IO_handler = io_handler; // a pointer to the io handler
    m_state = INIT;
    m_IO_handler->disable_escon();
    m_Timer.reset();
    m_Timer.start();
    m_integrator.integratorInit(Ts);
}

// decontructor for controller loop
realtime_thread::~realtime_thread() {}

// this is the main loop called every Ts with high priority
void realtime_thread::loop(void)
{
    const float km = 40.4e-3f; // silbrig 36.9
    float exc = 0.0f;
    //Matrix<float, 1, 2> K(-1.5739f, -0.0951f);
    //Matrix<float, 2, 1> x;
    Matrix<float, 1, 4> K4(-4.4738f, -0.5342f, -0.0106f, 0.0020f);
    Matrix<float, 4, 1> x;

    while (1) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        const float time = 1e-6f * (float)(duration_cast<microseconds>(m_Timer.elapsed_time()).count());
        // --------------------- THE LOOP ---------------------

        float w = myDataLogger.get_set_value(time); // get set values from the GUI
        myDataLogger.write_to_log(time, w, m_IO_handler->get_ax(), m_IO_handler->get_ay(), m_IO_handler->get_gz(), m_IO_handler->get_phi_bd(), 0.0f);
        // update sensor readings and readout values from io handler
        m_IO_handler->update();
        const float phi_fw_vel = m_IO_handler->get_phi_fw_vel();
        const float gz = m_IO_handler->get_gz();
        const float phi_bd = m_IO_handler->get_phi_bd();

        //x << phi_bd, gz;
        x << phi_bd, gz, phi_fw_vel, m_integrator(0-phi_fw_vel);

        // state machine
        float i_des = 0.0f;
        float M_des = 0.0f;
        const bool do_transition = m_IO_handler->get_and_reset_button_state();
        switch (m_state) {
            case INIT: {
                // ------------------- INIT -------------------
                // disable motor and wait for button press to switch to FLAT
                m_IO_handler->disable_escon();

                // switch to FLAT
                if (do_transition) {
                    m_state = FLAT;
                    m_IO_handler->enable_escon();
                }
                break;
            }
            case FLAT: {
                // ------------------- FLAT -------------------
                // switch to BALANCE
                if (do_transition) {
                    m_state = BALANCE;
                }
                break;
            }
            case BALANCE: {
                // ------------------- BALANCE ----------------
                // calculate control input
                M_des = -K4 * x;
                i_des = M_des / km;
                
                // switch to FLAT
                if (do_transition) {
                    m_state = FLAT;
                }
                break;
            }
            default:
                break;
        }

        // write current setpoint to motor
        i_des = saturate(i_des, -15.0f, 15.0f);
        m_IO_handler->write_current(i_des);

        // // GPA - do not overwrite exc if you want to excite via the GPA
        // exc = myGPA.update(i_des, phi_fw_vel); // GPA calculates future excitation exc(k+1)
    }
}

float realtime_thread::saturate(float x, float ll, float ul)
{
    if (x > ul)
        return ul;
    else if (x < ll)
        return ll;
    return x;
}

void realtime_thread::sendSignal() { m_Thread.flags_set(m_ThreadFlag); }

void realtime_thread::start_loop(void)
{
    m_Thread.start(callback(this, &realtime_thread::loop));
    m_Ticker.attach(callback(this, &realtime_thread::sendSignal), microseconds{static_cast<int64_t>(m_Ts * 1e6f)});
}
