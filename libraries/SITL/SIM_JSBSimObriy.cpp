/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for JSBSim
*/

#include "SIM_JSBSimObriy.h"
#include "SIM_JSBSim.h"

#if HAL_SIM_JSBSIM_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <stddef.h>
#include <errno.h>
#include <cstring>

extern const AP_HAL::HAL& hal;

namespace SITL {

// the asprintf() calls are not worth checking for SITL
#pragma GCC diagnostic ignored "-Wunused-result"

static constexpr auto rc_port = 5500;
static constexpr auto fdm_ip = "127.0.0.1";

JSBSimObriy::JSBSimObriy(const char *frame_str) :
    Aircraft(frame_str),
    sock_control(false),
    sock_fgfdm(true),
    sock_rc_in(true),
    rc_in_started(false),
    rc_in_port(rc_port),
    initialised(false),
    opened_control_socket(false),
    opened_fdm_socket(false)
{
    control_port = 5505 + instance*10;
    fdm_port = 5504 + instance*10;

    printf("JSBSim backend started: control_port=%u fdm_port=%u\n",
           control_port, fdm_port);
}

/*
  open control socket to JSBSim
 */
bool JSBSimObriy::open_control_socket(void)
{
    if (opened_control_socket) {
        return true;
    }
    if (!sock_control.connect(fdm_ip, control_port)) {
        //fprintf(stdout, "Try to connection JSBSIM %d....\n", control_port);
        return false;
    }
    fprintf(stdout, "Opened JSBSim control socket\n");
    fflush(stdout);
    sock_control.set_blocking(false);
    opened_control_socket = true;

    return true;
}

/*
  open fdm socket from JSBSim
 */
bool JSBSimObriy::open_fdm_socket(void)
{
    if (opened_fdm_socket) {
        return true;
    }
    const auto open_action = [&](SocketAPM_native& socket, int port)
    {
        if (!socket.bind(fdm_ip, port)) {
            return false;
        }

        socket.set_blocking(false);
        socket.reuseaddress();
        return true;
    };

    opened_fdm_socket = open_action(sock_fgfdm, fdm_port);
    const auto opened_rc = open_action(sock_rc_in, rc_in_port);

    return opened_fdm_socket && opened_rc;
}


/*
  decode and send servos
*/
void JSBSimObriy::send_servos(const struct sitl_input &input)
{
    char *buf = nullptr;
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float throttle = filtered_servo_range(input, 2);
    float rudder   = filtered_servo_angle(input, 3);
    
    float wind_speed_fps = input.wind.speed / FEET_TO_METERS;
    asprintf(&buf,
             "set fcs/aileron-cmd-norm %f\n"
             "set fcs/elevator-cmd-norm %f\n"
             "set fcs/rudder-cmd-norm %f\n"
             "set fcs/throttle-cmd-norm %f\n"
             "set atmosphere/psiw-rad %f\n"
             "set atmosphere/wind-mag-fps %f\n"
             "set atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps %f\n"
             "set atmosphere/turbulence/milspec/severity %f\n"
             ,//"iterate 1\n",
             aileron, elevator, rudder, throttle,
             radians(input.wind.direction),
             wind_speed_fps,
             wind_speed_fps/3,
             input.wind.turbulence);
    ssize_t buflen = strlen(buf);
    ssize_t sent = sock_control.send(buf, buflen);
    free(buf);
    if (sent < 0) {
        if (errno != EAGAIN) {
            fprintf(stderr, "Fatal: Failed to send on control socket: %s\n",
                    strerror(errno));
            exit(1);
        }
        return;
    }
    if (sent < buflen) {
        fprintf(stderr, "Failed to send all bytes on control socket\n");
    }
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void JSBSimObriy::recv_fdm(const struct sitl_input &input)
{
    FGNetFDM fdm;
    float rc[16];
    do {
        const auto size = sock_rc_in.recv(&rc, sizeof(rc), 10);
        if (size == (int)(sizeof(float) * 16)) {
            rcin_chan_count = 12;
            //fprintf(stdout, "Read %ld RC bytes throttle: %f \n", size, rc[2]);
            std::memcpy(rcin, rc, sizeof(float) * 12);
        }
        
        while (sock_fgfdm.recv(&fdm, sizeof(fdm), 100) != sizeof(fdm)) {
            send_servos(input);
        }
        fdm.ByteSwap();
    } while (fdm.cur_time == time_now_us);

    accel_body = Vector3f(fdm.A_X_pilot, fdm.A_Y_pilot, fdm.A_Z_pilot) * FEET_TO_METERS;

    double p, q, r;
    SIM::convert_body_frame(degrees(fdm.phi), degrees(fdm.theta),
                             degrees(fdm.phidot), degrees(fdm.thetadot), degrees(fdm.psidot),
                             &p, &q, &r);
    gyro = Vector3f(p, q, r);

    velocity_ef = Vector3f(fdm.v_north, fdm.v_east, fdm.v_down) * FEET_TO_METERS;
    location.lat = RAD_TO_DEG_DOUBLE * fdm.latitude * 1.0e7;
    location.lng = RAD_TO_DEG_DOUBLE * fdm.longitude * 1.0e7;
    location.alt = fdm.agl * 100 + home.alt;
    dcm.from_euler(fdm.phi, fdm.theta, fdm.psi);
    airspeed = fdm.vcas * KNOTS_TO_METERS_PER_SECOND;
    airspeed_pitot = airspeed;

    // update magnetic field
    update_mag_field_bf();
    
    rpm[0] = fdm.rpm[0];
    rpm[1] = fdm.rpm[1];
    
    time_now_us = fdm.cur_time;
}

void JSBSimObriy::drain_control_socket()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    do {
        sock_rc_in.recv(buf, buflen, 0 );
        received = sock_control.recv(buf, buflen, 0);
    } while (received > 0);
}
/*
  update the JSBSim simulation by one time step
 */
void JSBSimObriy::update(const struct sitl_input &input)
{
    while (!initialised) {
        if (!open_control_socket() ||
            !open_fdm_socket()) {
            time_now_us = 1;
            return;
        }
        initialised = true;
    }
    send_servos(input);
    recv_fdm(input);
    adjust_frame_time(rate_hz);
    sync_frame_time();
    drain_control_socket();
}

} // namespace SITL

#endif  // HAL_SIM_JSBSIM_ENABLED
