/****************************************************************************
 * Custom MAVLink Stream for pwm_control
 ****************************************************************************/

#ifndef MAVLinkStreamPWM_CONTROL_HPP
#define MAVLinkStreamPWM_CONTROL_HPP

#include <uORB/topics/pwm_control.h>
#include "mavlink/common/mavlink_msg_pwm_control.h"

class MavlinkStreamPWM_CONTROL : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) {
        return new MavlinkStreamPWM_CONTROL(mavlink);
    }

    static constexpr const char *get_name_static() { return "PWM_CONTROL"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_PWM_CONTROL; }

    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override
    {
        return _sub.advertised() ? (MAVLINK_MSG_ID_PWM_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
    }

private:
    explicit MavlinkStreamPWM_CONTROL(Mavlink *mavlink)
        : MavlinkStream(mavlink) {}

    uORB::Subscription _sub{ORB_ID(pwm_control)};

    bool send() override
    {
        pwm_control_s msg;
        if (_sub.update(&msg)) {
            mavlink_pwm_control_t m{};
            m.time_usec = msg.timestamp;
            m.port      = msg.port;
            m.duty      = msg.duty;
            m.frequency = msg.frequency;

            mavlink_msg_pwm_control_send_struct(_mavlink->get_channel(), &m);
            return true;
        }
        return false;
    }
};
#endif
