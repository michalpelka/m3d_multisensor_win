#pragma once
// MESSAGE SET_ROTATION_VELOCITY PACKING

#define MAVLINK_MSG_ID_SET_ROTATION_VELOCITY 200

MAVPACKED(
typedef struct __mavlink_set_rotation_velocity_t {
 int16_t rpm; /*< velocity in RPM*/
}) mavlink_set_rotation_velocity_t;

#define MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN 2
#define MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_MIN_LEN 2
#define MAVLINK_MSG_ID_200_LEN 2
#define MAVLINK_MSG_ID_200_MIN_LEN 2

#define MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_CRC 152
#define MAVLINK_MSG_ID_200_CRC 152



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_ROTATION_VELOCITY { \
    200, \
    "SET_ROTATION_VELOCITY", \
    1, \
    {  { "rpm", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_set_rotation_velocity_t, rpm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_ROTATION_VELOCITY { \
    "SET_ROTATION_VELOCITY", \
    1, \
    {  { "rpm", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_set_rotation_velocity_t, rpm) }, \
         } \
}
#endif

/**
 * @brief Pack a set_rotation_velocity message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rpm velocity in RPM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_rotation_velocity_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN];
    _mav_put_int16_t(buf, 0, rpm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN);
#else
    mavlink_set_rotation_velocity_t packet;
    packet.rpm = rpm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ROTATION_VELOCITY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_CRC);
}

/**
 * @brief Pack a set_rotation_velocity message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rpm velocity in RPM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_rotation_velocity_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN];
    _mav_put_int16_t(buf, 0, rpm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN);
#else
    mavlink_set_rotation_velocity_t packet;
    packet.rpm = rpm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ROTATION_VELOCITY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_CRC);
}

/**
 * @brief Encode a set_rotation_velocity struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_rotation_velocity C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_rotation_velocity_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_rotation_velocity_t* set_rotation_velocity)
{
    return mavlink_msg_set_rotation_velocity_pack(system_id, component_id, msg, set_rotation_velocity->rpm);
}

/**
 * @brief Encode a set_rotation_velocity struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_rotation_velocity C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_rotation_velocity_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_rotation_velocity_t* set_rotation_velocity)
{
    return mavlink_msg_set_rotation_velocity_pack_chan(system_id, component_id, chan, msg, set_rotation_velocity->rpm);
}

/**
 * @brief Send a set_rotation_velocity message
 * @param chan MAVLink channel to send the message
 *
 * @param rpm velocity in RPM
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_rotation_velocity_send(mavlink_channel_t chan, int16_t rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN];
    _mav_put_int16_t(buf, 0, rpm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY, buf, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_CRC);
#else
    mavlink_set_rotation_velocity_t packet;
    packet.rpm = rpm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY, (const char *)&packet, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_CRC);
#endif
}

/**
 * @brief Send a set_rotation_velocity message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_rotation_velocity_send_struct(mavlink_channel_t chan, const mavlink_set_rotation_velocity_t* set_rotation_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_rotation_velocity_send(chan, set_rotation_velocity->rpm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY, (const char *)set_rotation_velocity, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_rotation_velocity_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, rpm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY, buf, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_CRC);
#else
    mavlink_set_rotation_velocity_t *packet = (mavlink_set_rotation_velocity_t *)msgbuf;
    packet->rpm = rpm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY, (const char *)packet, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_ROTATION_VELOCITY UNPACKING


/**
 * @brief Get field rpm from set_rotation_velocity message
 *
 * @return velocity in RPM
 */
static inline int16_t mavlink_msg_set_rotation_velocity_get_rpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Decode a set_rotation_velocity message into a struct
 *
 * @param msg The message to decode
 * @param set_rotation_velocity C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_rotation_velocity_decode(const mavlink_message_t* msg, mavlink_set_rotation_velocity_t* set_rotation_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_rotation_velocity->rpm = mavlink_msg_set_rotation_velocity_get_rpm(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN? msg->len : MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN;
        memset(set_rotation_velocity, 0, MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_LEN);
    memcpy(set_rotation_velocity, _MAV_PAYLOAD(msg), len);
#endif
}
