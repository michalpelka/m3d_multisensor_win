#pragma once
// MESSAGE LD_CAMERA_TRIGGER PACKING

#define MAVLINK_MSG_ID_LD_CAMERA_TRIGGER 201

MAVPACKED(
typedef struct __mavlink_ld_camera_trigger_t {
 uint32_t request_token; /*< Token which will be returned with strobe message. Incremented with new images.*/
 uint16_t frequency; /*< Requested frequency of shutter trigger. For oneshot, request 0 .*/
}) mavlink_ld_camera_trigger_t;

#define MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN 6
#define MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_MIN_LEN 6
#define MAVLINK_MSG_ID_201_LEN 6
#define MAVLINK_MSG_ID_201_MIN_LEN 6

#define MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_CRC 122
#define MAVLINK_MSG_ID_201_CRC 122



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LD_CAMERA_TRIGGER { \
    201, \
    "LD_CAMERA_TRIGGER", \
    2, \
    {  { "request_token", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ld_camera_trigger_t, request_token) }, \
         { "frequency", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_ld_camera_trigger_t, frequency) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LD_CAMERA_TRIGGER { \
    "LD_CAMERA_TRIGGER", \
    2, \
    {  { "request_token", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ld_camera_trigger_t, request_token) }, \
         { "frequency", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_ld_camera_trigger_t, frequency) }, \
         } \
}
#endif

/**
 * @brief Pack a ld_camera_trigger message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param frequency Requested frequency of shutter trigger. For oneshot, request 0 .
 * @param request_token Token which will be returned with strobe message. Incremented with new images.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ld_camera_trigger_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t frequency, uint32_t request_token)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN];
    _mav_put_uint32_t(buf, 0, request_token);
    _mav_put_uint16_t(buf, 4, frequency);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN);
#else
    mavlink_ld_camera_trigger_t packet;
    packet.request_token = request_token;
    packet.frequency = frequency;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LD_CAMERA_TRIGGER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_CRC);
}

/**
 * @brief Pack a ld_camera_trigger message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param frequency Requested frequency of shutter trigger. For oneshot, request 0 .
 * @param request_token Token which will be returned with strobe message. Incremented with new images.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ld_camera_trigger_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t frequency,uint32_t request_token)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN];
    _mav_put_uint32_t(buf, 0, request_token);
    _mav_put_uint16_t(buf, 4, frequency);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN);
#else
    mavlink_ld_camera_trigger_t packet;
    packet.request_token = request_token;
    packet.frequency = frequency;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LD_CAMERA_TRIGGER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_CRC);
}

/**
 * @brief Encode a ld_camera_trigger struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ld_camera_trigger C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ld_camera_trigger_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ld_camera_trigger_t* ld_camera_trigger)
{
    return mavlink_msg_ld_camera_trigger_pack(system_id, component_id, msg, ld_camera_trigger->frequency, ld_camera_trigger->request_token);
}

/**
 * @brief Encode a ld_camera_trigger struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ld_camera_trigger C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ld_camera_trigger_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ld_camera_trigger_t* ld_camera_trigger)
{
    return mavlink_msg_ld_camera_trigger_pack_chan(system_id, component_id, chan, msg, ld_camera_trigger->frequency, ld_camera_trigger->request_token);
}

/**
 * @brief Send a ld_camera_trigger message
 * @param chan MAVLink channel to send the message
 *
 * @param frequency Requested frequency of shutter trigger. For oneshot, request 0 .
 * @param request_token Token which will be returned with strobe message. Incremented with new images.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ld_camera_trigger_send(mavlink_channel_t chan, uint16_t frequency, uint32_t request_token)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN];
    _mav_put_uint32_t(buf, 0, request_token);
    _mav_put_uint16_t(buf, 4, frequency);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER, buf, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_CRC);
#else
    mavlink_ld_camera_trigger_t packet;
    packet.request_token = request_token;
    packet.frequency = frequency;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER, (const char *)&packet, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_CRC);
#endif
}

/**
 * @brief Send a ld_camera_trigger message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ld_camera_trigger_send_struct(mavlink_channel_t chan, const mavlink_ld_camera_trigger_t* ld_camera_trigger)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ld_camera_trigger_send(chan, ld_camera_trigger->frequency, ld_camera_trigger->request_token);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER, (const char *)ld_camera_trigger, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_CRC);
#endif
}

#if MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ld_camera_trigger_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t frequency, uint32_t request_token)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, request_token);
    _mav_put_uint16_t(buf, 4, frequency);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER, buf, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_CRC);
#else
    mavlink_ld_camera_trigger_t *packet = (mavlink_ld_camera_trigger_t *)msgbuf;
    packet->request_token = request_token;
    packet->frequency = frequency;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER, (const char *)packet, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_CRC);
#endif
}
#endif

#endif

// MESSAGE LD_CAMERA_TRIGGER UNPACKING


/**
 * @brief Get field frequency from ld_camera_trigger message
 *
 * @return Requested frequency of shutter trigger. For oneshot, request 0 .
 */
static inline uint16_t mavlink_msg_ld_camera_trigger_get_frequency(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field request_token from ld_camera_trigger message
 *
 * @return Token which will be returned with strobe message. Incremented with new images.
 */
static inline uint32_t mavlink_msg_ld_camera_trigger_get_request_token(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a ld_camera_trigger message into a struct
 *
 * @param msg The message to decode
 * @param ld_camera_trigger C-struct to decode the message contents into
 */
static inline void mavlink_msg_ld_camera_trigger_decode(const mavlink_message_t* msg, mavlink_ld_camera_trigger_t* ld_camera_trigger)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ld_camera_trigger->request_token = mavlink_msg_ld_camera_trigger_get_request_token(msg);
    ld_camera_trigger->frequency = mavlink_msg_ld_camera_trigger_get_frequency(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN? msg->len : MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN;
        memset(ld_camera_trigger, 0, MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_LEN);
    memcpy(ld_camera_trigger, _MAV_PAYLOAD(msg), len);
#endif
}
