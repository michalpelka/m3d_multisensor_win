#pragma once
// MESSAGE UNIT_ENCODERS PACKING

#define MAVLINK_MSG_ID_UNIT_ENCODERS 203

MAVPACKED(
typedef struct __mavlink_unit_encoders_t {
 uint64_t ts; /*< Hardware timestam*/
 float encoder0; /*< Encoder Channel 0*/
 float encoder1; /*< Encoder Channel 1*/
 float encoder2; /*< Encoder Channel 2*/
}) mavlink_unit_encoders_t;

#define MAVLINK_MSG_ID_UNIT_ENCODERS_LEN 20
#define MAVLINK_MSG_ID_UNIT_ENCODERS_MIN_LEN 20
#define MAVLINK_MSG_ID_203_LEN 20
#define MAVLINK_MSG_ID_203_MIN_LEN 20

#define MAVLINK_MSG_ID_UNIT_ENCODERS_CRC 181
#define MAVLINK_MSG_ID_203_CRC 181



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UNIT_ENCODERS { \
    203, \
    "UNIT_ENCODERS", \
    4, \
    {  { "ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_unit_encoders_t, ts) }, \
         { "encoder0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_unit_encoders_t, encoder0) }, \
         { "encoder1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_unit_encoders_t, encoder1) }, \
         { "encoder2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_unit_encoders_t, encoder2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UNIT_ENCODERS { \
    "UNIT_ENCODERS", \
    4, \
    {  { "ts", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_unit_encoders_t, ts) }, \
         { "encoder0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_unit_encoders_t, encoder0) }, \
         { "encoder1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_unit_encoders_t, encoder1) }, \
         { "encoder2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_unit_encoders_t, encoder2) }, \
         } \
}
#endif

/**
 * @brief Pack a unit_encoders message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ts Hardware timestam
 * @param encoder0 Encoder Channel 0
 * @param encoder1 Encoder Channel 1
 * @param encoder2 Encoder Channel 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_unit_encoders_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t ts, float encoder0, float encoder1, float encoder2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UNIT_ENCODERS_LEN];
    _mav_put_uint64_t(buf, 0, ts);
    _mav_put_float(buf, 8, encoder0);
    _mav_put_float(buf, 12, encoder1);
    _mav_put_float(buf, 16, encoder2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UNIT_ENCODERS_LEN);
#else
    mavlink_unit_encoders_t packet;
    packet.ts = ts;
    packet.encoder0 = encoder0;
    packet.encoder1 = encoder1;
    packet.encoder2 = encoder2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UNIT_ENCODERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UNIT_ENCODERS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UNIT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_CRC);
}

/**
 * @brief Pack a unit_encoders message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ts Hardware timestam
 * @param encoder0 Encoder Channel 0
 * @param encoder1 Encoder Channel 1
 * @param encoder2 Encoder Channel 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_unit_encoders_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t ts,float encoder0,float encoder1,float encoder2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UNIT_ENCODERS_LEN];
    _mav_put_uint64_t(buf, 0, ts);
    _mav_put_float(buf, 8, encoder0);
    _mav_put_float(buf, 12, encoder1);
    _mav_put_float(buf, 16, encoder2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UNIT_ENCODERS_LEN);
#else
    mavlink_unit_encoders_t packet;
    packet.ts = ts;
    packet.encoder0 = encoder0;
    packet.encoder1 = encoder1;
    packet.encoder2 = encoder2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UNIT_ENCODERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UNIT_ENCODERS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UNIT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_CRC);
}

/**
 * @brief Encode a unit_encoders struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param unit_encoders C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_unit_encoders_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_unit_encoders_t* unit_encoders)
{
    return mavlink_msg_unit_encoders_pack(system_id, component_id, msg, unit_encoders->ts, unit_encoders->encoder0, unit_encoders->encoder1, unit_encoders->encoder2);
}

/**
 * @brief Encode a unit_encoders struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param unit_encoders C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_unit_encoders_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_unit_encoders_t* unit_encoders)
{
    return mavlink_msg_unit_encoders_pack_chan(system_id, component_id, chan, msg, unit_encoders->ts, unit_encoders->encoder0, unit_encoders->encoder1, unit_encoders->encoder2);
}

/**
 * @brief Send a unit_encoders message
 * @param chan MAVLink channel to send the message
 *
 * @param ts Hardware timestam
 * @param encoder0 Encoder Channel 0
 * @param encoder1 Encoder Channel 1
 * @param encoder2 Encoder Channel 2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_unit_encoders_send(mavlink_channel_t chan, uint64_t ts, float encoder0, float encoder1, float encoder2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UNIT_ENCODERS_LEN];
    _mav_put_uint64_t(buf, 0, ts);
    _mav_put_float(buf, 8, encoder0);
    _mav_put_float(buf, 12, encoder1);
    _mav_put_float(buf, 16, encoder2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UNIT_ENCODERS, buf, MAVLINK_MSG_ID_UNIT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_CRC);
#else
    mavlink_unit_encoders_t packet;
    packet.ts = ts;
    packet.encoder0 = encoder0;
    packet.encoder1 = encoder1;
    packet.encoder2 = encoder2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UNIT_ENCODERS, (const char *)&packet, MAVLINK_MSG_ID_UNIT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_CRC);
#endif
}

/**
 * @brief Send a unit_encoders message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_unit_encoders_send_struct(mavlink_channel_t chan, const mavlink_unit_encoders_t* unit_encoders)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_unit_encoders_send(chan, unit_encoders->ts, unit_encoders->encoder0, unit_encoders->encoder1, unit_encoders->encoder2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UNIT_ENCODERS, (const char *)unit_encoders, MAVLINK_MSG_ID_UNIT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_CRC);
#endif
}

#if MAVLINK_MSG_ID_UNIT_ENCODERS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_unit_encoders_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t ts, float encoder0, float encoder1, float encoder2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, ts);
    _mav_put_float(buf, 8, encoder0);
    _mav_put_float(buf, 12, encoder1);
    _mav_put_float(buf, 16, encoder2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UNIT_ENCODERS, buf, MAVLINK_MSG_ID_UNIT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_CRC);
#else
    mavlink_unit_encoders_t *packet = (mavlink_unit_encoders_t *)msgbuf;
    packet->ts = ts;
    packet->encoder0 = encoder0;
    packet->encoder1 = encoder1;
    packet->encoder2 = encoder2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UNIT_ENCODERS, (const char *)packet, MAVLINK_MSG_ID_UNIT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_LEN, MAVLINK_MSG_ID_UNIT_ENCODERS_CRC);
#endif
}
#endif

#endif

// MESSAGE UNIT_ENCODERS UNPACKING


/**
 * @brief Get field ts from unit_encoders message
 *
 * @return Hardware timestam
 */
static inline uint64_t mavlink_msg_unit_encoders_get_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field encoder0 from unit_encoders message
 *
 * @return Encoder Channel 0
 */
static inline float mavlink_msg_unit_encoders_get_encoder0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field encoder1 from unit_encoders message
 *
 * @return Encoder Channel 1
 */
static inline float mavlink_msg_unit_encoders_get_encoder1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field encoder2 from unit_encoders message
 *
 * @return Encoder Channel 2
 */
static inline float mavlink_msg_unit_encoders_get_encoder2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a unit_encoders message into a struct
 *
 * @param msg The message to decode
 * @param unit_encoders C-struct to decode the message contents into
 */
static inline void mavlink_msg_unit_encoders_decode(const mavlink_message_t* msg, mavlink_unit_encoders_t* unit_encoders)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    unit_encoders->ts = mavlink_msg_unit_encoders_get_ts(msg);
    unit_encoders->encoder0 = mavlink_msg_unit_encoders_get_encoder0(msg);
    unit_encoders->encoder1 = mavlink_msg_unit_encoders_get_encoder1(msg);
    unit_encoders->encoder2 = mavlink_msg_unit_encoders_get_encoder2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UNIT_ENCODERS_LEN? msg->len : MAVLINK_MSG_ID_UNIT_ENCODERS_LEN;
        memset(unit_encoders, 0, MAVLINK_MSG_ID_UNIT_ENCODERS_LEN);
    memcpy(unit_encoders, _MAV_PAYLOAD(msg), len);
#endif
}
