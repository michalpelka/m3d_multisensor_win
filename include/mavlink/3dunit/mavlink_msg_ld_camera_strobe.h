#pragma once
// MESSAGE LD_CAMERA_STROBE PACKING

#define MAVLINK_MSG_ID_LD_CAMERA_STROBE 202

MAVPACKED(
typedef struct __mavlink_ld_camera_strobe_t {
 uint64_t time_trigg_usec; /*< Timestamp (micros since boot (when no PPS) or Unix epoch)*/
 uint64_t time_strob_usec; /*< Timestamp (micros since boot (when no PPS) or Unix epoch)*/
 uint32_t request_token; /*< Token which will be returned with strobe message. Incremented with new images.*/
}) mavlink_ld_camera_strobe_t;

#define MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN 20
#define MAVLINK_MSG_ID_LD_CAMERA_STROBE_MIN_LEN 20
#define MAVLINK_MSG_ID_202_LEN 20
#define MAVLINK_MSG_ID_202_MIN_LEN 20

#define MAVLINK_MSG_ID_LD_CAMERA_STROBE_CRC 65
#define MAVLINK_MSG_ID_202_CRC 65



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LD_CAMERA_STROBE { \
    202, \
    "LD_CAMERA_STROBE", \
    3, \
    {  { "time_trigg_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ld_camera_strobe_t, time_trigg_usec) }, \
         { "time_strob_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_ld_camera_strobe_t, time_strob_usec) }, \
         { "request_token", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_ld_camera_strobe_t, request_token) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LD_CAMERA_STROBE { \
    "LD_CAMERA_STROBE", \
    3, \
    {  { "time_trigg_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ld_camera_strobe_t, time_trigg_usec) }, \
         { "time_strob_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_ld_camera_strobe_t, time_strob_usec) }, \
         { "request_token", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_ld_camera_strobe_t, request_token) }, \
         } \
}
#endif

/**
 * @brief Pack a ld_camera_strobe message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param request_token Token which will be returned with strobe message. Incremented with new images.
 * @param time_trigg_usec Timestamp (micros since boot (when no PPS) or Unix epoch)
 * @param time_strob_usec Timestamp (micros since boot (when no PPS) or Unix epoch)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ld_camera_strobe_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t request_token, uint64_t time_trigg_usec, uint64_t time_strob_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN];
    _mav_put_uint64_t(buf, 0, time_trigg_usec);
    _mav_put_uint64_t(buf, 8, time_strob_usec);
    _mav_put_uint32_t(buf, 16, request_token);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN);
#else
    mavlink_ld_camera_strobe_t packet;
    packet.time_trigg_usec = time_trigg_usec;
    packet.time_strob_usec = time_strob_usec;
    packet.request_token = request_token;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LD_CAMERA_STROBE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LD_CAMERA_STROBE_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_CRC);
}

/**
 * @brief Pack a ld_camera_strobe message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param request_token Token which will be returned with strobe message. Incremented with new images.
 * @param time_trigg_usec Timestamp (micros since boot (when no PPS) or Unix epoch)
 * @param time_strob_usec Timestamp (micros since boot (when no PPS) or Unix epoch)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ld_camera_strobe_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t request_token,uint64_t time_trigg_usec,uint64_t time_strob_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN];
    _mav_put_uint64_t(buf, 0, time_trigg_usec);
    _mav_put_uint64_t(buf, 8, time_strob_usec);
    _mav_put_uint32_t(buf, 16, request_token);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN);
#else
    mavlink_ld_camera_strobe_t packet;
    packet.time_trigg_usec = time_trigg_usec;
    packet.time_strob_usec = time_strob_usec;
    packet.request_token = request_token;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LD_CAMERA_STROBE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LD_CAMERA_STROBE_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_CRC);
}

/**
 * @brief Encode a ld_camera_strobe struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ld_camera_strobe C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ld_camera_strobe_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ld_camera_strobe_t* ld_camera_strobe)
{
    return mavlink_msg_ld_camera_strobe_pack(system_id, component_id, msg, ld_camera_strobe->request_token, ld_camera_strobe->time_trigg_usec, ld_camera_strobe->time_strob_usec);
}

/**
 * @brief Encode a ld_camera_strobe struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ld_camera_strobe C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ld_camera_strobe_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ld_camera_strobe_t* ld_camera_strobe)
{
    return mavlink_msg_ld_camera_strobe_pack_chan(system_id, component_id, chan, msg, ld_camera_strobe->request_token, ld_camera_strobe->time_trigg_usec, ld_camera_strobe->time_strob_usec);
}

/**
 * @brief Send a ld_camera_strobe message
 * @param chan MAVLink channel to send the message
 *
 * @param request_token Token which will be returned with strobe message. Incremented with new images.
 * @param time_trigg_usec Timestamp (micros since boot (when no PPS) or Unix epoch)
 * @param time_strob_usec Timestamp (micros since boot (when no PPS) or Unix epoch)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ld_camera_strobe_send(mavlink_channel_t chan, uint32_t request_token, uint64_t time_trigg_usec, uint64_t time_strob_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN];
    _mav_put_uint64_t(buf, 0, time_trigg_usec);
    _mav_put_uint64_t(buf, 8, time_strob_usec);
    _mav_put_uint32_t(buf, 16, request_token);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LD_CAMERA_STROBE, buf, MAVLINK_MSG_ID_LD_CAMERA_STROBE_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_CRC);
#else
    mavlink_ld_camera_strobe_t packet;
    packet.time_trigg_usec = time_trigg_usec;
    packet.time_strob_usec = time_strob_usec;
    packet.request_token = request_token;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LD_CAMERA_STROBE, (const char *)&packet, MAVLINK_MSG_ID_LD_CAMERA_STROBE_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_CRC);
#endif
}

/**
 * @brief Send a ld_camera_strobe message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ld_camera_strobe_send_struct(mavlink_channel_t chan, const mavlink_ld_camera_strobe_t* ld_camera_strobe)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ld_camera_strobe_send(chan, ld_camera_strobe->request_token, ld_camera_strobe->time_trigg_usec, ld_camera_strobe->time_strob_usec);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LD_CAMERA_STROBE, (const char *)ld_camera_strobe, MAVLINK_MSG_ID_LD_CAMERA_STROBE_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_CRC);
#endif
}

#if MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ld_camera_strobe_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t request_token, uint64_t time_trigg_usec, uint64_t time_strob_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_trigg_usec);
    _mav_put_uint64_t(buf, 8, time_strob_usec);
    _mav_put_uint32_t(buf, 16, request_token);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LD_CAMERA_STROBE, buf, MAVLINK_MSG_ID_LD_CAMERA_STROBE_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_CRC);
#else
    mavlink_ld_camera_strobe_t *packet = (mavlink_ld_camera_strobe_t *)msgbuf;
    packet->time_trigg_usec = time_trigg_usec;
    packet->time_strob_usec = time_strob_usec;
    packet->request_token = request_token;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LD_CAMERA_STROBE, (const char *)packet, MAVLINK_MSG_ID_LD_CAMERA_STROBE_MIN_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN, MAVLINK_MSG_ID_LD_CAMERA_STROBE_CRC);
#endif
}
#endif

#endif

// MESSAGE LD_CAMERA_STROBE UNPACKING


/**
 * @brief Get field request_token from ld_camera_strobe message
 *
 * @return Token which will be returned with strobe message. Incremented with new images.
 */
static inline uint32_t mavlink_msg_ld_camera_strobe_get_request_token(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field time_trigg_usec from ld_camera_strobe message
 *
 * @return Timestamp (micros since boot (when no PPS) or Unix epoch)
 */
static inline uint64_t mavlink_msg_ld_camera_strobe_get_time_trigg_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_strob_usec from ld_camera_strobe message
 *
 * @return Timestamp (micros since boot (when no PPS) or Unix epoch)
 */
static inline uint64_t mavlink_msg_ld_camera_strobe_get_time_strob_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Decode a ld_camera_strobe message into a struct
 *
 * @param msg The message to decode
 * @param ld_camera_strobe C-struct to decode the message contents into
 */
static inline void mavlink_msg_ld_camera_strobe_decode(const mavlink_message_t* msg, mavlink_ld_camera_strobe_t* ld_camera_strobe)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ld_camera_strobe->time_trigg_usec = mavlink_msg_ld_camera_strobe_get_time_trigg_usec(msg);
    ld_camera_strobe->time_strob_usec = mavlink_msg_ld_camera_strobe_get_time_strob_usec(msg);
    ld_camera_strobe->request_token = mavlink_msg_ld_camera_strobe_get_request_token(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN? msg->len : MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN;
        memset(ld_camera_strobe, 0, MAVLINK_MSG_ID_LD_CAMERA_STROBE_LEN);
    memcpy(ld_camera_strobe, _MAV_PAYLOAD(msg), len);
#endif
}
