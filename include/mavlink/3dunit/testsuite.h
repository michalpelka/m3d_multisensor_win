/** @file
 *    @brief MAVLink comm protocol testsuite generated from 3dunit.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef 3DUNIT_TESTSUITE_H
#define 3DUNIT_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_3dunit(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_3dunit(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_set_rotation_velocity(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_ROTATION_VELOCITY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_rotation_velocity_t packet_in = {
        17235
    };
    mavlink_set_rotation_velocity_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.rpm = packet_in.rpm;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_ROTATION_VELOCITY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_rotation_velocity_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_rotation_velocity_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_rotation_velocity_pack(system_id, component_id, &msg , packet1.rpm );
    mavlink_msg_set_rotation_velocity_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_rotation_velocity_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rpm );
    mavlink_msg_set_rotation_velocity_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_rotation_velocity_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_rotation_velocity_send(MAVLINK_COMM_1 , packet1.rpm );
    mavlink_msg_set_rotation_velocity_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ld_camera_trigger(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LD_CAMERA_TRIGGER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ld_camera_trigger_t packet_in = {
        963497464,17443
    };
    mavlink_ld_camera_trigger_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.request_token = packet_in.request_token;
        packet1.frequency = packet_in.frequency;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LD_CAMERA_TRIGGER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ld_camera_trigger_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ld_camera_trigger_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ld_camera_trigger_pack(system_id, component_id, &msg , packet1.frequency , packet1.request_token );
    mavlink_msg_ld_camera_trigger_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ld_camera_trigger_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.frequency , packet1.request_token );
    mavlink_msg_ld_camera_trigger_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ld_camera_trigger_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ld_camera_trigger_send(MAVLINK_COMM_1 , packet1.frequency , packet1.request_token );
    mavlink_msg_ld_camera_trigger_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ld_camera_strobe(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LD_CAMERA_STROBE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ld_camera_strobe_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,963498296
    };
    mavlink_ld_camera_strobe_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_trigg_usec = packet_in.time_trigg_usec;
        packet1.time_strob_usec = packet_in.time_strob_usec;
        packet1.request_token = packet_in.request_token;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LD_CAMERA_STROBE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LD_CAMERA_STROBE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ld_camera_strobe_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ld_camera_strobe_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ld_camera_strobe_pack(system_id, component_id, &msg , packet1.request_token , packet1.time_trigg_usec , packet1.time_strob_usec );
    mavlink_msg_ld_camera_strobe_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ld_camera_strobe_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.request_token , packet1.time_trigg_usec , packet1.time_strob_usec );
    mavlink_msg_ld_camera_strobe_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ld_camera_strobe_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ld_camera_strobe_send(MAVLINK_COMM_1 , packet1.request_token , packet1.time_trigg_usec , packet1.time_strob_usec );
    mavlink_msg_ld_camera_strobe_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_unit_encoders(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UNIT_ENCODERS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_unit_encoders_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0
    };
    mavlink_unit_encoders_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ts = packet_in.ts;
        packet1.encoder0 = packet_in.encoder0;
        packet1.encoder1 = packet_in.encoder1;
        packet1.encoder2 = packet_in.encoder2;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UNIT_ENCODERS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UNIT_ENCODERS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_unit_encoders_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_unit_encoders_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_unit_encoders_pack(system_id, component_id, &msg , packet1.ts , packet1.encoder0 , packet1.encoder1 , packet1.encoder2 );
    mavlink_msg_unit_encoders_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_unit_encoders_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ts , packet1.encoder0 , packet1.encoder1 , packet1.encoder2 );
    mavlink_msg_unit_encoders_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_unit_encoders_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_unit_encoders_send(MAVLINK_COMM_1 , packet1.ts , packet1.encoder0 , packet1.encoder1 , packet1.encoder2 );
    mavlink_msg_unit_encoders_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_3dunit(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_set_rotation_velocity(system_id, component_id, last_msg);
    mavlink_test_ld_camera_trigger(system_id, component_id, last_msg);
    mavlink_test_ld_camera_strobe(system_id, component_id, last_msg);
    mavlink_test_unit_encoders(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // 3DUNIT_TESTSUITE_H
