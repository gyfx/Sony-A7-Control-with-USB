#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<stdint.h>
#include<unistd.h>
#include <libusb-1.0/libusb.h>

#include "a7_usb.h"
#include "net_mgn.h"
#include "a7_interface2mav.h"

#define SONY_A7_VID          0x054c
#define SONY_A7_PID          0x0a6b
#define SET_PARAM1_WRONG    -11
#define SET_PARAM2_WRONG    -12
#define LIB_USB_INIT_ERROR  -14
#define TRANSFER_TIMEOUT     10*1000


static struct libusb_device_handle *g_devh = NULL;
static uint8_t g_ep_out = 0;
static uint8_t g_ep_in = 0;
static struct SonyA7Status g_current_a7_status = {0};
static struct SonyA7Status g_last_a7_status = {0};
static uint16_t g_transfer_count = 0;
static uint8_t g_stop_a7_usb_thread = 0;
static uint8_t g_usb_enabled = 0;

static struct udp_session_mgn* g_air_udp_sess;
static unsigned short g_air_port;

//#define MAV_DATA_LEN 74


static int get_endpoint_out()
{
    uint8_t endpoint_out = 0;    // default OUT endpoints

    libusb_device *dev = libusb_get_device(g_devh);
    if (!dev)
        return -1;

    struct libusb_config_descriptor *conf_desc;
    if (libusb_get_config_descriptor(dev, 0, &conf_desc) < 0)
        return -1;

    int i, j, k;
    for (i = 0; i < conf_desc->bNumInterfaces; i++) {
        for (j = 0; j < conf_desc->interface[i].num_altsetting; j++) {
            for (k = 0; k < conf_desc->interface[i].altsetting[j].bNumEndpoints; k++) {
                // Use the first bulk IN/OUT endpoints as default
                const struct libusb_endpoint_descriptor *endpoint;
                endpoint = &conf_desc->interface[i].altsetting[j].endpoint[k];
                if (endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK & LIBUSB_TRANSFER_TYPE_BULK) {
                    if (!(endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN)) {
                        endpoint_out = endpoint->bEndpointAddress;
                        goto out;
                    }
                }
            }
        }
    }

out:
    libusb_free_config_descriptor(conf_desc);
    return endpoint_out;
}

static int get_endpoint_in()
{
    uint8_t endpoint_in = 0;    // default OUT endpoints

    libusb_device *dev = libusb_get_device(g_devh);
    if (!dev)
        return -1;

    struct libusb_config_descriptor *conf_desc;
    if (libusb_get_config_descriptor(dev, 0, &conf_desc) < 0)
        return -1;

    int i, j, k;
    for (i = 0; i < conf_desc->bNumInterfaces; i++) {
        for (j = 0; j < conf_desc->interface[i].num_altsetting; j++) {
            for (k = 0; k < conf_desc->interface[i].altsetting[j].bNumEndpoints; k++) {
                // Use the first bulk IN/OUT endpoints as default
                const struct libusb_endpoint_descriptor *endpoint;
                endpoint = &conf_desc->interface[i].altsetting[j].endpoint[k];
                if (endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK & LIBUSB_TRANSFER_TYPE_BULK) {
                    if (endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                        endpoint_in = endpoint->bEndpointAddress;
                        goto out;
                    }
                }
            }
        }
    }

out:
    libusb_free_config_descriptor(conf_desc);
    return endpoint_in;
}


static int a7_send_ack_to_ground(uint8_t* send_buf,uint16_t command)
{
    char buf[MSG_DATA_MAX_LEN];
    struct msg_head_s* pmsg = (struct msg_head_s*)buf;
    msg_head_init(pmsg);
    pmsg->msg_src = AIRCRAFT_NODE;
    pmsg->msg_dst = GROUND_NODE;
    pmsg->msg_type = command;
    pmsg->sequence = 0;
    pmsg->package_id = 0;
    pmsg->flag.reply = 0;
    pmsg->flag.form = 1;

    memcpy(pmsg->data, send_buf, MAV_DATA_LEN);
    pmsg->length = MSG_HEAD_LEN + MAV_DATA_LEN;
    udp_send_message(g_air_udp_sess, buf, pmsg->length, GROUND_IP, UDP_CLIENT_PORT);
    return 0;
}

int a7_enable_udp(struct udp_session_mgn* udp_sess)
{
    g_air_udp_sess = udp_sess;
    return 0;
}

void update_last_status()
{
    g_last_a7_status.tap_position.enable = g_current_a7_status.tap_position.enable;
    g_last_a7_status.tap_position.value = g_current_a7_status.tap_position.value;

    g_last_a7_status.ISO.enable = g_current_a7_status.ISO.enable;
    g_last_a7_status.ISO.value = g_current_a7_status.ISO.value;

    g_last_a7_status.F.enable = g_current_a7_status.F.enable;
    g_last_a7_status.F.value = g_current_a7_status.F.value;

    g_last_a7_status.shutter_rate.enable = g_current_a7_status.shutter_rate.enable;
    g_last_a7_status.shutter_rate.value = g_current_a7_status.shutter_rate.value;

    g_last_a7_status.glitter_offset.enable = g_current_a7_status.glitter_offset.enable;
    g_last_a7_status.glitter_offset.value = g_current_a7_status.glitter_offset.value;

    g_last_a7_status.exposure_adjust.enable = g_current_a7_status.exposure_adjust.enable;
    g_last_a7_status.exposure_adjust.value = g_current_a7_status.exposure_adjust.value;

    g_last_a7_status.white_balance.enable = g_current_a7_status.white_balance.enable;
    g_last_a7_status.white_balance.value = g_current_a7_status.white_balance.value;

    g_last_a7_status.color_temperature.enable = g_current_a7_status.color_temperature.enable;
    g_last_a7_status.color_temperature.value = g_current_a7_status.color_temperature.value;

    g_last_a7_status.color_filterAB.enable = g_current_a7_status.color_filterAB.enable;
    g_last_a7_status.color_filterAB.value = g_current_a7_status.color_filterAB.value;

    g_last_a7_status.color_filterGM.enable = g_current_a7_status.color_filterGM.enable;
    g_last_a7_status.color_filterGM.value = g_current_a7_status.color_filterGM.value;

    g_last_a7_status.photo_effect.enable = g_current_a7_status.photo_effect.enable;
    g_last_a7_status.photo_effect.value = g_current_a7_status.photo_effect.value;

    g_last_a7_status.DRO_auto_HDR.enable = g_current_a7_status.DRO_auto_HDR.enable;
    g_last_a7_status.DRO_auto_HDR.value = g_current_a7_status.DRO_auto_HDR.value;

    g_last_a7_status.photo_quality.enable = g_current_a7_status.photo_quality.enable;
    g_last_a7_status.photo_quality.value = g_current_a7_status.photo_quality.value;

    g_last_a7_status.photo_size.enable = g_current_a7_status.photo_size.enable;
    g_last_a7_status.photo_size.value = g_current_a7_status.photo_size.value;

    g_last_a7_status.photo_ratio.enable = g_current_a7_status.photo_ratio.enable;
    g_last_a7_status.photo_ratio.value = g_current_a7_status.photo_ratio.value;

    g_last_a7_status.snap_mode.enable = g_current_a7_status.snap_mode.enable;
    g_last_a7_status.snap_mode.value = g_current_a7_status.snap_mode.value;

    g_last_a7_status.interval_time = g_current_a7_status.interval_time;
    g_last_a7_status.pictures_num = g_current_a7_status.pictures_num;
}

int compare_param()
{
    if (g_current_a7_status.tap_position.value != g_last_a7_status.tap_position.value) {
        return 0;
    }
    if (g_current_a7_status.ISO.value != g_last_a7_status.ISO.value) {
        return 0;
    }    
    if (g_current_a7_status.F.value != g_last_a7_status.F.value) {
        return 0;
    }    
    if (g_current_a7_status.shutter_rate.value != g_last_a7_status.shutter_rate.value) {
        return 0;
    }    
    if (g_current_a7_status.glitter_offset.value != g_last_a7_status.glitter_offset.value) {
        return 0;
    }
    if (g_current_a7_status.exposure_adjust.value != g_last_a7_status.exposure_adjust.value) {
        return 0;
    }
    if (g_current_a7_status.white_balance.value != g_last_a7_status.white_balance.value) {
        return 0;
    }
    if (g_current_a7_status.color_temperature.value != g_last_a7_status.color_temperature.value) {
        return 0;
    }
    if (g_current_a7_status.color_filterAB.value != g_last_a7_status.color_filterAB.value) {
        return 0;
    }
    if (g_current_a7_status.color_filterGM.value != g_last_a7_status.color_filterGM.value) {
        return 0;
    }
    if (g_current_a7_status.photo_effect.value != g_last_a7_status.photo_effect.value) {
        return 0;
    }
    if (g_current_a7_status.DRO_auto_HDR.value != g_last_a7_status.DRO_auto_HDR.value) {
        return 0;
    }
    if (g_current_a7_status.photo_quality.value != g_last_a7_status.photo_quality.value) {
        return 0;
    }
    if (g_current_a7_status.photo_size.value != g_last_a7_status.photo_size.value) {
        return 0;
    }
    if (g_current_a7_status.photo_ratio.value != g_last_a7_status.photo_ratio.value) {
        return 0;
    }
    if (g_current_a7_status.snap_mode.value != g_last_a7_status.snap_mode.value) {
        return 0;
    }
    if (g_current_a7_status.interval_time != g_last_a7_status.interval_time) {
        return 0;
    }
    if (g_current_a7_status.pictures_num != g_last_a7_status.pictures_num) {
        return 0;
    }
    return 1;
}

int a7_usb_count_thread()
{
    pthread_detach(pthread_self());
    unsigned char bulk_out_data[24] = {0};
    unsigned char bulk_in_data1[1536] = {0};
    unsigned char bulk_in_data2[20] = {0};
    int length = -1, length1 = -1;
    int i, ret;
    uint16_t tmp_count;
    uint16_t tmp_flag;
    int16_t tmp_exposure_adjust, tmp_glitter_offset;
    uint8_t ack_buf[128] = {0};

    memset(bulk_out_data, 0, sizeof(bulk_out_data));
    memset(bulk_in_data1, 0, sizeof(bulk_in_data1));
    memset(bulk_in_data2, 0, sizeof(bulk_in_data2));

    while (1) {
        if (g_stop_a7_usb_thread) {
            break;
        }
        memcpy(bulk_out_data, "\x0c\x00\x00\x00\x01\x00\x09\x92\x0b\x00\x00\x00", 12);
        tmp_count = g_transfer_count;
        bulk_out_data[8] = 0x00 | tmp_count;
        tmp_count = tmp_count >> 8;
        bulk_out_data[9] = 0x00 | tmp_count;
        ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 12, &length, TRANSFER_TIMEOUT);
        if (ret < 0) {
            printf("out error!\n");
            a7_deenable_usb();
            goto ThreadFail;
        }

        ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data1, 1536, &length1, TRANSFER_TIMEOUT);
        if (ret < 0) {
            printf("in error!\n");
            a7_deenable_usb();
            goto ThreadFail;
        }

        ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data2, 20, &length, TRANSFER_TIMEOUT);
        if (ret < 0) {
            printf("in error!\n");
            a7_deenable_usb();
            goto ThreadFail;
        }

        for (i = 20; i < length1 - 13; i++) {
            tmp_flag = 0x0000 | bulk_in_data1[i];
            tmp_flag = tmp_flag << 8;
            tmp_flag |= bulk_in_data1[i + 1];
            switch (tmp_flag) {
                case 0x0e50:
                    g_current_a7_status.tap_position.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.tap_position.value = 0x00000000;
                    g_current_a7_status.tap_position.value |= bulk_in_data1[i + 8];
                    break;
                case 0x1ed2:
                    g_current_a7_status.ISO.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.ISO.value = 0x00000000;
                    g_current_a7_status.ISO.value |= bulk_in_data1[i + 12];
                    g_current_a7_status.ISO.value = g_current_a7_status.ISO.value << 8;
                    g_current_a7_status.ISO.value |= bulk_in_data1[i + 11];
                    g_current_a7_status.ISO.value = g_current_a7_status.ISO.value << 8;
                    g_current_a7_status.ISO.value |= bulk_in_data1[i + 10];
                    break;
                case 0x0750:
                    g_current_a7_status.F.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.F.value = 0x00000000;
                    g_current_a7_status.F.value |= bulk_in_data1[i + 9];
                    g_current_a7_status.F.value = g_current_a7_status.F.value << 8;
                    g_current_a7_status.F.value |= bulk_in_data1[i + 8];
                    break;
                case 0x0dd2:
                    g_current_a7_status.shutter_rate.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.shutter_rate.value = 0x00000000;
                    if (bulk_in_data1[i + 10] == 0x0a && bulk_in_data1[i + 11] == 0x00) {
                        if(bulk_in_data1[i + 12] == 0x01 && bulk_in_data1[i + 13] == 0x00) {
                            g_current_a7_status.shutter_rate.enable = bulk_in_data1[i + 5];
                            g_current_a7_status.shutter_rate.value = 0x00000000;
                            g_current_a7_status.shutter_rate.value |= bulk_in_data1[i + 11];
                            g_current_a7_status.shutter_rate.value = g_current_a7_status.shutter_rate.value << 8;
                            g_current_a7_status.shutter_rate.value |= bulk_in_data1[i + 10];
                        } else {                     
                            g_current_a7_status.shutter_rate.enable = bulk_in_data1[i + 5];
                            g_current_a7_status.shutter_rate.value = 0x00000000;
                            g_current_a7_status.shutter_rate.value |= bulk_in_data1[i + 13];
                            g_current_a7_status.shutter_rate.value = g_current_a7_status.shutter_rate.value << 8;
                            g_current_a7_status.shutter_rate.value |= bulk_in_data1[i + 12];
                            g_current_a7_status.shutter_rate.value = g_current_a7_status.shutter_rate.value*-1;
                        }
                    } else {
                        g_current_a7_status.shutter_rate.enable = bulk_in_data1[i + 5];
                        g_current_a7_status.shutter_rate.value = 0x00000000;
                        g_current_a7_status.shutter_rate.value |= bulk_in_data1[i + 11];
                        g_current_a7_status.shutter_rate.value = g_current_a7_status.shutter_rate.value << 8;
                        g_current_a7_status.shutter_rate.value |= bulk_in_data1[i + 10];
                    }
                    break;
                case 0x00d2:
                    g_current_a7_status.glitter_offset.enable = bulk_in_data1[i + 5];
                    tmp_glitter_offset = 0x0000;
                    tmp_glitter_offset |= bulk_in_data1[i + 9];
                    tmp_glitter_offset = tmp_glitter_offset << 8;
                    tmp_glitter_offset |= bulk_in_data1[i + 8];
                    g_current_a7_status.glitter_offset.value = tmp_glitter_offset;
                    break;
                case 0x1050:
                    g_current_a7_status.exposure_adjust.enable = bulk_in_data1[i + 5];
                    tmp_exposure_adjust = 0x0000;
                    tmp_exposure_adjust |= bulk_in_data1[i + 9];
                    tmp_exposure_adjust = tmp_exposure_adjust << 8;
                    tmp_exposure_adjust |= bulk_in_data1[i + 8];
                    g_current_a7_status.exposure_adjust.value = tmp_exposure_adjust;
                    break;
                case 0x0550:
                    g_current_a7_status.white_balance.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.white_balance.value = 0x00000000;
                    g_current_a7_status.white_balance.value |= bulk_in_data1[i + 8];
                    g_current_a7_status.white_balance.value = g_current_a7_status.white_balance.value << 8;
                    g_current_a7_status.white_balance.value |= bulk_in_data1[i + 9];
                    break;
                case 0x0fd2:
                    g_current_a7_status.color_temperature.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.color_temperature.value = 0x00000000;
                    g_current_a7_status.color_temperature.value |= bulk_in_data1[i + 9];
                    g_current_a7_status.color_temperature.value = g_current_a7_status.color_temperature.value << 8;
                    g_current_a7_status.color_temperature.value |= bulk_in_data1[i + 8];
                    break;
                case 0x1cd2:
                    g_current_a7_status.color_filterAB.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.color_filterAB.value = 0x00000000;
                    g_current_a7_status.color_filterAB.value |= bulk_in_data1[i + 7];
                    break;
                case 0x10d2:
                    g_current_a7_status.color_filterGM.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.color_filterGM.value = 0x00000000;
                    g_current_a7_status.color_filterGM.value |= bulk_in_data1[i + 7];
                    break;
                case 0x1bd2:
                    g_current_a7_status.photo_effect.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.photo_effect.value = 0x00000000;
                    g_current_a7_status.photo_effect.value |= bulk_in_data1[i + 8];
                    g_current_a7_status.photo_effect.value = g_current_a7_status.photo_effect.value << 8;
                    g_current_a7_status.photo_effect.value |= bulk_in_data1[i + 9];
                    break;
                case 0x01d2:
                    g_current_a7_status.DRO_auto_HDR.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.DRO_auto_HDR.value = 0x00000000;
                    g_current_a7_status.DRO_auto_HDR.value |= bulk_in_data1[i + 7];
                    break;
                case 0x0450:
                    g_current_a7_status.photo_quality.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.photo_quality.value = 0x00000000;
                    g_current_a7_status.photo_quality.value |= bulk_in_data1[i + 7];
                    break;
                case 0x03d2:
                    g_current_a7_status.photo_size.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.photo_size.value = 0x00000000;
                    g_current_a7_status.photo_size.value |= bulk_in_data1[i + 7];
                    break;
                case 0x11d2:
                    g_current_a7_status.photo_ratio.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.photo_ratio.value = 0x00000000;
                    g_current_a7_status.photo_ratio.value |= bulk_in_data1[i + 7];
                    break;
                case 0x1350:
                    g_current_a7_status.snap_mode.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.snap_mode.value = 0x00000000;
                    g_current_a7_status.snap_mode.value |= bulk_in_data1[i + 8];
                    g_current_a7_status.snap_mode.value = g_current_a7_status.snap_mode.value << 8;
                    g_current_a7_status.snap_mode.value |= bulk_in_data1[i + 9];
                default:
                    break;
            }
        }
        if (!compare_param()) {
            if (g_air_udp_sess != NULL && g_air_port != -1) {
                a7_pac_mav_param_msg(&g_current_a7_status,A7_PARAM_EVENT ,ack_buf);
                a7_send_ack_to_ground(ack_buf,AU_CAMERA_EVENT);
                printf("udp send change to ground!!\n");
            }
        }
        update_last_status();
        g_transfer_count++;
        if(g_transfer_count >= 0xfffe) {
            g_transfer_count = 0x000f;
        }
       sleep(1);
    }
    return 0;
ThreadFail:
    a7_deenable_usb();
    a7_pac_mav_status_msg(A7_ENABLE_USB_ACK,ret,ack_buf);
    a7_send_ack_to_ground(ack_buf,AU_CAMERA_EVENT);
    return ret;
}

int a7_enable_usb(int enable,uint8_t interval,uint8_t pictures)
{
    if(0 == enable) {
        a7_deenable_usb();
        return 0;
    }
    if(g_usb_enabled) {
        printf("allready enabled usb\n");
        return 0;
    }
    int length = -1;
    int i, ret;
    unsigned char bulk_out_data[32] = {0};
    unsigned char bulk_in_data[512] = {0};
    unsigned char bulk_inter_data[32] = {0};
    memset(bulk_out_data, 0, sizeof(bulk_out_data));
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    uint8_t ack_buf[128] = {0};    //UDP buf

    printf("start enable usb.....\n");

    ret = libusb_init(NULL); //initialize the library for the session we just declared  
    if (ret < 0) {
        printf("Init Error\n"); //there was an error  
        goto InitFail;
    }

    /* open device with vid and pid, must after libusb_init */
    g_devh = libusb_open_device_with_vid_pid(NULL, SONY_A7_VID, SONY_A7_PID);
    if (!g_devh) {
        printf("libusb_open_device_with_pid_vid error\n");
        libusb_exit(NULL);
        ret = LIB_USB_INIT_ERROR;
        goto InitFail;
    }

    libusb_set_auto_detach_kernel_driver(g_devh, 1);
    /* end */

    ret = libusb_kernel_driver_active(g_devh, 0);
    if (ret == 1) {
        ret = libusb_detach_kernel_driver(g_devh, 0);
        if (ret < 0) {
            printf("libusb_detach_kernel_driver error\n");
            goto InitFail;
        }
    }

    /* claim interface */
    ret = libusb_claim_interface(g_devh, 0);
    if (ret < 0) {
        printf("libusb_claim_interface error\n");
        g_devh = NULL;
        libusb_close(g_devh);
        libusb_exit(NULL);
        goto InitFail;
    }
    /* end */

    g_ep_out = get_endpoint_out();
    g_ep_in = get_endpoint_in();

    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x02\x10\x00\x00\x00\x00\x01\x00\x00\x00", 16);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }

    memcpy(bulk_out_data, "\x0c\x00\x00\x00\x01\x00\x01\x10\x01\x00\x00\x00", 12);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 12, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    memcpy(bulk_out_data, "\x0c\x00\x00\x00\x01\x00\x04\x10\x02\x00\x00\x00", 12);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 12, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    memcpy(bulk_out_data, "\x18\x00\x00\x00\x01\x00\x01\x92\x03\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00", 24);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 24, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }

    memcpy(bulk_out_data, "\x18\x00\x00\x00\x01\x00\x01\x92\x04\x00\x00\x00\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00", 24);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 24, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }

    //the camera show graph

    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x02\x92\x05\x00\x00\x00\xc8\x00\x00\x00", 16);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }

    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x02\x92\x06\x00\x00\x00\xc8\x00\x00\x00", 16);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));


    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }

    memcpy(bulk_out_data, "\x18\x00\x00\x00\x01\x00\x01\x92\x07\x00\x00\x00\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00", 24);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 24, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    memcpy(bulk_out_data, "\x0c\x00\x00\x00\x01\x00\x09\x92\x08\x00\x00\x00", 12);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 12, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_interrupt_transfer(g_devh, 0x83, bulk_inter_data, 32, &length, 400);
    if (ret < 0) {
        printf("interrupt error!\n");
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x09\x00\x00\x00\xc1\xd2\x00\x00", 16);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x09\x00\x00\x00\x01\x00", 14);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    memcpy(bulk_out_data, "\x0c\x00\x00\x00\x01\x00\x09\x92\x0a\x00\x00\x00", 12);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 12, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    memcpy(bulk_out_data, "\x0c\x00\x00\x00\x01\x00\x09\x92\x0b\x00\x00\x00", 12);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 12, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));

    memcpy(bulk_out_data, "\x0c\x00\x00\x00\x01\x00\x09\x92\x0c\x00\x00\x00", 12);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 12, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        goto InitFail;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    memset(bulk_in_data, 0, sizeof(bulk_in_data));
    length = -1;

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }

    memset(bulk_in_data, 0, sizeof(bulk_in_data));
    length = -1;

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }

    memset(bulk_in_data, 0, sizeof(bulk_in_data));
    length = -1;

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 512, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        goto InitFail;
    }
    g_usb_enabled = 1;
    g_transfer_count = 0x000d;
    pthread_t get_a7_count_tid;
    g_stop_a7_usb_thread = 0;
    g_current_a7_status.interval_time = interval;
    g_current_a7_status.pictures_num = pictures;
    g_last_a7_status.interval_time = interval;
    g_last_a7_status.pictures_num = pictures;
    ret = pthread_create(&get_a7_count_tid, NULL, a7_usb_count_thread, NULL);
    if(ret < 0){
        printf("create thread failed\n");
        goto InitFail;
    }  
    printf("enabled usb OK....\n");
    
    a7_pac_mav_status_msg(A7_ENABLE_USB_ACK,ret,ack_buf);
    a7_send_ack_to_ground(ack_buf,AU_CAMERA_ENABLE_USB_A7_ACK);
    return ret;   
InitFail:
    a7_deenable_usb();
    a7_pac_mav_status_msg(A7_ENABLE_USB_ACK,ret,ack_buf);
    a7_send_ack_to_ground(ack_buf,AU_CAMERA_ENABLE_USB_A7_ACK);
    return ret;
}

int a7_deenable_usb()
{
    g_stop_a7_usb_thread = 1;
    if(g_devh) {
        /* release claim interface */
        libusb_release_interface(g_devh, 0);
        /* close device */
        libusb_close(g_devh);
        /*exit libusb */
        libusb_exit(NULL);
    }
    g_devh = NULL;
    g_usb_enabled = 0;
    return 0;
}

int a7_set_param_usb(enum SetCameraParam enType, int value)
{
    unsigned char ack_buf[128];
    if (SET_SNAP_INTERVAL_PARAM == enType) {
        printf("set interval!\n");
        g_current_a7_status.interval_time = value;
        a7_pac_mav_status_msg(A7_SET_PARAM_ACK,0,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_SET_PARAM_USB_A7_ACK);
        return 0;
    }
    if (SET_SNAP_PICTURE_NUM_PARAM == enType) {
        printf("set picture num!\n");
        g_current_a7_status.pictures_num = value;
        a7_pac_mav_status_msg(A7_SET_PARAM_ACK,0,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_SET_PARAM_USB_A7_ACK);
        return 0;
    }
    if(!g_usb_enabled) {
        printf("not enabled usb\n");
        a7_pac_mav_status_msg(A7_SET_PARAM_ACK,-13,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_SET_PARAM_USB_A7_ACK);
        return -13;
    }
    uint16_t tmp_count = 0x0000;
    uint16_t current_color_temperature_K = 0x157C;
    uint16_t temp_color_temperature_K = 0x157C;
    unsigned char bulk_out_data[24] = {0};
    unsigned char bulk_in_data[32] = {0};
    int length = -1;
    int ret = 0;

    memset(bulk_out_data, 0, sizeof(bulk_out_data));
    memset(bulk_in_data, 0, sizeof(bulk_in_data));


    switch (enType) {
        /*case SET_RECORDING: {
            SetRecording_s record_commamd = (SetRecording_s)(value);
            switch (record_commamd) {
                case BEGIN_RECORDING:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0a\x00\x00\x00\xc8\xd2\x00\x00", 16);
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0a\x00\x00\x00\x02\x00", 14);
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    printf("ret in=%d\tlength=%d\n", ret, length);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case END_RECORDING:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\xc8\xd2\x00\x00", 16);
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x01\x00", 14);
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    printf("ret in=%d\tlength=%d\n", ret, length);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }*/
        case SET_ISO: {
            SetISO_s ISO_commamd = (SetISO_s)(value);
            switch (ISO_commamd) {
                case ISO_INC:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\x1e\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x01", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case ISO_DEC:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\x1e\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\xff", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_F: {
            SetF_s F_commamd = (SetF_s)(value);
            switch (F_commamd) {
                case F_INC:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\x07\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x01", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case F_DEC:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\x07\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\xff", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_SHUTTER_RATE: {
            SetShutterRate_s shutter_rate_commamd = (SetShutterRate_s)(value);
            switch (shutter_rate_commamd) {
                case SHUTTER_RATE_INC:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\x0d\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x01", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SHUTTER_RATE_DEC:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\x0d\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\xff", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
             break;
        }
        case SET_EXPOSURE: {
            SetExposure_s exposure_commamd = (SetExposure_s)(value);
            switch (exposure_commamd) {
                case EXPOSURE_INC:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\x10\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x01", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }

                    break;
                case EXPOSURE_DEC:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\x10\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                         printf("out error\n");
                         goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\xff", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                         printf("out error\n");
                         goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                         printf("in error\n");
                         goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_GILTTER_OFFSET: {
            SetGlitterOffset_s glitter_offset_commamd = (SetGlitterOffset_s)(value);
            switch (glitter_offset_commamd) {
                case FLITTER_OFFSET_INC:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\x00\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x01", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FLITTER_OFFSET_DEC:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\x00\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\xff", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_AE: {
            SetAE_s AE_commamd = (SetAE_s)(value);
            switch (AE_commamd) {
                case AE_LOCK:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\xc3\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x02\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case AE_UNLOCK:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\xc3\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x01\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_SNAP_MODE: {
            SetSnapMode_s snap_commamd = (SetSnapMode_s)(value);
            switch (snap_commamd) {
                case SNAP_MODE_SINGLE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x01\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_HI:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x02\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_LO:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x12\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_TIMING2:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x05\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_TIMING5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x03\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_TIMING10:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x04\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_TIMING10_3PIECES:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x08\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_TIMING10_5PIECES:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x09\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_TIMING5_3PIECES:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x0c\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_TIMING5_5PIECES:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x0d\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_TIMING2_3PIECES:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x0e\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_TIMING2_5PIECES:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x0f\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_03EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x37\x83", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_03EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x37\x85", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_03EV9:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x37\x89", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_05EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x57\x83", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_05EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x57\x85", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_05EV9:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x57\x89", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_07EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x77\x83", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_07EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x77\x85", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_07EV9:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x77\x89", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_10EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x11\x83", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_10EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x11\x85", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_10EV9:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x11\x89", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_20EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x21\x83", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_20EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x21\x85", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_30EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x31\x83", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_CONTINUE_30EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x31\x85", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_03EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x36\x83", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_03EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x36\x85", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_03EV9:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x36\x89", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_05EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x56\x83", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_05EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x56\x85", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_05EV9:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x56\x89", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_07EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x76\x83", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_07EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x76\x85", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_07EV9:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x76\x89", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_10EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x10\x83", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_10EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x10\x85", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_10EV9:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x10\x89", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_20EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x20\x83", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_20EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x20\x85", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_30EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x30\x83", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_SINGLE_30EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x30\x85", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_WB_LO:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x18\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_WB_HI:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x28\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_DRO_LO:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x19\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case SNAP_MODE_DRO_HI:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x13\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x29\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_WHITE_BALANCE: {
            SetWhiteBalance_s WB_commamd = (SetWhiteBalance_s)(value);
            switch (WB_commamd) {
                case BW_AUTO:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x02\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_SUNLIGHT:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x04\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_SHADOW:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x11\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_CLOUDY:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x10\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_INCANDESCENT:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x06\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_FLUORESCENT_WARM_WHITE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x01\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_FLUORESCENT_COOL_WHITE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x02\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_FLUORESCENT_SUNLIGHT_WHITE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x03\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_FLUORESCENT_SUNLIGHT:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x04\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_FLASH_LIGHT:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x07\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_WATER_AUTO:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x30\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_COLOR_TEMPERATURE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x12\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_USER_DEFINE1:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x20\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_USER_DEFINE2:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x21\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case BW_USER_DEFINE3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x05\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x22\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_PHOTO_EFFECT: {
            SetPhotoEffect_s photo_effect_commamd = (SetPhotoEffect_s)(value);
            switch (photo_effect_commamd) {
                case PHOTO_EFFECT_OFF:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x00\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_TOY_STD:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x01\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_TOY_COOL_COLOR:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x02\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_TOY_WARM_COLOR:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x03\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_TOY_GREEN:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x04\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_TOY_RED:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x05\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_POP_COLOR:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x10\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_BALCK_WHITE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x20\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_COLORFUL:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x21\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }

                    break;
                case PHOTO_EFFECT_RETRO_PHOTO:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x30\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_SUBDUED_LIGHT:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x40\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_PART_COLOR_RED:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x50\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_PART_COLOR_GREEN:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x51\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_PART_COLOR_BLUE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x52\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_PART_COLOR_YELLOW:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x53\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_STRONG_CONTRAST_SINGLE_COLOR:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x60\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_SOFT_FOCUS_LOW:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x70\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_SOFT_FOCUS_MIDDLE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x71\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_SOFT_FOCUS_HIGH:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x72\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_HDR_LOW:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x80\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_HDR_MIDDLE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x81\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_HDR_HIGH:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x82\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_COLORFUL_BLACK_WHITE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x90\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_MICRO_AUTO:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa0\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_MICRO_LEVEL_UP:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa1\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_MICRO_LEVEL_MIDDLE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa2\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_MICRO_LEVEL_DOWN:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa3\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_MICRO_VERTICAL_RIGHT:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa4\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_MICRO_VERTICAL_MIDDLE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa5\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_MICRO_VERTICAL_LEFT:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa6\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_WATERCOLOUR:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb0\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_ILLUSTRATION_LOW:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc0\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_ILLUSTRATION_MIDDLE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc1\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_EFFECT_ILLUSTRATION_HIGH:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1b\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc2\x80", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_PHOTO_QUALITY: {
            SetPhotoQuality_s photo_quality_commamd = (SetPhotoQuality_s)(value);
            switch (photo_quality_commamd) {
                case PHOTO_QUALITY_STD:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x04\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x02", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_QUALITY_FINE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x04\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x03", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_QUALITY_X_FINE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x04\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x04", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_QUALITY_RAW:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x04\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x10", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_QUALITY_RAW_JPEG:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x04\x50\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x13", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_PHOTO_SIZE: {
            SetPhotoSize_s photo_size_commamd = (SetPhotoSize_s)(value);
            switch (photo_size_commamd) {
                case PHOTO_SIZE_L:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x03\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x01", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_SIZE_M:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x03\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x02", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_SIZE_S:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x03\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x03", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }


                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_PHOTO_RATIO: {
            SetPhotoRatio_s photo_ratio_commamd = (SetPhotoRatio_s)(value);
            switch (photo_ratio_commamd) {
                case PHOTO_RATIO_3_2:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x11\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x01", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case PHOTO_RATIO_16_9:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x11\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x02", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_FILTER_A_B: {
            SetFilterAB_s filter_AB_commamd = (SetFilterAB_s)(value);
            switch (filter_AB_commamd) {
                case FILTER_A700:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xdc", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A650:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xda", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A600:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd8", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A550:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd6", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A500:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd4", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A450:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd2", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A400:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd0", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A350:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xce", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A300:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xcc", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A250:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xca", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A200:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc8", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A150:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc6", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }


                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A100:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc4", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_A050:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc2", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_AB000:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc0", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B050:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xbe", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B100:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xbc", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B150:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xba", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B200:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb8", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B250:
                memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb6", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B300:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb4", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B350:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb2", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B400:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb0", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B450:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xae", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B500:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xac", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B550:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xaa", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B600:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa8", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B650:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa6", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_B700:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x1c\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0d\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa4", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_FILTER_G_M: {
            SetFilterGM_s filter_GM_commamd = (SetFilterGM_s)(value);
            switch (filter_GM_commamd) {
                case FILTER_G700:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xdc\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G675:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xdb\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G650:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xda\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G625:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd9\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G600:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd8\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G575:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd7\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G550:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd6\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G525:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd5\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G500:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd4\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G475:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd3\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G450:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd2\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G425:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd1\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G400:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xd0\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G375:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xcf\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G350:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xce\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G325:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xcd\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G300:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xcc\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G275:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xcb\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G250:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xca\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G225:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc9\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G200:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc8\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G175:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc7\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G150:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc6\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G125:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc5\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G100:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc4\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G075:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc3\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G050:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc2\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_G025:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc1\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_GM000:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xc0\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M025:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xbf\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M050:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xbe\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M075:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xbd\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M100:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xbc\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M125:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xbb\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M150:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xba\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M175:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb9\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M200:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb8\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M225:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb7\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }

                    break;
                case FILTER_M250:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb6\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M275:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb5\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M300:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb4\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M325:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb3\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M350:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb2\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M375:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb1\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M400:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xb0\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M425:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xaf\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M450:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xae\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M475:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xad\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M500:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xac\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M525:
                memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xab\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M550:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xaa\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M575:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa9\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M600:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa8\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M625:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa7\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M650:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa6\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M675:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa5\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case FILTER_M700:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x10\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\xa4\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_WB_COLOR_TEMPERATURE: {
            SetColorTemperature_s color_temperature_commamd = (SetColorTemperature_s)(value);
            switch (color_temperature_commamd) {
                case WB_COLOR_TEMPERATURE_DEFAULT:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x0f\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    current_color_temperature_K = 0x157C;
                    memcpy(bulk_out_data, "\x0c\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x7c\x15", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case WB_COLOR_TEMPERATURE_INCREASE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x0f\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0c\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x01\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;

                    current_color_temperature_K = (uint16_t)g_current_a7_status.color_temperature.value + 0x64;
                    if (current_color_temperature_K > 0x26AC)
                    {
                    current_color_temperature_K = 0x26AC;
                    }
                    temp_color_temperature_K = current_color_temperature_K;
                    bulk_out_data[12] = 0x00 | temp_color_temperature_K;
                    temp_color_temperature_K = temp_color_temperature_K >> 8;
                    bulk_out_data[13] = 0x00 | temp_color_temperature_K;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case WB_COLOR_TEMPERATURE_DECREASE:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x0f\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0c\x00\x00\x00\x02\x00\x05\x92\x0b\x00\x00\x00\x01\x00", 14);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;

                    current_color_temperature_K = (uint16_t)g_current_a7_status.color_temperature.value - 0x64;
                    if (current_color_temperature_K < 0x09c4)
                    {
                    current_color_temperature_K = 0x09c4;
                    }
                    temp_color_temperature_K = current_color_temperature_K;
                    bulk_out_data[12] = 0x00 | temp_color_temperature_K;
                    temp_color_temperature_K = temp_color_temperature_K >> 8;
                    bulk_out_data[13] = 0x00 | temp_color_temperature_K;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_DRO_AUTO_HDR: {
            SetDROAutoHDR_s DRO_auto_HDR_commamd = (SetDROAutoHDR_s)(value);
            switch (DRO_auto_HDR_commamd) {
                case DRO_AUTO_HDR_OFF:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x01", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                         printf("in error\n");
                         goto SetFail;
                    }
                    break;
                case DRO_AUTO:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x1f", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case DRO_LV1:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x11", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case DRO_LV2:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x12", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case DRO_LV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x13", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case DRO_LV4:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x14", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case DRO_LV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x15", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case HDR_AUTO:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x20", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case HDR_EV1:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x21", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case HDR_EV2:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x22", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case HDR_EV3:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x23", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case HDR_EV4:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x24", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case HDR_EV5:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x25", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                case HDR_EV6:
                    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x05\x92\x0b\x00\x00\x00\x01\xd2\x00\x00", 16);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x26", 13);
                    tmp_count = g_transfer_count;
                    bulk_out_data[8] = 0x00 | tmp_count;
                    tmp_count = tmp_count >> 8;
                    bulk_out_data[9] = 0x00 | tmp_count;
                    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 13, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("out error\n");
                        goto SetFail;
                    }

                    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
                    if (ret < 0) {
                        printf("in error\n");
                        goto SetFail;
                    }
                    break;
                default:
                    printf("wrong commamd 2 !!!\n");
                    ret = SET_PARAM2_WRONG;
                    break;
            }
            break;
        }
        case SET_SNAP_INTERVAL_PARAM: {
            g_current_a7_status.interval_time = value;
            break;
        }
        case SET_SNAP_PICTURE_NUM_PARAM: {
            g_current_a7_status.pictures_num = value;
            break;
        }
        default: {
            printf("wrong commamd 1 !!!\n");
            ret = SET_PARAM1_WRONG;
            break;
        }
    }
    g_transfer_count++;
    if(g_transfer_count >= 0xfffe) {
        g_transfer_count = 0x000f;
    }

    a7_pac_mav_status_msg(A7_SET_PARAM_ACK,ret,ack_buf);
    a7_send_ack_to_ground(ack_buf,AU_CAMERA_SET_PARAM_USB_A7_ACK);
    return ret;

SetFail:
    a7_deenable_usb();
    a7_pac_mav_status_msg(A7_SET_PARAM_ACK,ret,ack_buf);
    a7_send_ack_to_ground(ack_buf,AU_CAMERA_SET_PARAM_USB_A7_ACK);
    return ret;
        
}

int a7_get_param_usb()
{
    unsigned char ack_buf[128];
    if(!g_usb_enabled) {
        printf("not enabled usb\n");
        a7_pac_mav_status_msg(A7_GET_PARAM_ACK,-13,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_GET_PARAM_USB_A7_ACK);
        return -13;
    }
    unsigned char bulk_out_data[32] = {0};
    unsigned char bulk_in_data1[1536] = {0};
    unsigned char bulk_in_data2[20] = {0};
    int length = -1, length1 = -1;
    int i, ret;
    uint16_t tmp_count;
    uint16_t tmp_flag;
    int16_t tmp_exposure_adjust, tmp_glitter_offset;
    

    memset(bulk_out_data, 0, sizeof(bulk_out_data));
    memset(bulk_in_data1, 0, sizeof(bulk_in_data1));
    memset(bulk_in_data2, 0, sizeof(bulk_in_data2));

    memcpy(bulk_out_data, "\x0c\x00\x00\x00\x01\x00\x09\x92\x0b\x00\x00\x00", 12);
    tmp_count = g_transfer_count;
    bulk_out_data[8] = 0x00 | tmp_count;
    tmp_count = tmp_count >> 8;
    bulk_out_data[9] = 0x00 | tmp_count;
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 12, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error!\n");
        a7_deenable_usb();
        a7_pac_mav_status_msg(A7_GET_PARAM_ACK,ret,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_GET_PARAM_USB_A7_ACK);
        return ret;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data1, 1536, &length1, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        a7_deenable_usb();
        a7_pac_mav_status_msg(A7_GET_PARAM_ACK,ret,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_GET_PARAM_USB_A7_ACK);
        return ret;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data2, 20, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("in error!\n");
        a7_deenable_usb();
        a7_pac_mav_status_msg(A7_GET_PARAM_ACK,ret,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_GET_PARAM_USB_A7_ACK);
        return ret;
    }

    for (i = 20; i < length1 - 13; i++) {
        tmp_flag = 0x0000 | bulk_in_data1[i];
        tmp_flag = tmp_flag << 8;
        tmp_flag |= bulk_in_data1[i + 1];
        switch (tmp_flag) {
            case 0x0e50:
                g_current_a7_status.tap_position.enable = bulk_in_data1[i + 5];
                g_current_a7_status.tap_position.value = 0x00000000;
                g_current_a7_status.tap_position.value |= bulk_in_data1[i + 8];
                break;
            case 0x1ed2:
                g_current_a7_status.ISO.enable = bulk_in_data1[i + 5];
                g_current_a7_status.ISO.value = 0x00000000;
                g_current_a7_status.ISO.value |= bulk_in_data1[i + 12];
                g_current_a7_status.ISO.value = g_current_a7_status.ISO.value << 8;
                g_current_a7_status.ISO.value |= bulk_in_data1[i + 11];
                g_current_a7_status.ISO.value = g_current_a7_status.ISO.value << 8;
                g_current_a7_status.ISO.value |= bulk_in_data1[i + 10];
                break;
            case 0x0750:
                g_current_a7_status.F.enable = bulk_in_data1[i + 5];
                g_current_a7_status.F.value = 0x00000000;
                g_current_a7_status.F.value |= bulk_in_data1[i + 9];
                g_current_a7_status.F.value = g_current_a7_status.F.value << 8;
                g_current_a7_status.F.value |= bulk_in_data1[i + 8];
                break;
            case 0x0dd2:
                g_current_a7_status.shutter_rate.enable = bulk_in_data1[i + 5];
                g_current_a7_status.shutter_rate.value = 0x00000000;
                if (bulk_in_data1[i + 10] == 0x0a && bulk_in_data1[i + 11] == 0x00) {
                    if(bulk_in_data1[i + 12] == 0x01 && bulk_in_data1[i + 13] == 0x00) {
                        g_current_a7_status.shutter_rate.enable = bulk_in_data1[i + 5];
                        g_current_a7_status.shutter_rate.value = 0x00000000;
                        g_current_a7_status.shutter_rate.value |= bulk_in_data1[i + 11];
                        g_current_a7_status.shutter_rate.value = g_current_a7_status.shutter_rate.value << 8;
                        g_current_a7_status.shutter_rate.value |= bulk_in_data1[i + 10];
                    }
                    else {
                        g_current_a7_status.shutter_rate.enable = bulk_in_data1[i + 5];
                        g_current_a7_status.shutter_rate.value = 0x00000000;
                        g_current_a7_status.shutter_rate.value |= bulk_in_data1[i + 13];
                        g_current_a7_status.shutter_rate.value = g_current_a7_status.shutter_rate.value << 8;
                        g_current_a7_status.shutter_rate.value |= bulk_in_data1[i + 12];
                        g_current_a7_status.shutter_rate.value = g_current_a7_status.shutter_rate.value*-1;
                    }
                } else {
                    g_current_a7_status.shutter_rate.enable = bulk_in_data1[i + 5];
                    g_current_a7_status.shutter_rate.value = 0x00000000;
                    g_current_a7_status.shutter_rate.value |= bulk_in_data1[i + 11];
                    g_current_a7_status.shutter_rate.value = g_current_a7_status.shutter_rate.value << 8;
                    g_current_a7_status.shutter_rate.value |= bulk_in_data1[i + 10];
                }
                break;
            case 0x00d2:
                g_current_a7_status.glitter_offset.enable = bulk_in_data1[i + 5];
                tmp_glitter_offset = 0x0000;
                tmp_glitter_offset |= bulk_in_data1[i + 9];
                tmp_glitter_offset = tmp_glitter_offset << 8;
                tmp_glitter_offset |= bulk_in_data1[i + 8];
                g_current_a7_status.glitter_offset.value = tmp_glitter_offset;
                break;
            case 0x1050:
                g_current_a7_status.exposure_adjust.enable = bulk_in_data1[i + 5];
                tmp_exposure_adjust = 0x0000;
                tmp_exposure_adjust |= bulk_in_data1[i + 9];
                tmp_exposure_adjust = tmp_exposure_adjust << 8;
                tmp_exposure_adjust |= bulk_in_data1[i + 8];
                g_current_a7_status.exposure_adjust.value = tmp_exposure_adjust;
                break;
            case 0x0550:
                g_current_a7_status.white_balance.enable = bulk_in_data1[i + 5];
                g_current_a7_status.white_balance.value = 0x00000000;
                g_current_a7_status.white_balance.value |= bulk_in_data1[i + 8];
                g_current_a7_status.white_balance.value = g_current_a7_status.white_balance.value << 8;
                g_current_a7_status.white_balance.value |= bulk_in_data1[i + 9];
                break;
            case 0x0fd2:
                g_current_a7_status.color_temperature.enable = bulk_in_data1[i + 5];
                g_current_a7_status.color_temperature.value = 0x00000000;
                g_current_a7_status.color_temperature.value |= bulk_in_data1[i + 9];
                g_current_a7_status.color_temperature.value = g_current_a7_status.color_temperature.value << 8;
                g_current_a7_status.color_temperature.value |= bulk_in_data1[i + 8];
                break;
            case 0x1cd2:
                g_current_a7_status.color_filterAB.enable = bulk_in_data1[i + 5];
                g_current_a7_status.color_filterAB.value = 0x00000000;
                g_current_a7_status.color_filterAB.value |= bulk_in_data1[i + 7];
                break;
            case 0x10d2:
                g_current_a7_status.color_filterGM.enable = bulk_in_data1[i + 5];
                g_current_a7_status.color_filterGM.value = 0x00000000;
                g_current_a7_status.color_filterGM.value |= bulk_in_data1[i + 7];
                break;
            case 0x1bd2:
                g_current_a7_status.photo_effect.enable = bulk_in_data1[i + 5];
                g_current_a7_status.photo_effect.value = 0x00000000;
                g_current_a7_status.photo_effect.value |= bulk_in_data1[i + 8];
                g_current_a7_status.photo_effect.value = g_current_a7_status.photo_effect.value << 8;
                g_current_a7_status.photo_effect.value |= bulk_in_data1[i + 9];
                break;
            case 0x01d2:
                g_current_a7_status.DRO_auto_HDR.enable = bulk_in_data1[i + 5];
                g_current_a7_status.DRO_auto_HDR.value = 0x00000000;
                g_current_a7_status.DRO_auto_HDR.value |= bulk_in_data1[i + 7];
                break;
            case 0x0450:
                g_current_a7_status.photo_quality.enable = bulk_in_data1[i + 5];
                g_current_a7_status.photo_quality.value = 0x00000000;
                g_current_a7_status.photo_quality.value |= bulk_in_data1[i + 7];
                break;
            case 0x03d2:
                g_current_a7_status.photo_size.enable = bulk_in_data1[i + 5];
                g_current_a7_status.photo_size.value = 0x00000000;
                g_current_a7_status.photo_size.value |= bulk_in_data1[i + 7];
                break;
            case 0x11d2:
                g_current_a7_status.photo_ratio.enable = bulk_in_data1[i + 5];
                g_current_a7_status.photo_ratio.value = 0x00000000;
                g_current_a7_status.photo_ratio.value |= bulk_in_data1[i + 7];
                break;
            case 0x1350:
                g_current_a7_status.snap_mode.enable = bulk_in_data1[i + 5];
                g_current_a7_status.snap_mode.value = 0x00000000;
                g_current_a7_status.snap_mode.value |= bulk_in_data1[i + 8];
                g_current_a7_status.snap_mode.value = g_current_a7_status.snap_mode.value << 8;
                g_current_a7_status.snap_mode.value |= bulk_in_data1[i + 9];
            default:
                break;
        }
    }
    g_current_a7_status.interval_time = g_current_a7_status.interval_time;
    g_current_a7_status.pictures_num = g_current_a7_status.pictures_num;
    g_transfer_count++;
    if(g_transfer_count >= 0xfffe) {
        g_transfer_count = 0x000f;
    }
    if (g_air_udp_sess != NULL && g_air_port != -1) {
        printf("udp send to app ack request!!\n");
        a7_pac_mav_param_msg(&g_current_a7_status,A7_GET_PARAM_ACK ,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_GET_PARAM_USB_A7_ACK);
    }
    update_last_status();
    return 0;
}

int a7_start_record()
{
    unsigned char ack_buf[128] = {0};
    if(!g_usb_enabled) {
        printf("not enabled usb\n");
        a7_pac_mav_status_msg(A7_START_RECORD_ACK,-13,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_START_RECORDING_ACK);
        return -13;
    }
    unsigned char bulk_out_data[24] = {0};
    unsigned char bulk_in_data[32] = {0};
    int length = -1;
    int ret = 0;
    
    
    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0a\x00\x00\x00\xc8\xd2\x00\x00", 16);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error\n");
        a7_pac_mav_status_msg(A7_START_RECORD_ACK,ret,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_START_RECORDING_ACK);
        return ret;
    }

    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0a\x00\x00\x00\x02\x00", 14);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error\n");
        a7_pac_mav_status_msg(A7_START_RECORD_ACK,ret,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_START_RECORDING_ACK);
        return ret;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
    printf("ret in=%d\tlength=%d\n", ret, length);
    if (ret < 0) {
        printf("in error\n");
        a7_pac_mav_status_msg(A7_START_RECORD_ACK,ret,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_START_RECORDING_ACK);
        return ret;
    }
    g_transfer_count++;
    if(g_transfer_count >= 0xfffe) {
        g_transfer_count = 0x000f;
    }
    a7_pac_mav_status_msg(A7_START_RECORD_ACK,ret,ack_buf);
    a7_send_ack_to_ground(ack_buf,AU_CAMERA_START_RECORDING_ACK);
    return 0;
}

int a7_end_record()
{
    unsigned char ack_buf[128] = {0};
    if(!g_usb_enabled) {
        printf("not enabled usb\n");
        a7_pac_mav_status_msg(A7_END_RECORD_ACK,-13,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_STOP_RECORDING_ACK);
        return 0;
    }

    unsigned char bulk_out_data[24] = {0};
    unsigned char bulk_in_data[32] = {0};
    int length = -1;
    int ret = 0;
    
    memcpy(bulk_out_data, "\x10\x00\x00\x00\x01\x00\x07\x92\x0b\x00\x00\x00\xc8\xd2\x00\x00", 16);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 16, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error\n");
        a7_pac_mav_status_msg(A7_END_RECORD_ACK,ret,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_STOP_RECORDING_ACK);
        return ret;
    }

    memcpy(bulk_out_data, "\x0e\x00\x00\x00\x02\x00\x07\x92\x0b\x00\x00\x00\x01\x00", 14);
    ret = libusb_bulk_transfer(g_devh, g_ep_out, bulk_out_data, 14, &length, TRANSFER_TIMEOUT);
    if (ret < 0) {
        printf("out error\n");
        a7_pac_mav_status_msg(A7_END_RECORD_ACK,ret,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_STOP_RECORDING_ACK);
        return ret;
    }

    ret = libusb_bulk_transfer(g_devh, g_ep_in, bulk_in_data, 32, &length, TRANSFER_TIMEOUT);
    printf("ret in=%d\tlength=%d\n", ret, length);
    if (ret < 0) {
        printf("in error\n");
        a7_pac_mav_status_msg(A7_END_RECORD_ACK,ret,ack_buf);
        a7_send_ack_to_ground(ack_buf,AU_CAMERA_STOP_RECORDING_ACK);
        return ret;
    }
    g_transfer_count++;
    if(g_transfer_count >= 0xfffe) {
        g_transfer_count = 0x000f;
    }
    a7_pac_mav_status_msg(A7_END_RECORD_ACK,ret,ack_buf);
    a7_send_ack_to_ground(ack_buf,AU_CAMERA_STOP_RECORDING_ACK);
    return 0;
}


