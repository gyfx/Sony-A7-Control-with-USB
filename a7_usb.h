#ifndef _A7_USB_H
#define _A7_USB_H

#include "net_mgn.h"


/*typedef enum SetRecording{
    BEGIN_RECORDING = 0,                    //开始录像
    END_RECORDING = 1                       //暂停录像
}SetRecording_s;*/

typedef enum SetISO{
    ISO_INC = 0,                            //增大ISO
    ISO_DEC = 1                             //减小ISO
}SetISO_s;

typedef enum SetF{
    F_INC = 0,                              //增大F
    F_DEC = 1                               //减小F
}SetF_s;

typedef enum SetShutterRate{
    SHUTTER_RATE_INC = 0,                   //增大快门速度
    SHUTTER_RATE_DEC = 1                    //减小快门速度
}SetShutterRate_s;

typedef enum SetExposure{
    EXPOSURE_INC = 0,                       //增大曝光校正值
    EXPOSURE_DEC = 1                        //减小曝光校正值
}SetExposure_s;

typedef enum SetGlitterOffset{
    FLITTER_OFFSET_INC = 0,                 //增大闪光补偿
    FLITTER_OFFSET_DEC = 1                  //减小闪光补偿
}SetGlitterOffset_s;

typedef enum SetAE{
    AE_LOCK = 0,                           //AE锁定
    AE_UNLOCK = 1                          //AE解锁
}SetAE_s;

typedef enum SetSnapMode{                  //拍摄模式：
    SNAP_MODE_SINGLE = 0,                  //单张拍摄
    SNAP_MODE_CONTINUE_HI = 1,             //连拍：Hi
    SNAP_MODE_CONTINUE_LO = 2,             //连拍：Lo
    SNAP_MODE_TIMING2 = 3,                 //自拍定时2秒
    SNAP_MODE_TIMING5 = 4,                 //自拍定时5秒
    SNAP_MODE_TIMING10 = 5,                //自拍定时10秒
    SNAP_MODE_TIMING10_3PIECES = 6,        //自拍定时（连拍）10秒3张影像
    SNAP_MODE_TIMING10_5PIECES = 7,        //自拍定时（连拍）10秒5张影像
    SNAP_MODE_TIMING5_3PIECES = 8,         //自拍定时（连拍）5秒3张影像
    SNAP_MODE_TIMING5_5PIECES = 9,         //自拍定时（连拍）5秒5张影像
    SNAP_MODE_TIMING2_3PIECES = 10,        //自拍定时（连拍）2秒3张影像
    SNAP_MODE_TIMING2_5PIECES = 11,        //自拍定时（连拍）2秒5张影像
    SNAP_MODE_CONTINUE_03EV3 = 12,         //连续阶段曝光：0.3EV3张
    SNAP_MODE_CONTINUE_03EV5 = 13,         //连续阶段曝光：0.3EV5张
    SNAP_MODE_CONTINUE_03EV9 = 14,         //连续阶段曝光：0.3EV9张
    SNAP_MODE_CONTINUE_05EV3 = 15,         //连续阶段曝光：0.5EV3张
    SNAP_MODE_CONTINUE_05EV5 = 16,         //连续阶段曝光：0.5EV5张
    SNAP_MODE_CONTINUE_05EV9 = 17,         //连续阶段曝光：0.5EV9张
    SNAP_MODE_CONTINUE_07EV3 = 18,         //连续阶段曝光：0.7EV3张
    SNAP_MODE_CONTINUE_07EV5 = 19,         //连续阶段曝光：0.7EV5张
    SNAP_MODE_CONTINUE_07EV9 = 20,         //连续阶段曝光：0.7EV9张
    SNAP_MODE_CONTINUE_10EV3 = 21,         //连续阶段曝光：1.0EV3张
    SNAP_MODE_CONTINUE_10EV5 = 22,         //连续阶段曝光：1.0EV5张
    SNAP_MODE_CONTINUE_10EV9 = 23,         //连续阶段曝光：1.0EV9张
    SNAP_MODE_CONTINUE_20EV3 = 24,         //连续阶段曝光：2.0EV3张
    SNAP_MODE_CONTINUE_20EV5 = 25,         //连续阶段曝光：2.0EV5张
    SNAP_MODE_CONTINUE_30EV3 = 26,         //连续阶段曝光：3.0EV3张
    SNAP_MODE_CONTINUE_30EV5 = 27,         //连续阶段曝光：3.0EV5张
    SNAP_MODE_SINGLE_03EV3 = 28,           //单拍阶段曝光：0.3EV3张
    SNAP_MODE_SINGLE_03EV5 = 29,           //单拍阶段曝光：0.3EV5张
    SNAP_MODE_SINGLE_03EV9 = 30,           //单拍阶段曝光：0.3EV9张
    SNAP_MODE_SINGLE_05EV3 = 31,           //单拍阶段曝光：0.5EV3张
    SNAP_MODE_SINGLE_05EV5 = 32,           //单拍阶段曝光：0.5EV5张
    SNAP_MODE_SINGLE_05EV9 = 33,           //单拍阶段曝光：0.5EV9张
    SNAP_MODE_SINGLE_07EV3 = 34,           //单拍阶段曝光：0.7EV3张
    SNAP_MODE_SINGLE_07EV5 = 35,           //单拍阶段曝光：0.7EV5张
    SNAP_MODE_SINGLE_07EV9 = 36,           //单拍阶段曝光：0.7EV9张
    SNAP_MODE_SINGLE_10EV3 = 37,           //单拍阶段曝光：1.0EV3张
    SNAP_MODE_SINGLE_10EV5 = 38,           //单拍阶段曝光：1.0EV5张
    SNAP_MODE_SINGLE_10EV9 = 39,           //单拍阶段曝光：1.0EV9张
    SNAP_MODE_SINGLE_20EV3 = 40,           //单拍阶段曝光：2.0EV3张
    SNAP_MODE_SINGLE_20EV5 = 41,           //单拍阶段曝光：2.0EV5张
    SNAP_MODE_SINGLE_30EV3 = 42,           //单拍阶段曝光：3.0EV3张
    SNAP_MODE_SINGLE_30EV5 = 43,           //单拍阶段曝光：3.0EV5张
    SNAP_MODE_WB_LO = 44,                  //白平衡阶段曝光：Lo
    SNAP_MODE_WB_HI = 45,                  //白平衡阶段曝光：Hi
    SNAP_MODE_DRO_LO = 46,                 //DRO阶段曝光：Lo
    SNAP_MODE_DRO_HI = 47                  //DRO阶段曝光：Hi
}SetSnapMode_s;

typedef enum SetWhiteBalance{              //白平衡模式
    BW_AUTO = 0,                           //自动
    BW_SUNLIGHT = 1,                       //日光
    BW_SHADOW = 2,                         //阴影
    BW_CLOUDY = 3,                         //阴天
    BW_INCANDESCENT = 4,                   //白炽灯
    BW_FLUORESCENT_WARM_WHITE = 5,         //荧光灯：暖白色
    BW_FLUORESCENT_COOL_WHITE = 6,         //荧光灯：冷白色
    BW_FLUORESCENT_SUNLIGHT_WHITE = 7,     //荧光灯：日光白色
    BW_FLUORESCENT_SUNLIGHT = 8,           //荧光灯：日光
    BW_FLASH_LIGHT = 9,                    //闪光灯
    BW_WATER_AUTO = 10,                    //水中自动
    BW_COLOR_TEMPERATURE = 11,             //色温/滤光片
    BW_USER_DEFINE1 = 12,                  //自定义1
    BW_USER_DEFINE2 = 13,                  //自定义2
    BW_USER_DEFINE3 = 14                   //自定义3
}SetWhiteBalance_s;

typedef enum SetPhotoEffect{               //照片效果
    PHOTO_EFFECT_OFF = 0,                  //关
    PHOTO_EFFECT_TOY_STD = 1,              //玩具相机：标准
    PHOTO_EFFECT_TOY_COOL_COLOR = 2,       //玩具相机：冷色
    PHOTO_EFFECT_TOY_WARM_COLOR = 3,       //玩具相机：暖色
    PHOTO_EFFECT_TOY_GREEN = 4,            //玩具相机：绿色
    PHOTO_EFFECT_TOY_RED = 5,              //玩具相机：品红色
    PHOTO_EFFECT_POP_COLOR = 6,            //流行色彩
    PHOTO_EFFECT_BALCK_WHITE = 7,          //黑白
    PHOTO_EFFECT_COLORFUL = 8,             //彩色
    PHOTO_EFFECT_RETRO_PHOTO = 9,          //复古照片
    PHOTO_EFFECT_SUBDUED_LIGHT = 10,       //柔光亮调
    PHOTO_EFFECT_PART_COLOR_RED = 11,      //局部彩色：红
    PHOTO_EFFECT_PART_COLOR_GREEN = 12,    //局部彩色：绿
    PHOTO_EFFECT_PART_COLOR_BLUE = 13,     //局部彩色：黄
    PHOTO_EFFECT_PART_COLOR_YELLOW = 14,   //局部彩色：蓝
    PHOTO_EFFECT_STRONG_CONTRAST_SINGLE_COLOR = 15,    //强反差单色
    PHOTO_EFFECT_SOFT_FOCUS_LOW = 16,      //柔焦：低
    PHOTO_EFFECT_SOFT_FOCUS_MIDDLE = 17,   //柔焦：中
    PHOTO_EFFECT_SOFT_FOCUS_HIGH = 18,     //柔焦：高
    PHOTO_EFFECT_HDR_LOW = 19,             //HDR绘画：低
    PHOTO_EFFECT_HDR_MIDDLE = 20,          //HDR绘画：中
    PHOTO_EFFECT_HDR_HIGH = 21,            //HDR绘画：高
    PHOTO_EFFECT_COLORFUL_BLACK_WHITE = 22,//丰富色调黑白
    PHOTO_EFFECT_MICRO_AUTO = 23,          //微缩景观：自动
    PHOTO_EFFECT_MICRO_LEVEL_UP = 24,      //微缩景观：上
    PHOTO_EFFECT_MICRO_LEVEL_MIDDLE = 25,  //微缩景观：中（水平）
    PHOTO_EFFECT_MICRO_LEVEL_DOWN = 26,    //微缩景观：下
    PHOTO_EFFECT_MICRO_VERTICAL_RIGHT = 27,//微缩景观：右
    PHOTO_EFFECT_MICRO_VERTICAL_MIDDLE = 28,           //微缩景观：中（垂直）
    PHOTO_EFFECT_MICRO_VERTICAL_LEFT = 29, //微缩景观：左
    PHOTO_EFFECT_WATERCOLOUR = 30,         //水彩画
    PHOTO_EFFECT_ILLUSTRATION_LOW = 31,    //插图：低
    PHOTO_EFFECT_ILLUSTRATION_MIDDLE = 32, //插图：中
    PHOTO_EFFECT_ILLUSTRATION_HIGH = 33    //插图：高
}SetPhotoEffect_s;

typedef enum SetPhotoQuality{              //影像质量
    PHOTO_QUALITY_STD = 0,                 //标准
    PHOTO_QUALITY_FINE = 1,                //精细
    PHOTO_QUALITY_X_FINE = 2,              //超精细
    PHOTO_QUALITY_RAW = 3,                 //RAW
    PHOTO_QUALITY_RAW_JPEG = 4             //RAW&JPEG
}SetPhotoQuality_s;

typedef enum SetPhotoSize{                 //影像尺寸
    PHOTO_SIZE_L = 0,                      //L
    PHOTO_SIZE_M = 1,                      //M
    PHOTO_SIZE_S = 2                       //S
}SetPhotoSize_s;

typedef enum SetPhotoRatio{                //纵横比
    PHOTO_RATIO_3_2 = 0,                   //3:2
    PHOTO_RATIO_16_9 = 1                   //16:9
}SetPhotoRatio_s;

typedef enum SetFilterAB{                //彩色滤光片A-B
    FILTER_A700 = 0,                     //A7
    FILTER_A650 = 1,                     //A6.5
    FILTER_A600 = 2,                     //A6
    FILTER_A550 = 3,                     //A5.5
    FILTER_A500 = 4,                     //A5
    FILTER_A450 = 5,                     //A4.5
    FILTER_A400 = 6,                     //A4
    FILTER_A350 = 7,                     //A3.5
    FILTER_A300 = 8,                     //A3
    FILTER_A250 = 9,                     //A2.5
    FILTER_A200 = 10,                    //A2
    FILTER_A150 = 11,                    //A1.5
    FILTER_A100 = 12,                    //A1
    FILTER_A050 = 13,                    //A0.5
    FILTER_AB000 = 14,                   //0
    FILTER_B050 = 15,                    //B0.5
    FILTER_B100 = 16,                    //B1
    FILTER_B150 = 17,                    //B1.5
    FILTER_B200 = 18,                    //B2
    FILTER_B250 = 19,                    //B2.5
    FILTER_B300 = 20,                    //B3
    FILTER_B350 = 21,                    //B3.5
    FILTER_B400 = 22,                    //B4
    FILTER_B450 = 23,                    //B4.5
    FILTER_B500 = 24,                    //B5
    FILTER_B550 = 25,                    //B5.5
    FILTER_B600 = 26,                    //B6
    FILTER_B650 = 27,                    //B6.5
    FILTER_B700 = 28                     //B7
}SetFilterAB_s;

typedef enum SetFilterGM{                //彩色滤光片G-M
    FILTER_G700 = 0,                     //G7
    FILTER_G675 = 1,                     //G6.75
    FILTER_G650 = 2,                     //G6.5
    FILTER_G625 = 3,                     //G6.25
    FILTER_G600 = 4,                     //G6
    FILTER_G575 = 5,                     //G5.75
    FILTER_G550 = 6,                     //G5.5
    FILTER_G525 = 7,                     //G5.25
    FILTER_G500 = 8,                     //G5
    FILTER_G475 = 9,                     //G4.75
    FILTER_G450 = 10,                    //G4.5
    FILTER_G425 = 11,                    //G4.25
    FILTER_G400 = 12,                    //G4
    FILTER_G375 = 13,                    //G3.75
    FILTER_G350 = 14,                    //G3.5
    FILTER_G325 = 15,                    //G3.25
    FILTER_G300 = 16,                    //G3
    FILTER_G275 = 17,                    //G2.75
    FILTER_G250 = 18,                    //G2.5
    FILTER_G225 = 19,                    //G2.25
    FILTER_G200 = 20,                    //G2
    FILTER_G175 = 21,                    //G1.75
    FILTER_G150 = 22,                    //G1.5
    FILTER_G125 = 23,                    //G1.25
    FILTER_G100 = 24,                    //G1
    FILTER_G075 = 25,                    //G0.75
    FILTER_G050 = 26,                    //G0.5
    FILTER_G025 = 27,                    //G0.25
    FILTER_GM000 = 28,                   //0
    FILTER_M025 = 29,                    //M0.25
    FILTER_M050 = 30,                    //M0.5
    FILTER_M075 = 31,                    //M0.75
    FILTER_M100 = 32,                    //M1
    FILTER_M125 = 33,                    //M1.25
    FILTER_M150 = 34,                    //M1.5
    FILTER_M175 = 35,                    //M1.75
    FILTER_M200 = 36,                    //M2
    FILTER_M225 = 37,                    //M2.25
    FILTER_M250 = 38,                    //M2.5
    FILTER_M275 = 39,                    //M2.75
    FILTER_M300 = 40,                    //M3
    FILTER_M325 = 41,                    //M3.25
    FILTER_M350 = 42,                    //M3.5
    FILTER_M375 = 43,                    //M3.75
    FILTER_M400 = 44,                    //M4
    FILTER_M425 = 45,                    //M4.25
    FILTER_M450 = 46,                    //M4.5
    FILTER_M475 = 47,                    //M4.75
    FILTER_M500 = 48,                    //M5
    FILTER_M525 = 49,                    //M5.25
    FILTER_M550 = 50,                    //M5.5
    FILTER_M575 = 51,                    //M5.75
    FILTER_M600 = 52,                    //M6
    FILTER_M625 = 53,                    //M6.25
    FILTER_M650 = 54,                    //M6.5
    FILTER_M675 = 55,                    //M6.75
    FILTER_M700 = 56                     //M7
}SetFilterGM_s;

typedef enum SetColorTemperature{          //色温
    WB_COLOR_TEMPERATURE_DEFAULT = 0,      //默认值 5500K
    WB_COLOR_TEMPERATURE_INCREASE = 1,     //增大
    WB_COLOR_TEMPERATURE_DECREASE = 2      //减小
}SetColorTemperature_s;

typedef enum SetDROAutoHDR{                //DRO/自动HDR
    DRO_AUTO_HDR_OFF = 0,                  //关
    DRO_AUTO = 1,                          //DRO自动
    DRO_LV1 = 2,                           //DRO Lv1
    DRO_LV2 = 3,                           //DRO Lv2
    DRO_LV3 = 4,                           //DRO Lv3
    DRO_LV4 = 5,                           //DRO Lv4
    DRO_LV5 = 6,                           //DRO Lv5
    HDR_AUTO = 7,                          //自动HDR
    HDR_EV1 = 8,                           //自动HDR 1.0Ev
    HDR_EV2 = 9,                           //自动HDR 2.0Ev
    HDR_EV3 = 10,                          //自动HDR 3.0Ev
    HDR_EV4 = 11,                          //自动HDR 4.0Ev
    HDR_EV5 = 12,                          //自动HDR 5.0Ev
    HDR_EV6 = 13                           //自动HDR 6.0Ev
}SetDROAutoHDR_s;

typedef enum SetCameraParam
{
//    SET_RECORDING = 1,                       //值对应SetRecording_s，设置相机开始摄像和暂停摄像
    SET_ISO = 2,                             //值对应SetISO_s，设置相机ISO
    SET_F = 3,                               //值对应SetF_s 设置相机光圈值F
    SET_SHUTTER_RATE = 4,                    //值对应SetShutterRate_s 设置相机快门速度
    SET_EXPOSURE = 5,                        //值对应SetExposure_s，设置相机曝光校正值
    SET_GILTTER_OFFSET = 6,                  //值对应SetGlitterOffset_s，设置相机闪光补偿
    SET_AE = 7,                              //值对应SetAE_s，锁定AE，解锁AE
    SET_SNAP_MODE = 8,                       //值对应SetSnapMode_s，设置相机拍摄模式
    SET_WHITE_BALANCE = 9,                   //值对应SetWhiteBalance_s，设置相机白平衡模式
    SET_PHOTO_EFFECT = 10,                   //值对应SetPhotoEffect_s，设置照片效果
    SET_PHOTO_QUALITY = 11,                  //值对应SetPhotoQuality_s，设置影像质量
    SET_PHOTO_SIZE = 12,                     //值对应SetPhotoSize_s，设置影像尺寸
    SET_PHOTO_RATIO = 13,                    //值对应SetPhotoRatio_s，设置纵横比
    SET_FILTER_A_B = 14,                     //值对应SetFilterAB_s，设置彩色滤光片AB值
    SET_FILTER_G_M = 15,                     //值对应SetFilterGM_s，设置彩色滤光片GM值
    SET_WB_COLOR_TEMPERATURE = 16,           //值对应SetColorTemperature_s，设置在色温/滤光片的白平衡模式下的色温
    SET_DRO_AUTO_HDR = 17,                   //值对应SetDROAutoHDR_s，设置DRO/自动HDR
    SET_SNAP_INTERVAL_PARAM = 18,            //值对应ParamSanp_s中的interval_seconds,设置拍照的时间间隔秒数
    SET_SNAP_PICTURE_NUM_PARAM = 19          //值对应ParamSanp_s中的picture_number，设置连拍的张数
}SetCameraParam_s;
typedef struct ParamSanp
{
    int interval_seconds;                    //时间间隔，单位秒，默认为1
    int picture_number;                      //连拍张数，默认为0，点击拍照按钮设置为1
    SetPhotoQuality_s photo_quality;         //值对应SetPhotoQuality_s,表示图片类型，RAW和JPEG
}ParamSanp_s;

typedef struct ParamTypeValue
{
    unsigned char enable;                          //0不可调整（不变化）                1可调整         2不可调整（自动变化）
    int32_t value;                           
}ParamTypeValue_s;

typedef struct SonyA7Status
{
    ParamTypeValue_s tap_position;           //相机档位
    ParamTypeValue_s ISO;                    //ISO
    ParamTypeValue_s F;                      //光圈值
    ParamTypeValue_s shutter_rate;           //快门速度
    ParamTypeValue_s glitter_offset;         //闪光补偿
    ParamTypeValue_s exposure_adjust;        //曝光值校正
    ParamTypeValue_s white_balance;          //白平衡模式
    ParamTypeValue_s color_temperature;      //色温
    ParamTypeValue_s color_filterAB;         //彩色滤光片A-B
    ParamTypeValue_s color_filterGM;         //彩色滤光片G-M
    ParamTypeValue_s photo_effect;           //照片效果
    ParamTypeValue_s DRO_auto_HDR;           //DRO/自动HDR
    ParamTypeValue_s photo_quality;          //影像质量
    ParamTypeValue_s photo_size;             //影像尺寸
    ParamTypeValue_s photo_ratio;            //纵横比
    ParamTypeValue_s snap_mode;              //拍摄模式
    unsigned char interval_time;             //拍摄时间间隔
    unsigned char pictures_num;              //连续拍摄张数
}SonyA7Status_s;

int a7_enable_usb(int enable,uint8_t interval,uint8_t pictures);
int a7_start_snap();
int a7_end_snap();
int a7_set_param_usb(SetCameraParam_s enType, int value);
int a7_get_param_usb();
int a7_start_record();
int a7_end_record();
int a7_enable_udp(struct udp_session_mgn* udp_sess);


#endif
