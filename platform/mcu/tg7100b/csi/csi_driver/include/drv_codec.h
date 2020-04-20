/*
 * Copyright (C) 2017 C-SKY Microsystems Co., Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/******************************************************************************
 * @file     drv_codec.h
 * @brief    head file for codec
 * @version  V1.0
 * @date     12. April 2019
 ******************************************************************************/
#ifndef _DRV_CODEC_H_
#define _DRV_CODEC_H_


#include <drv_common.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef void *codec_handle_t;
typedef struct codec_adc codec_adc_t;
typedef struct codec_dac codec_dac_t;

typedef enum {
    CODEC_SET_MUTE_FAIL         = (DRV_ERROR_SPECIFIC + 1),  ///< set mute mode failed
    CODEC_MALLOC_SPACE_FAIL,                                ///< malloc failed
} codec_error_e;

typedef enum {
    CODEC_ADC_MODE_MIC_IN = 0,
    CODEC_ADC_MODE_LINE_IN = 1,
} codec_adc_mode_e;

typedef enum {
    CODEC_DAC_MODE_SPEADKER_OUT = 0,
    CODEC__MODE_LINE_OUT = 1,
} codec_dac_mode_e;

typedef struct codec_buffer {
    uint8_t *buffer;
    uint32_t size;
    uint32_t write;
    uint32_t read;
} codec_buffer_t;

typedef enum {
    CODEC_SAMPLE_RATE_8000              = 8000,
    CODEC_SAMPLE_RATE_11025             = 11025,
    CODEC_SAMPLE_RATE_12000             = 12000,
    CODEC_SAMPLE_RATE_16000             = 16000,
    CODEC_SAMPLE_RATE_22050             = 22050,
    CODEC_SAMPLE_RATE_24000             = 24000,
    CODEC_SAMPLE_RATE_32000             = 32000,
    CODEC_SAMPLE_RATE_44100             = 44100,
    CODEC_SAMPLE_RATE_48000             = 48000,
    CODEC_SAMPLE_RATE_96000             = 96000,
    CODEC_SAMPLE_RATE_192000            = 192000,
} codec_sample_rate_e;

/****** CODEC Event *****/
typedef enum {
    CODEC_EVENT_PERIOD_READ_COMPLETE        = 0,  ///< a peroid data read complete
    CODEC_EVENT_PERIOD_WRITE_COMPLETE       = 1,  ///< a peroid data write complete
    CODEC_EVENT_WRITE_BUFFER_EMPTY          = 2,  ///< fifo is empty
    CODEC_EVENT_READ_BUFFER_FULL            = 3,  ///< fifo is full
    CODEC_EVENT_TRANSFER_ERROR              = 4,  ///< transfer error
} codec_event_e;

typedef void (*codec_adc_event_cb_t)(codec_adc_t *dac, codec_event_e event);
typedef void (*codec_dac_event_cb_t)(codec_dac_t *dac, codec_event_e event);

struct codec_adc {
    int codec_idx;
    int id;
    codec_adc_mode_e mode;
    codec_sample_rate_e sample_rate;
    uint32_t bit_width;
    uint8_t left_channel_en;
    uint8_t right_channel_en;
    uint32_t left_channel_gain;         ///< set gain control
    uint32_t right_channel_gain;
    uint32_t boost;                     ///< bass boost settings. default 0dB ,2dB ...
    codec_buffer_t *fifo;
    uint32_t buffer_size;               ///< alloc ringbuffer size
    uint32_t period;                    ///< when a peroid data is reached,the callback function is called
    codec_adc_event_cb_t adc_event_cb;  ///< callback Structure pointer to contain all callback functions
	void     *priv;
};
struct codec_dac {
    int codec_idx;
    int id;
    codec_dac_mode_e mode;
    codec_sample_rate_e sample_rate;
    uint32_t bit_width;
    uint8_t left_channel_en;
    uint8_t right_channel_en;
    uint32_t left_channel_gain;         ///< set gain control
    uint32_t right_channel_gain;
    codec_buffer_t *fifo;
    uint32_t buffer_size;
    uint32_t period;
    codec_dac_event_cb_t dac_event_cb;
	void     *priv;
};
/*
 * \brief CODEC capability information.
 * Defines a structure to get the capability information of CODEC.
 */
typedef struct {
    uint32_t beep_generator;       ///< support sine wave gerneration
    uint32_t adaptive_filter;      ///< Adaptive filter mode
} codec_capabilities_t;

/**
  \brief  Initializes the CODEC according to the specified
  \param[in]   idx codec index
  \return codec handle if success
*/
codec_handle_t drv_codec_initialize(uint32_t idx);

/**
  \brief       De-initialize codec Interface. stops operation and releases the software resources used by the interface
  \param[in]   handle  CODEC handle to operate.
*/
void drv_codec_uninitialize(codec_handle_t handle);

/**
  \brief       Get driver capabilities.
  \param[in]   handle  CODEC handle to operate.
  \param[out]  capability CODEC capability information \ref codec_capabilities_t
 */
codec_capabilities_t drv_codec_get_capabilities(int32_t idx);

/**
  \brief       control codec power.
  \param[in]   handle  CODEC handle to operate.
  \param[in]   state   power state.\ref csi_power_stat_e.
  \return      error code
*/
int32_t drv_codec_power_control(codec_handle_t handle, csi_power_stat_e state);

/**
  \brief  Initializes the CODEC adc according to the specified
          1. create ringbuffer depend on buffer size.
          2. set callback function gain fs and so on
  \param[in]  adc  structure pointer to contain adc channel
  \return     error code
*/
int32_t drv_codec_adc_init(codec_adc_t *adc);

/**
  \brief       De-initialize codec adc channel.
  \param[in]  adc  structure pointer to contain adc channel
*/
void drv_codec_adc_uninit(codec_adc_t *adc);

/**
  \brief  Initializes the CODEC dac according to the specified
          1. create ringbuffer depend on buffer size.
          2. set callback function gain fs and so on
  \param[in]  dac  structure pointer to contain dac channel
  \return     error code
*/
int32_t drv_codec_dac_init(codec_dac_t *dac);

/**
  \brief       De-initialize codec dac channel.
  \param[in]  dac  structure pointer to contain dac channel
  */
void drv_codec_dac_uninit(codec_dac_t *dac);

/**
  \brief      Get adc channel left && right gain.
  \param[in]  adc  structure pointer to contain adc channel
  \param[in]  left_gain  Pointer to left channel gain
  \param[in]  right_gain Pointer to right channel gain
  \return     error code
  */
int32_t drv_codec_adc_get_gain(codec_adc_t *adc, int *left_gain, int *right_gain);

/**
  \brief      Set dac channel left && right gain.
  \param[in]  dac  structure pointer to contain dac channel
  \param[in]  left_gain  left channel gain
  \param[in]  right_gain right channel gain
  \return     error code
  */
int32_t drv_codec_dac_set_gain(codec_dac_t *dac, int left_gain, int right_gain);

/**
  \brief      Set adc channel left && right gain.
  \param[in]  adc  structure pointer to contain adc channel
  \param[in]  left_gain  left channel gain
  \param[in]  right_gain right channel gain
  \return     error code
  */
int32_t drv_codec_adc_set_gain(codec_adc_t *adc, int left_gain, int right_gain);

/**
  \brief      Set dac channel left && right digital gain.
  \param[in]  dac  structure pointer to contain dac channel
  \param[in]  left_gain  left channel gain
  \param[in]  right_gain right channel gain
  \return     error code
  */
int32_t drv_codec_dac_set_digital_gain(codec_dac_t *dac, int left_gain, int right_gain);

/**
  \brief      Set adc channel left && right digital gain.
  \param[in]  adc  structure pointer to contain adc channel
  \param[in]  left_gain  left channel gain
  \param[in]  right_gain right channel gain
  \return     error code
  */
int32_t drv_codec_adc_set_digital_gain(codec_adc_t *adc, int left_gain, int right_gain);


/**
  \brief      start adc channel transfer.
  \param[in]  adc  structure pointer to contain adc channel
  \return     error code
  */
int32_t drv_codec_adc_start(codec_adc_t *adc);

/**
  \brief      stop adc channel transfer.
  \param[in]  adc  structure pointer to contain adc channel
  */
void drv_codec_adc_stop(codec_adc_t *adc);

/**
  \brief      pause adc channel transfer.
  \param[in]  adc  structure pointer to contain adc channel
  */
void drv_codec_adc_pause(codec_adc_t *adc);

/**
  \brief      resume adc channel transfer.
  \param[in]  adc  structure pointer to contain adc channel
  */
void drv_codec_adc_resume(codec_adc_t *adc);

/**
  \brief      mute adc channel transfer.
  \param[in]  adc  structure pointer to contain adc channel
  */
void drv_codec_adc_mute(codec_adc_t *adc);

/**
  \brief      unmtue adc channel transfer.
  \param[in]  adc  structure pointer to contain adc channel
  */
void drv_codec_adc_unmtue(codec_adc_t *adc);

/**
  \brief      start dac channel transfer.
  \param[in]  dac  structure pointer to contain dac channel
  \return     error code
  */
int32_t drv_codec_dac_start(codec_dac_t *dac);

/**
  \brief      stop dac channel transfer.
  \param[in]  dac  structure pointer to contain dac channel
  */
void drv_codec_dac_stop(codec_dac_t *dac);

/**
  \brief      pause dac channel transfer.
  \param[in]  dac  structure pointer to contain dac channel
  */
void drv_codec_dac_pause(codec_dac_t *dac);

/**
  \brief      resume dac channel transfer.
  \param[in]  dac  structure pointer to contain dac channel
  */
void drv_codec_dac_resume(codec_dac_t *dac);

/**
  \brief      mute dac channel transfer.
  \param[in]  dac  structure pointer to contain dac channel
  */
void drv_codec_dac_mute(codec_dac_t *dac);

/**
  \brief      unmute dac channel transfer.
  \param[in]  dac  structure pointer to contain dac channel
  */
void drv_codec_dac_unmute(codec_dac_t *dac);

/**
  \brief      write some data into the FIFO. This function copies at most @size bytes from the @buf into the FIFO depending on the free space, and returns the number of bytes copied.
  \param[in]  fifo pointed to codec buffer struction
  \param[in]  buf  Pointer to buffer with data to write to ringbuffer
  \param[in]  size Size of data items to write
  \return     the number of bytes copied
  */
uint32_t drv_codec_buffer_write(codec_buffer_t *fifo, void *buf, uint32_t size);

/**
  \brief      Gets some data from the FIFO.
  \param[in]  fifo pointed to codec buffer struction
  \param[in]  buf  Pointer to buffer with data to read from ringbuffer
  \param[in]  size Size of data items to read
  \return     the number of bytes copied
  */
uint32_t drv_codec_buffer_read(codec_buffer_t *fifo, void *buf, uint32_t size);

/**
  \brief      Returns the number of bytes available in the FIFO.
  \param[in]  fifo pointed to codec buffer struction
  \return     the number of bytes available.
  */
uint32_t drv_codec_buffer_avail(codec_buffer_t *fifo);

/**
  \brief      Returns the number of used bytes in the FIFO.
  \param[in]  fifo pointed to codec buffer struction
  \return     The number of used bytes.
  */
uint32_t drv_codec_buffer_size(codec_buffer_t *fifo);

/**
  \brief      Removes the entire FIFO contents.
  \param[in]  fifo pointed to codec buffer struction
  */
void drv_codec_buffer_reset(codec_buffer_t *fifo);

#ifdef __cplusplus
}
#endif

#endif /* _DRV_CODEC_H_  */

