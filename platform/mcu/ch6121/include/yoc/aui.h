#ifndef __YOC_AUI_H__
#define __YOC_AUI_H__

#include <aos/list.h>
#include <yoc/uservice.h>
#include "cJSON.h"

/*************************************************
 * AUI系统API
 * 主要分为三个部分，麦克风输入、语音解析(云端)、NLP Json解析及处理
 *************************************************/
typedef struct aui_config {
    char *per;  // 发音人选择，预留配置，具体视AI云端情况而定
    int   vol;  // 音量，取值0-15，默认为5中音量，预留配置，具体视AI云端情况而定
    int   pit;  // 音调，取值0-9，默认为5中语调，预留配置，具体视AI云端情况而定
    int   spd;  // 语速，取值0-9，默认为5中语速，预留配置，具体视AI云端情况而定
    void (*nlp_cb)(const char *json_text); // NLP处理回调
} aui_config_t;

typedef struct _aui {
    aui_config_t *config;
    void        *context;
} aui_t;

/**
 * AUI系统初始化
 * 
 * @param aui aui对象指针
 * @return 0:成功
 */
int aui_init(aui_t *aui);

/**
 * 向AUI系统输入语音数据
 * 
 * @param aui aui对象指针
 * @param data 数据指针
 * @param size 数据大小
 * @return 0:成功
 */
int aui_cloud_push_pcm(aui_t *aui, void *data, size_t size);

/**
 * 结束语音数据输入，AUI系统开始处理
 * 
 * @param aui aui对象指针
 * @return 0:成功
 */
int aui_cloud_push_pcm_finish(aui_t *aui);

/**
 * 向AUI系统输入文本数据，AUI进行处理并回调NLP处理回调函数
 * 
 * @param aui aui对象指针
 * @param test 文本字符串
 * @param engine_name 预留多引擎的支持
 * @return 0:成功
 */
int aui_cloud_push_text(aui_t *aui, char *text, const char *engine_name);

/**
 * 向AUI系统输入文本数据，要求返回文本的TTS转换后的语音结果
 * 
 * @param aui aui对象指针
 * @param name 播放器的播放接口,若为NULL,默认请使用"chunkfifo://tts/1"
 * @param text 需要转换的文本
 * @return 0:成功
 */
int aui_cloud_tts(aui_t *aui, const char *name, const char *text);


/**
 * 向AI引擎输入文本数据，要求返回文本的TTS转换后的语音结果
 * 
 * @param max_text_len 匹配字符串的长度
 * @param delim 匹配文本的分隔符
 */
void aui_textcmd_matchinit(int max_text_len, const char* delim);

/**
 *  控制命令设备端预解析
 *  AUI系统的本地加速处理，主要对短文本进行字符匹配，处理完成后会调用NLP处理回调
 *  @param cmd 如果匹配，返回NLP的控制命令字符串
 *  @param text 需要匹配的文本
 * 
 *  NLP返回格式：{"textcmd":"volume_minus"}
 *  例如："volume_plus", "volume_minus", "volume_mute", "media_repaly", "media_close", "media_pause"
 */
void aui_textcmd_matchadd(const char *cmd, const char *text);

/**
 *  预处理命令与AUI NLP的绑定
 *  把语音通过ASR转化为文本后，再调用该函数，把预处理的控制命令统一通过AUI的NLP处理回调进行处理
 *  @param aui aui对象指针
 *  @param text 命令文本
 *  @return 0:匹配到关键字 <0:没有匹配
 * 
 *  NLP返回格式：{"textcmd":"volume_minus"}
 *  例如："volume_plus", "volume_minus", "volume_mute", "media_repaly", "media_close", "media_pause"
 */
int aui_textcmd_matchnlp(aui_t *aui, const char *text);

/*************************************************
 * 命令执行器
 * 
 *    AUI云端处理后返回NLP结果，调用命令执行器进行具体的操作
 *    如：音乐播放、或继续调用AUI进行其他功能解析
 *
 *************************************************/

/**
 * NLP处理回调
 * 
 * @param js 传入的Json对象
 * @param json_text 传入的Json文本
 * @return 0:解析到请求并做了处理,处理循环退出,不继续检查其他处理
 *        -1:没有解析到请求,处理循环继续匹配其他回调
 *        -2:解析到请求但请求的格式或内容有误,处理循环退出,不继续检查其他处理
 */
typedef int (*aui_nlp_proc_cb_t)(cJSON *js, const char *json_text);

typedef struct aui_cmd {
    aui_nlp_proc_cb_t cmd;  // 命令执行处理
    slist_t next;
} aui_cmd_t;

typedef struct aui_cmd_set {
    slist_t cmds;
    aui_nlp_proc_cb_t pre_check_cmd;
} aui_cmd_set_t;

/**
 * 添加NLP处理回调
 * 
 * @param set 命令执行器的集合对象
 * @param cmd NLP处理回调函数
 */
void aui_nlp_process_add(aui_cmd_set_t *set, aui_nlp_proc_cb_t cmd);

/**
 * 删除NLP处理回调
 * 
 * @param set 命令执行器的集合对象
 * @param cmd NLP处理回调函数
 */
void aui_nlp_process_remove(aui_cmd_set_t *set, aui_nlp_proc_cb_t cmd);

/**
 * 设置命令执行器的预处理回调
 * 预处理回调函数不是必须的，一般对NLP返回的格式进行与检查是否合法
 * 预处理回调函数 cmd 的实现中，如果返回0，说明格式正确，会进入后续
 * 具体的处理回调中
 * 
 * @param set 命令执行器的集合对象
 * @param cmd 预处理回调函数
 */
void aui_nlp_process_set_pre_check(aui_cmd_set_t *set, aui_nlp_proc_cb_t cmd);


#define AUI_CMD_PROC_NOMATCH -1        /* 无法匹配到可处理的技能 */
#define AUI_CMD_PROC_MATCH_NOACTION -2 /* 匹配到技能执行器，但意图不支持或解析有错误 */
#define AUI_CMD_PROC_NET_ABNORMAL -3   /* 网络异常 */
#define AUI_CMD_PROC_ERROR -10         /* 其他错误 */

/**
 * 命令执行器的执行
 * 按照添加顺序逆序执行aui_cmd_set_t 中每个注册的命令处理回调函数
 * 
 * @param set 命令执行器的集合对象
 * @param json_text 需要进行分析处理的NLP处理文本
 * @return 0:执行成功
 *         AUI_CMD_PROC_NOMATCH
 *         AUI_CMD_PROC_MATCH_NOACTION
 *         AUI_CMD_PROC_ERROR
 */
int aui_nlp_process_run(aui_cmd_set_t *set, const char *json_text);

/************************************************* 
 * 音频服务
 * type:
 *  0: 音乐播放
 *  1: 提示音播放
 *      每一个 type 只有一个播放实例，新的播放会替换旧的播放实例，
 *      提示音的播放会先暂停音乐播放，播放完成之后继续音乐播放
 *************************************************/

typedef enum {
    MEDIA_MUSIC  = 0, /*音乐播放*/
    MEDIA_SYSTEM = 1, /*通知播放*/
    MEDIA_ALL    = 255,
} aui_player_type_t;

typedef enum {
    AUI_PLAYER_STATE_UNKNOWN,
    AUI_PLAYER_STATE_STOP,
    AUI_PLAYER_STATE_PLAYING,
    AUI_PLAYER_STATE_PAUSED
} aui_player_state_t;

typedef enum {
    AUI_PLAYER_EVENT_ERROR,
    AUI_PLAYER_EVENT_START,
    AUI_PLAYER_EVENT_FINISH
} aui_player_evtid_t;

/**
 * 播放器事件用户处理函数
 *
 * @param evt_id
 */
typedef void (*media_evt_t)(int type, aui_player_evtid_t evt_id);

/**
 * 获取指定播放器的状态
 *
 * @param task 指定播放器服务的utask
 *        如果==NULL，播放器自己创建utask
 * @return 0:成功
 */
int aui_player_init(utask_t *task, media_evt_t evt_cb);

/**
 * 播放音乐，强制停止已经在播的音乐
 *
 * @param type 支持MEDIA_MUSIC,MEDIA_SYSTEM
 * @param url 媒体资源
 *        file:///fatfs0/Music/1.mp3  SD卡中的音频文件
 *        http://.../1.mp3            http音频
 *        chunkfifo://tts/1           播放云端反馈的tts流
 *        mem://addr=%u&size=%u       播放存放在ram中资源
 * @return 0:成功
 */
int aui_player_play(int type, const char *url);

/**
 * 暂停播放
 * 
 * @param type 支持MEDIA_MUSIC,MEDIA_SYSTEM
 * @return 0:成功
 */
int aui_player_pause(int type);

/**
 * 暂停状态继续播放和静音状态恢复播放
 * 
 * @param type 支持MEDIA_MUSIC,MEDIA_SYSTEM,
 * @return 0:成功
 */
int aui_player_resume(int type);

/**
 * 停止播放器
 * 
 * @param type 支持MEDIA_MUSIC,MEDIA_SYSTEM,MEDIA_ALL
 * @return 0:成功
 */
int aui_player_stop(int type);

/**
 * 播放器静音,调用aui_player_resume接口恢复音频输出
 *
 * @param type 支持MEDIA_MUSIC,MEDIA_SYSTEM,MEDIA_ALL
 * @return 0:成功
 */
int aui_player_mute(int type);

/**
 * 调整音量
 * 
 * @param type 支持MEDIA_MUSIC,MEDIA_SYSTEM,,MEDIA_ALL
 * @param inc_volume 正数加大，负数减小
 * @return 0:成功
 */
int aui_player_vol_adjust(int type, int inc_volume);

/**
 * 调整音量到指定值
 *
 * @param type 支持MEDIA_MUSIC,MEDIA_SYSTEM,MEDIA_ALL
 * @param volume 指定音量的百分比 0~100
 * @return 0:成功
 */
int aui_player_vol_set(int type, int volume);

/**
 * 渐变调整音量到指定值
 * 
 * @param type 支持MEDIA_MUSIC,MEDIA_SYSTEM,
 * @param new_volume 目标音量百分比0~100
 * @param ms 渐变时间
 * @return 0:成功
 */
int aui_player_vol_gradual(int type, int new_volume, int ms);

/**
 * 获取当前音量值
 *
 * @param type 支持MEDIA_MUSIC,MEDIA_SYSTEM
 * @return 音量值
 */
int aui_player_vol_get(int type);

/**
 * 设置播放时最小音量
 *
 * @param type 支持MEDIA_MUSIC,MEDIA_SYSTEM,MEDIA_ALL
 * @param volume 指定音量的百分比 0~100
 * @return 0:成功
 */
int aui_player_set_minvol(int type, int volume);

/**
 * 获取指定播放器的状态
 *
 * @param type 支持MEDIA_MUSIC,MEDIA_SYSTEM
 * @return aui_player_state_t
 */
aui_player_state_t aui_player_get_state(int type);

/**
 * 通知播放完成后恢复音乐播放(有音量渐变效果)
 *
 * @return 0:成功
 */
int aui_player_resume_music(void);

/*************************************************
 * 麦克风服务
 *************************************************/
typedef enum {
    MIC_CTRL_START_PCM,      /* 启动音频数据传输 */
    MIC_CTRL_STOP_PCM,       /* 停止音频数据传输 */
    MIC_CTRL_START_SESSION,  /* 强制进入对话模式 */
    MIC_END
} mic_ctrl_cmd_t;

typedef enum {
    MIC_STATE_IDLE,    /* 空闲 */
    MIC_STATE_SESSION, /* 对话状态 */
    MIC_STATE_PCM,     /* 正在传输音频数据 */

    MIC_STATE_END
} mic_state_t;

typedef enum {
    MIC_EVENT_PCM_DATA,      /* 有音频数据 */
    MIC_EVENT_SESSION_START, /* 开始对话 */
    MIC_EVENT_SESSION_STOP   /* 停止对话 */
} mic_event_id_t;

/**
 * 麦克风事件回调
 * @param source 预留，数据来源
 * @param evt_id 参见mic_event_id_t枚举的说明
 * @param data 事件参数
 *        若是MIC_EVENT_PCM_DATA事件，data为音频数据
 *        若是MIC_EVENT_SESSION_START事件，data指针指向一个整型，值为0时表示强制唤醒，10为标准语音唤醒，其他值预留
 * @param size 参数的字节数
 */
typedef void (*aui_mic_evt_t)(int source, mic_event_id_t evt_id, void *data, int size);

/**
 * 启动麦克风服务
 * @param task 传入服务绑定的任务
 * @return 0:成功
 */
int aui_mic_start(utask_t *task, aui_mic_evt_t evt_cb);

/**
 * 停止麦克风服务
 * @return 0:成功
 */
int aui_mic_stop(void);

/**
 * 设置麦克风音频来源
 * @pram source 音频来源
 * @return 0:成功
 */
int aui_mic_set_active(int source);

/**
 * 麦克风控制命令
 * @pram cmd 控制命令
 * @return 0:成功
 */
int aui_mic_control(mic_ctrl_cmd_t cmd);

/**
 * 获取麦克风状态
 * @pram state 输出状态值
 * @return 0:成功
 */
int aui_mic_get_state(mic_state_t *state);

/*************************************************
 * 麦克风适配接口
 *************************************************/
typedef struct __mic mic_t;

typedef struct mic_hw_param {
    int sample_bits;  /* 音频采样位 16bit*/
    int channels;     /* 通道个数 1/2 */
    int rate;         /* 音频采样率 16000*/
    int encode;       /* 编码格式 0:PCM */
    int source;       /* 预留，用于控制使能哪些麦克风 */
} mic_hw_param_t;

typedef struct mic_sw_param {
    int buffer_size;      /* 麦克风驱动每次返回的最大数据量 */
    int buffer_num;       /* 预留，用于控制麦克风驱动使用的缓存个数 */
    int start_threshold;  /* 预留 */
    int sentence_time_ms; /* 断句时间 */
} mic_sw_param_t;

/**
 * MIC 事件回调处理
 * @param priv 用于传递私有数据
 * @param event_id 麦克风事件
 * @param data 事件参数数据指针
 * @param size 数据字节数
 */
typedef void (*mic_event_t)(void *priv, mic_event_id_t evt_id, void *data, int size);

typedef struct mic_ops {
    int (*init)(mic_t *mic, mic_event_t pcm_event);
    int (*deinit)(mic_t *mic);
    int (*asr_control)(mic_t *mic, int flag);
    int (*asr_wake)(mic_t *mic, int flag);
    int (*pcm_data_control)(mic_t *mic, int flag);
    int (*pcm_aec_control)(mic_t *mic, int flag);
    int (*pcm_set_param)(mic_t *mic, void *param);
    int (*pcm_get_param)(mic_t *mic, void *param);
} mic_ops_t;

int mic_set_privdata(void *priv);
void *mic_get_privdata(void);
int mic_ops_register(mic_ops_t *ops);

/*************************************************
 * 已适配的麦克风
 *************************************************/
/**
 * 基于yunvoice接口的麦克风设备注册
 * @param ver 1:sc5654  2:yunvoice2
 */
void yunvoice_mic_register(int ver);

#endif
