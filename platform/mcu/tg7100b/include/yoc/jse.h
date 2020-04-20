#ifndef YOC_JSE_H
#define YOC_JSE_H


void jse_init();

/*
 * BoneEngine初始化
 */
void bone_engine_init();

/*
 * 运行JS程序
 *   可以多次调用, BoneEngine符号表共享
 */
void bone_engine_start(const char* js);

/*
 * BoneEngine复位之后重新运行JS程序
 */
void bone_engine_restart(const char* js);

/*
 * 退出BoneEngine
 */
void bone_engine_exit();

#endif
