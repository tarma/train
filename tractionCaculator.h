/**
 * @file    tractionCaculator.h
 * @brief  提供牵引计算模型的基本方法接口
 * @date  2014.4.8
 * @author sinlly
 * @note 包括 作用力、油耗的计算,单位时间行驶距离的计算, 单位距离行驶时间的计算,多质点模型中等效坡度的计算
 */
#ifndef TRACTIONCACULATOR_H_
#define TRACTIONCACULATOR_H_
//#include "tractionCaculator.h"
//#include <math.h>
//#include <string.h>
#include <stdio.h>
#include <stdlib.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *  机车参数
 */
struct locoParameter  //机车参数
{
	int count; /**< 辆数 */
	float weight[2]; /**< 包括车头weight[0]为车头的重量,weight[1]为单节车厢平均重量 */
	float length[2]; /**< length[0]100为车头的质量，length[1]为单节车厢平均长度 */
	float totalLength; /**< 车总长度  include train head */
	float totalWeight;/**< 车总重量  include train head */
	float davis[3]; /**< 戴维斯系数 */
	float dragConsumption[8];/**< dGearE 牵引各档位耗油量 */
	float idleConsumption; /**< cGearE  惰性档位耗油量*/
	float brakeConsumption[8]; /**< bGearE 制动各档位耗油量 */
};
typedef struct locoParameter LOCOPARAMETER;

/**
 * 机车编组信息结构(每节车厢的长度和重量)
 */
struct carStruct {
	float carLength; /**< 每节车厢的长度 */
	float carWeight;/**< 每节车厢的重量 */
	float loadWeight;/**< 载重 */
};
typedef struct carStruct CARSTRUCT;

/**
 *   加算坡度
 */
struct mGradient {
	int start; /**< 起始公里标*/
	int end; /**< 终止 公里标*/
	float value; /**< 坡度值 */
};
typedef struct mGradient MGRADIENT;

extern MGRADIENT* mGradients;

/**
 * 优化中用到的常量值，从配置文件中读取
 */
struct optConstParam {
	float G; /**<重力加速度 */
	float PRECISION; /**<判断浮点类型是否相等的精度*/
	float EMPTYINTERVAL;/**<空车陡上坡的用于调整平均速度的阈值 */
	float LIMIT_INTERVAL;/**<限速调整中，所用限速比正常限速低的阈值*/
	float DELTA_S;/**<距离步长*/
	float TSTEP;/**<计算时间步长*/
	float SSTEP;/**<计算空气制动的距离步长*/
	float MAXV;/**<标记机车及限速理论允许的最大恒速值*/
	float AIRLENGTH;/**<空气制动提前开启的距离*/
	float MAXLENGTH;/**<设定的追上曲线的最大距离*/
	float LIMITV;/**<设定的最大限速值*/
	float MINV;/**<标记自动驾驶允许的最低速度*/
	int MAXGEAR;/**<标记机车最大牵引档位*/
	int MINGEAR;/**<标记机车最小的制动档位*/
	int AIRS;/**<用于标记空气制动降速起始位置档位*/
	int AIRE;/**<用于标记空气制动降速结束匀速开始位置档位*/
	int DISTANCE;/**<坡段中划分出小的坡段的长度阈值*/
	int FEATUREINVALID;/**<策略配置文件中对应特征无效的标识*/
	int LIMITTHRESHOLD;/**<限速调整阈值，调整限速起始公里标的值*/
	int THROUGHEXPERT;/**<手动区域中贯通实验标识*/
	int PUSHERENGINE;/**<手动区域中非贯通实验即补机段标识*/
	float COMGEARSTEP;/**<档位渐变中普通档位换档的持续时间*/
	float IDELGEARSTEP;/**<档位渐变中惰行档位换档的持续时间*/
	float NEARLIMIT;/**<档位拉平过程中，判断是否距离距离限速较近的速度阈值*/
	int POSTINITLENGTH;/**<优化中拉平档位所新生成的曲线的初始长度（临时限速拉平所新生成的曲线）*/
	float GEARLENGTHLIMIT;/**<档位拉平中，需要进行档位拉平的最大的档位长度，若档位长度小于该值，则需要进行拉平*/
	float TMPSPEEDACCURACY;/**<临时限速中档位跳跃追上原始曲线的点的速度差阈值(连接起来的精度)*/
	float MAXDRAG;/**< 牵引计算中阻力的最大值*/
	float RAWOPT_V_INTERVAL; /**<在原始优化中保持匀速运行的速度阈值*/
	float MAX_ADJUST_V; /**<在时间偏差调整中所能调整的最大速度*/
	float PERCENTAGE_OF_DISTANCE; /**<超过两车站不再升档（降档）的比例*/
	int QUICK_RISE_GEAR; /**<快速升速提升的档位*/
	int QUICK_DROP_GEAR; /**<快速降速下降的档位*/
	float CATCH_UP_LEN_INTERVAL; /**<以原档位运行判断能否追上原曲线距离到达车站的距离阈值*/
	int MAX_TIME_DIFF; /**<最大允许的时间偏差*/
};
typedef struct optConstParam OPTCONSTPARAM;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  对单位时间行驶的距离，油耗，速度变化量计算
 * @param start  起始位置
 * @param v   当前速度
 * @param gear  当前档位
 * @param tStep   单位时间
 * @param count   roadCategory的index,只是为了快速索引，从0开始也可以
 * @param delta_s  单位时间所行驶的距离
 * @param delta_v  单位时间速度变化量
 * @param delta_e  单位时间消耗油耗
 * @return  成功标识为1,非成功标识0
 */
void DoCaculateByTime(float start, float v, int gear, float tStep, int *count,
		float *delta_s, float *delta_v, float *delta_e);  //单位时间行驶距离的计算

/**
 * @brief 多指点模型计算坡度
 * @param start  当前公里标
 * @param count  roadCategory的index,只是为了快速索引，从0开始也可以
 * @return  多质点计算之后的坡度
 */
float multiParticalModel(float start, int *count);  //多质点模型
/**
 * @brief 根据档位和时间获得油耗
 * @param gear  档位
 * @param tStep  时间
 * @return   油耗
 */

// 获取文档行数
int get_file_line_num(FILE *fp);

// 从文件中读取加算坡度
void readMGradient();

// 读取牵引力和制动力
void readDragAndBrakeForce();

// 程序结束处理
void dispose();

// 初始化
LOCOPARAMETER* initLocoInfo();
OPTCONSTPARAM* initOptConst();
int initModel(LOCOPARAMETER* locoInfoPtr, OPTCONSTPARAM* optConstPtr);

// Debug使用
float getValue(int n);

#endif /* TRACTIONCACULATOR_H_ */
