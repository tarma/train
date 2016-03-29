/**
 * @file    tractionCaculator.c
 * @brief  提供牵引计算模型的基本方法实现
 * @date  2014.4.1
 * @author sinlly
 * @note 包括多质点模型以及bdforce的相关计算
 */
#include "tractionCaculator.h"

LOCOPARAMETER locoInfo;
MGRADIENT* mGradients;
CARSTRUCT* carStruct;/**<保存每个车厢的基本信息*/
OPTCONSTPARAM opt_const;/**<优化中使用到的常量*/
int MAX_MGRADIENT;
int CARNUMS;/**<车厢节数*/
float m_loco; // train head weight
float l_loco; // train head length
float l_car; //average length for a car
float m_car; //average weight for a car
float dragForce[8][120];
float brakeForce[8][120];

// 初始化
LOCOPARAMETER* initLocoInfo() {
	LOCOPARAMETER* locoInfoPtr = (LOCOPARAMETER*) malloc(sizeof(LOCOPARAMETER));
	
	locoInfoPtr->count = 56;
	locoInfoPtr->weight[0] = 0;
	locoInfoPtr->totalWeight = locoInfoPtr->weight[0] + 5000;
	locoInfoPtr->weight[1] = locoInfoPtr->totalWeight / locoInfoPtr->count;
	locoInfoPtr->length[0] = 0;
	locoInfoPtr->totalLength = locoInfoPtr->length[0] + 69.8;
	locoInfoPtr->length[1] = locoInfoPtr->length[0] / locoInfoPtr->count;
	locoInfoPtr->davis[0] = 0.0001;
	locoInfoPtr->davis[1] = -0.0104;
	locoInfoPtr->davis[2] = 1.3889;
	locoInfoPtr->dragConsumption[0] = 0.0211550441452778;
	locoInfoPtr->dragConsumption[1] = 0.0385553514500000;
	locoInfoPtr->dragConsumption[2] = 0.0729779724177778;
	locoInfoPtr->dragConsumption[3] = 0.103948251458333;
	locoInfoPtr->dragConsumption[4] = 0.138471670730556;
	locoInfoPtr->dragConsumption[5] = 0.182041737826667;
	locoInfoPtr->dragConsumption[6] = 0.221756269777778;
	locoInfoPtr->dragConsumption[7] = 0.279589296952778;
	locoInfoPtr->idleConsumption = 0.00516591310277778;
	locoInfoPtr->brakeConsumption[0] = 0.00812686329583333;
	locoInfoPtr->brakeConsumption[1] = 0.00812686329583333;
	locoInfoPtr->brakeConsumption[2] = 0.00812686329583333;
	locoInfoPtr->brakeConsumption[3] = 0.00812686329583333;
	locoInfoPtr->brakeConsumption[4] = 0.00812686329583333;
	locoInfoPtr->brakeConsumption[5] = 0.00812686329583333;
	locoInfoPtr->brakeConsumption[6] = 0.00812686329583333;
	locoInfoPtr->brakeConsumption[7] = 0.00812686329583333;

	return locoInfoPtr;
}

OPTCONSTPARAM* initOptConst() {
	OPTCONSTPARAM* optConstPtr = (OPTCONSTPARAM*) malloc(sizeof(OPTCONSTPARAM));
	
	optConstPtr->G = 9.8;
	optConstPtr->PRECISION = 0.000001;
	optConstPtr->EMPTYINTERVAL = 3.0;
	optConstPtr->LIMIT_INTERVAL = 8;
	optConstPtr->DELTA_S = 5.0;
	optConstPtr->TSTEP = 0.5;
	optConstPtr->SSTEP = 1.0;
	optConstPtr->MAXV = 72.0;
	optConstPtr->AIRLENGTH = 1000.0;
	optConstPtr->MAXLENGTH = 1000.0;
	optConstPtr->LIMITV = 75.0;
	optConstPtr->MINV = 20.0;
	optConstPtr->MAXGEAR = 8;
	optConstPtr->MINGEAR = -6;
	optConstPtr->AIRS = -10;
	optConstPtr->AIRE = -9;
	optConstPtr->DISTANCE = 100;
	optConstPtr->FEATUREINVALID = 100;
	optConstPtr->LIMITTHRESHOLD = 380;
	optConstPtr->THROUGHEXPERT = -11;
	optConstPtr->PUSHERENGINE = -12;
	optConstPtr->COMGEARSTEP = 1.0;
	optConstPtr->IDELGEARSTEP = 1.0;
	optConstPtr->NEARLIMIT = 8.0;
	optConstPtr->POSTINITLENGTH = 10000;
	optConstPtr->GEARLENGTHLIMIT = 200;
	optConstPtr->TMPSPEEDACCURACY = 0.1;
	optConstPtr->MAXDRAG = 50;
	optConstPtr->RAWOPT_V_INTERVAL = 1.0;
	optConstPtr->MAX_ADJUST_V = 5.0;
	optConstPtr->PERCENTAGE_OF_DISTANCE = 0.6;
	optConstPtr->QUICK_RISE_GEAR = 8;
	optConstPtr->QUICK_DROP_GEAR = 6;
	optConstPtr->CATCH_UP_LEN_INTERVAL = 1000.0;
	optConstPtr->MAX_TIME_DIFF = 2;
	
	return optConstPtr;
}

int initModel(LOCOPARAMETER* locoInfoPtr, OPTCONSTPARAM* optConstPtr) {

	locoInfo = *locoInfoPtr;
	m_loco = locoInfo.weight[0];
	m_car = locoInfo.weight[1];
	l_loco = locoInfo.length[0];
	l_car = locoInfo.length[1];

	opt_const = *optConstPtr;

	carStruct = (CARSTRUCT*) malloc(sizeof(CARSTRUCT) * locoInfo.count);

	int i;
	CARSTRUCT* curCar;
	for (i = 0; i < locoInfo.count; i++) {
		curCar = carStruct + i;
		curCar->carLength = locoInfo.totalLength / locoInfo.count;
		curCar->carWeight = locoInfo.totalWeight / locoInfo.count;
		curCar->loadWeight = 0.0f;
	}

	// 读取加算坡度
	readMGradient();
	// 读取牵引力和制动力
	readDragAndBrakeForce();

	return 1;
}

/**
 * @brief 对float类型数据进行四舍五入到整数
 * @param v  需要四舍五入的数
 * @return  四舍五入后的整数
 */
int roundInt(float v) {
	return (int) (v + 0.5);
}

/**
 * @brief 根据机车的档位和速度获得机车的作用力
 * @param gear  驾驶档位
 * @param v   速度
 * @return  作用力
 */
void getForce(int gear, float v, float *force) {

	int v1 = roundInt(v); //对v进行四舍五入

	if (v1 == 0) v1 = 1;

	if (gear < 0) {
		*force = -brakeForce[abs(gear) - 1][v1 - 1];
	} else {
		if (gear > 0)
			*force = dragForce[gear - 1][v1 - 1];
		else
			*force = 0.0;
	}
}
/**
 * @brief 根据档位和时间获得油耗
 * @param gear  档位
 * @param tStep  时间
 * @return   油耗
 */
float getConsumption(int gear, float tStep) {

	float delta_e = 0;
	if (gear > 0)
		delta_e = tStep * locoInfo.dragConsumption[gear - 1];
	else {
		if (gear == 0)
			delta_e = tStep * locoInfo.idleConsumption;
		else
			delta_e = tStep * locoInfo.brakeConsumption[-gear - 1];

	}
	return delta_e;
}
/**
 * @brief 多指点模型计算坡度
 * @param start  当前公里标
 * @param count  roadCategory的index,只是为了快速索引，从0开始也可以
 * @return  多质点计算之后的坡度
 */
float multiParticalModel(float start, int *count) {
//	m_loco = locoInfo.weight[0]; // train head weight
//	l_loco = locoInfo.length[0]; // train head length
	m_loco = 0.0; // train head weight
	l_loco = 0.0; // train head length
//	l_car = (locoInfo.totalLength - l_loco) / locoInfo.count; //average length for a car
//	m_car = (locoInfo.totalWeight - m_loco) / locoInfo.count; //average weight for a car
	float w_loco = 0;
	float x_loco = start - l_loco / 2.0;
	float grad = 0.0;
	if (x_loco < (float) (mGradients[*count].start)) {   //前行方向为前方，反向为后方
		if (*count == 0) //首段后方
			grad = 0.0;
		else {
			while (1) {
				*count = *count - 1; //后移至相应段
				if (x_loco > (float) mGradients[*count].start
						|| fabs(x_loco - (float) mGradients[*count].start)
								< opt_const.PRECISION) {
					grad = mGradients[*count].value;
					break;
				} else {
					if (*count == 0) {
						grad = 0.0;
						break;
					}
				}
			}
		}
	} else {
		if (x_loco > (float) mGradients[*count].end) {
			if (*count == MAX_MGRADIENT - 1) //末端前方
				grad = 0.0;
			else {
				while (1) {
					*count = *count + 1;
					if (x_loco < (float) mGradients[*count].end
							|| fabs(x_loco - (float) mGradients[*count].end)
									< opt_const.PRECISION) {
						grad = mGradients[*count].value;
						break;
					} else {
						if (*count == MAX_MGRADIENT - 1) {
							grad = 0.0;
							break;
						}
					}
				}
			}
		} else
			grad = mGradients[*count].value;
	}
	w_loco = w_loco + m_loco * grad;
	int count_car = *count;
	l_car = carStruct[0].carLength;
	float x_car = x_loco - l_loco / 2.0 - l_car / 2.0;
	float w_cars = 0.0;
	float temp_start = (float) mGradients[count_car].start;
	float temp_end = (float) mGradients[count_car].end;
	int i = 0;
	for (i = 0; i < locoInfo.count; i++) {    //第n个car坡度阻力
		if ((x_car > temp_start
				|| fabs(x_car - temp_start) < opt_const.PRECISION)
				&& (x_car < temp_end
						|| fabs(x_car - temp_end) < opt_const.PRECISION))
			grad = mGradients[count_car].value;
		else {
			if (x_car < temp_start)
			//前行方向为前方，反向为后方
					{
				if (count_car == 0) //首段后方
					grad = 0.0;
				else {
					while (1) {
						count_car = count_car - 1; //后段移至相应段
						if (x_car > (float) mGradients[count_car].start
								|| fabs(
										x_car
												- (float) mGradients[count_car].start)
										< opt_const.PRECISION) {
							grad = mGradients[count_car].value;
							break;
						} else {
							if (count_car == 1) {
								grad = 0.0;
								break;
							}
						}
					}
					temp_start = (float) mGradients[count_car].start;
					temp_end = (float) mGradients[count_car].end;
				}
			} else {
				if (x_car > (float) mGradients[count_car].end) {
					if (count_car == MAX_MGRADIENT - 1) //末端前方
						grad = 0.0;
					else {
						while (1) {
							count_car = count_car + 1;
							if (x_car < (float) mGradients[count_car].end
									|| fabs(
											x_car
													- (float) mGradients[count_car].end)
											< opt_const.PRECISION) {
								grad = mGradients[count_car].value;
								break;
							} else {
								if (count_car == MAX_MGRADIENT - 1) {
									grad = 0.0;
									break;
								}
							}
						}
						temp_start = (float) mGradients[count_car].start;
						temp_end = (float) mGradients[count_car].end;
					}
				}
			}
		}
		m_car = carStruct[i].carWeight + carStruct[i].loadWeight;
		float w_car = m_car * grad;
		w_cars = w_cars + w_car;
		if (i + 1 != CARNUMS)
			l_car = carStruct[i + 1].carLength;
		x_car = x_car - l_car;
	}
	return (w_loco + w_cars) / (locoInfo.totalWeight);
}

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
		float *delta_s, float *delta_v, float *delta_e) {
	//优化后的参数
	float w_o = locoInfo.davis[0] * v * v + locoInfo.davis[1] * v
			+ locoInfo.davis[2];
	float gradient = multiParticalModel(start, count);
	float w = w_o + gradient; //总阻力
	float df;

	getForce(gear, v, &df);

//	float dft = 1000/G*df/(m_loco + locoInfo.totalWeight); //转换后的最大牵引力
	float dft = 1000.0 / opt_const.G * df / locoInfo.totalWeight; //转换后的最大牵引力
	float c = dft - w;
	float acc = c / 30.0;
	*delta_v = acc * tStep;
	*delta_s = (v / 3.6) * tStep;
	*delta_e = getConsumption(gear, tStep);
}

// 获取文件的行数
int get_file_line_num(FILE *fp) {
	int n = 0;
	int ch;
	while ((ch = fgetc(fp)) != EOF) {
		if (ch == '\n') {
			n++;
		}
	}
	rewind(fp);
	return n;
}

// 读取加算坡度
void readMGradient() {

	FILE *fp;

	if ((fp = fopen("./optimize_data/mGradient", "rt")) == NULL) {
		perror("can not find file!");
		exit(1);
	}

	MAX_MGRADIENT = get_file_line_num(fp);

	mGradients = (MGRADIENT*) malloc(sizeof(MGRADIENT) * MAX_MGRADIENT);

	int i;
	MGRADIENT* gradient;
	for (i = 0; i < MAX_MGRADIENT; i++) {
		gradient = mGradients + i;
		fscanf(fp, "%d\t%d\t%f", &gradient->start, &gradient->end,
				&gradient->value);
	}

	fclose(fp);
}

// 读取牵引力和制动力
void readDragAndBrakeForce() {

	// 读取牵引力
	FILE *fp1;

	if ((fp1 = fopen("./train_data/drag_force", "r")) == NULL) {
		perror("can not find drag_force!");
		exit(1);
	}

	int i, j;
	for (i = 0; i < 120; i++) {
		for (j = 0; j < 8; j++) {
			fscanf(fp1, "%f", &dragForce[j][i]);
		}
	}

	fclose(fp1);

	// 读取制动力
	FILE *fp2;

	if ((fp2 = fopen("./train_data/brake_force", "r")) == NULL) {
		perror("can not find brake_force!");
		exit(1);
	}

	for (i = 0; i < 120; i++) {
		for (j = 0; j < 8; j++) {
			fscanf(fp2, "%f", &brakeForce[j][i]);
		}
	}

	fclose(fp2);
}

// 程序结束处理
void dispose() {
	free(mGradients);
	free(carStruct);
}

float getValue(int n) {
	return 10.0;
}
