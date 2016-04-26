

#ifndef READ_BINARY_FOR_TEST_H_
#define READ_BINARY_FOR_TEST_H_

#include <stdint.h>
/* 信号机 */
struct tele_t
{
    uint8_t  t_type;            /* 信号机类型 */
    uint8_t  t_trend;           /* 信号机趋势 */
    uint16_t t_num;             /* 信号机编号 */
    uint16_t dr_num;            /* 交路号 */
    uint16_t sp_limit_code;     /* speed limit code 限速代号 */
    uint32_t distance;          /* 距离后方信号机距离 */
    int32_t  discont_km_post;   /* 原始信号机公里标 */
    int32_t  cont_km_post;      /* 连续信号机公里标 */
    
};

/* 股道信息 */ 
struct sidetrack_t
{
    uint8_t  st_num;            /* 股道号 */
    uint16_t in_dist;           /* 进岔距离 */
    uint16_t out_dist;          /* 出岔距离 */
    uint16_t in_limit_code;     /* 进岔限速代号 */
    uint16_t out_limit_code;    /* 出岔限速代号 */
    int16_t  adjust_dist;       /* 修正距离 */
    struct sidetrack_t *next;
};


/* 车站 */
struct station_t
{
    uint16_t s_num;                 /* station number */
    uint16_t dr_num;                /* data route number */
    uint32_t tmis_num;              /* tmis number */
    uint16_t l_num;                 /* line number */
    uint16_t tele_num;
    uint16_t cen_dist;
    uint32_t km_post;
};

struct tunnel_t
{
    uint16_t tele_num;
    uint16_t cross_dist;
    uint32_t km_post;
    uint32_t length;
};

/* 从base_data文件中提取的坡度，曲线，限速信息 */
struct gradient_t
{
    uint16_t tele_num;  /* 所属信号机编号 */
    uint16_t length;    /* 长度 */
    uint32_t cross_dis; /* 越过距离 */
    int32_t  km_post;   /* 连续公里标 */
    float value;        /* 坡度值 +:上坡; -:下坡 */
};

struct curve_t
{
    uint16_t tele_num;
    uint32_t length;
    uint32_t cross_dis; /* 越过距离 */
    int32_t  km_post;   /* 起始公里标 */
    uint16_t value;     /* 曲线值 */
    int dir;            /* 曲线方向 1:左 -1:右 */
};

/* 只包含base_data中的限速信息，不包括:信号机限速,侧线点限速,揭示信息限速 */
struct limit_t
{
    uint16_t tele_num;
    uint16_t cross_dis;
    uint16_t subregion_dis;
    uint16_t code;
    uint16_t value;
    uint32_t length;
    int32_t  km_post;   /* 起始公里标 */
    int32_t  end_post;  /* 终止公里标 */
};

        
#ifdef __cplusplus
extern "C" {
#endif

 /**
 * @function read_binary_file
 * @brief 从二进制文件中读取上述定义的数据
 * @param type 数据类型: 0x01--gradient 0x02--curve 0x03--limit 0x04--teleseme 0x05--station 
 *                       0x06--tunnel
 * @param length 返回数据长度
 * @return 成功返回相应的数据
 * @example:
 *          struct gradient_t *gra;
 *          int length;
 *          gra = (struct gradient_t *)read_binary_file(0x00, &length);
 **/  
void* read_binary_file(int type,int *length);

#ifdef __cplusplus
}
#endif
#endif

