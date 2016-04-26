
#include <stdio.h>
#include <stdlib.h>
#include "read_binary_for_route.h"

static long get_file_size(FILE *fp)
{
	long size;
	fseek(fp, 0, SEEK_END);
	size = ftell(fp);
	rewind(fp);
	return size;
}

void *read_binary_file(int type, int *length)
{
	struct gradient_t *gra;
	struct curve_t *cur;
	struct limit_t *lim;
	struct tele_t *tele;
	struct station_t *sta;
	struct tunnel_t *tun;
	FILE *fp;
	long f_size;
	int  nmeb;
	switch(type)
	{
	case 0x01:
		fp = fopen("./route_data/gradient_b", "rb");
		if(fp == NULL){
			perror("gradient_b not exist");
			exit(EXIT_FAILURE);
		}
		f_size = get_file_size(fp);
		nmeb = f_size/sizeof(struct gradient_t);
		gra = (struct gradient_t *)malloc(sizeof(struct gradient_t) * nmeb);
		fread(gra, sizeof(struct gradient_t), nmeb, fp);
		*length = nmeb;
		return gra;
		break;
	case 0x02:
		fp = fopen("./route_data/curve_b", "rb");
		if(fp == NULL){
			perror("curve_b not exist");
			exit(EXIT_FAILURE);
		}
		f_size = get_file_size(fp);
		nmeb = f_size/sizeof(struct curve_t);
		cur = (struct curve_t *)malloc(sizeof(struct curve_t) * nmeb);
		fread(cur, sizeof(struct curve_t), nmeb, fp);
		*length = nmeb;
		return cur;
		break;
	case 0x03:
		fp = fopen("./route_data/limit_b", "rb");
		if(fp == NULL){
			perror("limit_b not exist");
			exit(EXIT_FAILURE);
		}
		f_size = get_file_size(fp);
		nmeb = f_size/sizeof(struct limit_t);
		lim = (struct limit_t *)malloc(sizeof(struct limit_t) * nmeb);
		fread(lim, sizeof(struct limit_t), nmeb, fp);
		*length = nmeb;
		return lim;
		break;
	case 0x04:
		fp = fopen("./route_data/teleseme_b", "rb");
		if(fp == NULL){
			perror("teleseme_b not exist");
			exit(EXIT_FAILURE);
		}
		f_size = get_file_size(fp);
		nmeb = f_size/sizeof(struct tele_t);
		tele = (struct tele_t *)malloc(sizeof(struct tele_t) * nmeb);
		fread(tele, sizeof(struct tele_t), nmeb, fp);
		*length = nmeb;
		return tele;
		break;
	case 0x05:
		fp = fopen("./route_data/station_b", "rb");
		if(fp == NULL){
			perror("station_b not exist");
			exit(EXIT_FAILURE);
		}
		f_size = get_file_size(fp);
		nmeb = f_size/sizeof(struct station_t);
		sta = (struct station_t *)malloc(sizeof(struct station_t) * nmeb);
		fread(sta, sizeof(struct station_t), nmeb, fp);
		*length = nmeb;
		return sta;
		break;
	case 0x06:
		fp = fopen("./route_data/tunnel_b", "rb");
		if(fp == NULL){
			perror("tunnel_b not exist");
			exit(EXIT_FAILURE);
		}
		f_size = get_file_size(fp);
		nmeb = f_size/sizeof(struct tunnel_t);
		tun = (struct tunnel_t *)malloc(sizeof(struct tunnel_t) * nmeb);
		fread(tun, sizeof(struct tunnel_t), nmeb, fp);
		*length = nmeb;
		return tun;
		break;

	default:
		return NULL;
	}
	return NULL;
}

