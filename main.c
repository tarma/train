#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "tractionCaculator.h"

#define tStep 0.5 
#define gamma 0.8
#define alpha 1
#define epsilon 0.1
#define blame 10

int velocityToLevel(float velocity);

int main() {
	float Q[21][5][17] = {0};
	int i, j, times, k;
	int count = 0;

	srand(time(NULL));

	LOCOPARAMETER* locoInfoPtr = initLocoInfo();
	OPTCONSTPARAM* optConstPtr = initOptConst();
	initModel(locoInfoPtr, optConstPtr);

	for (times = 0; times < 1000000; times++) {
		int state = 0;
		float s = mGradients[0].start;
		int gear = 0;
		float velocity = 0;
		do {
			int action;
			int level = velocityToLevel(velocity);
			float p = (float) rand() / RAND_MAX;
			if (p > epsilon) {
			 	if (gear == 8) {
					if (Q[state][level][15] > Q[state][level][16]) {
						action = 0;
					} else {
						action = 1;
					}
				} else if (gear == -8) {
					if (Q[state][level][0] > Q[state][level][1]) {
						action = 1;
					} else {
						action = 2;
					}
				} else {
					float max = Q[state][level][gear + 9];
					action = 2;
					for (i = gear + 7; i < gear + 9; i++) {
						if (Q[state][level][i] > max) {
							action = i - gear - 7;
							max = Q[state][level][i];
						}
					}
				}
			} else { 
				if (gear == 8) {
					action = rand() % 2;
				} else if (gear == -8) {
					action = rand() % 2 + 1;
				} else {
					action = rand() % 3;
				}
			}
			float delta_s;
			float delta_v;
			float delta_e;
			float reward = 0;
			int next_state = state;
			int new_gear = gear + action - 1;

			DoCaculateByTime(s, velocity, new_gear, tStep, &count, &delta_s, &delta_v, &delta_e);
			if (s + delta_s > mGradients[state].end) {
				next_state++;
			}
			velocity += delta_v;
			int next_level = velocityToLevel(velocity);
			float max;
			if (new_gear == 8) {
				if (Q[next_state][next_level][15] > Q[next_state][next_level][16]) {
					max = Q[next_state][next_level][15];
				} else {
					max = Q[next_state][next_level][16];
				}
			} else if (new_gear == -8) {
				if (Q[next_state][next_level][0] > Q[next_state][next_level][1]) {
					max = Q[next_state][next_level][0];
				} else {
					max = Q[next_state][next_level][1];
				}
			} else {
				max = Q[next_state][next_level][gear + 9];
				for (i = gear + 7; i < gear + 9; i++) {
					if (Q[next_state][next_level][i] > max) {
						max = Q[next_state][next_level][i];
					}
				}
			}
			reward += -delta_e + delta_s;
			if ((int) (velocity + 0.5) < 0) {
				reward -= blame;
				Q[state][level][gear + 8] += alpha * (reward + gamma * max - Q[state][level][gear + 8]);
				break;
			}
			Q[state][level][gear + 8] += alpha * (reward + gamma * max - Q[state][level][gear + 8]);
			
			state = next_state;
			s += delta_s;
			gear = new_gear;
		} while (s < mGradients[19].end);
	}

	for (i = 0; i < 20; i++) {
		printf("%d\n", i + 1);
		for (j = 0; j < 17; j++) {
			for (k = 0; k < 5; k++) {
				printf("%f ", Q[i][k][j]);
			}
			printf("\n");
		}
		printf("\n");
	}

	dispose();

	return 0;
}

int velocityToLevel(float velocity) {
	int v = (int) (velocity + 0.5);
	if (v / 10 < 4) {
		return v / 10;
	}
	return 4;
}
