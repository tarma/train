#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "tractionCaculator.h"

#define tStep 0.5 
#define gamma 0.8
#define alpha 0.5
#define epsilon 0.1
#define blame 500

int velocityToLevel(float velocity);

int main() {
	float Q[51][12][17] = {0};
	int i, j, times, k;
	int count = 0;
	FILE *fout = fopen("result", "w+");
	FILE *result = fopen("gear_result", "w+");
	float s;
	int gear, new_gear;
	float velocity;
	int state;

	srand(time(NULL));

	LOCOPARAMETER* locoInfoPtr = initLocoInfo();
	OPTCONSTPARAM* optConstPtr = initOptConst();
	initModel(locoInfoPtr, optConstPtr);

	for (times = 0; times < 500000; times++) {
		state = 0;
		s = mGradients[0].start;
		gear = 0;
		velocity = 0;
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
			new_gear = gear + action - 1;

			DoCaculateByTime(s, velocity, new_gear, tStep, &count, &delta_s, &delta_v, &delta_e);
			while (s + delta_s > mGradients[next_state].end) {
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
			reward += delta_s / delta_e;
			if ((int) (velocity + 0.5) < 0) {
				reward -= blame;
				Q[state][level][gear + 8] += alpha * (reward + gamma * max - Q[state][level][gear + 8]);
				break;
			}
			Q[state][level][gear + 8] += alpha * (reward + gamma * max - Q[state][level][gear + 8]);
			
			state = next_state;
			s += delta_s;
			gear = new_gear;
		} while (s < mGradients[49].end);
	}

	for (i = 0; i < 50; i++) {
		fprintf(fout, "%d\n", i + 1);
		for (j = 0; j < 17; j++) {
			for (k = 0; k < 12; k++) {
				fprintf(fout, "%f ", Q[i][k][j]);
			}
			fprintf(fout, "\n");
		}
		fprintf(fout, "\n");
	}

	s = mGradients[0].start;
	gear = 0;
	velocity = 0;
	state = 0;

	printf("learn complete\n");

	while (s < mGradients[49].end) {
		int level = velocityToLevel(velocity);
		if (gear == 8) {
			if (Q[state][level][15] > Q[state][level][16]) {
				gear = 7;
			} else {
				gear = 8;
			}
		} else if (gear == -8) {
			if (Q[state][level][0] > Q[state][level][1]) {
				gear = -8;
			} else {
				gear = -7;
			}
		} else {
			float max = Q[state][level][gear + 9];
			new_gear = gear + 1;
			for (i = gear + 7; i < gear + 9; i++) {
				if (Q[state][level][i] > max) {
					new_gear = i - 8;
					max = Q[state][level][i];
				}
			}
			gear = new_gear;
		}
		float delta_s;
		float delta_e;
		float delta_v;
		int next_state = state;
		DoCaculateByTime(s, velocity, gear, tStep, &count, &delta_s, &delta_v, &delta_e);
		while (s + delta_s > mGradients[next_state].end) {
			next_state++;
		}
		velocity += delta_v;
		if ((int) (velocity + 0.5) < 0) {
			fprintf(result, "fail!\n");
			break;
		}
		s += delta_s;
		fprintf(result, "%f\t%f\t%d\t0.5\n", s, velocity, gear);
	}

	dispose();
	fclose(fout);
	fclose(result);

	return 0;
}

int velocityToLevel(float velocity) {
	int v = (int) (velocity + 0.5);
	if (v / 10 < 12) {
		return v / 10;
	}
	return 11;
}
