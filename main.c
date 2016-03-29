#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "tractionCaculator.h"

#define tStep 1
#define gamma 0.8
#define alpha 1

int velocityToLevel(float velocity);

int main() {
	float Q[11][5][3] = {0};
	int i, j, times, k;
	int count = 0;

	srand(time(NULL));

	LOCOPARAMETER* locoInfoPtr = initLocoInfo();
	OPTCONSTPARAM* optConstPtr = initOptConst();
	initModel(locoInfoPtr, optConstPtr);

	for (times = 0; times < 10000000; times++) {
		int state = 0;
		float s = mGradients[0].start;
		int gear = 0;
		float velocity = 0;
		do {
			int action;
			int level = velocityToLevel(velocity);
			float p = (float) rand() / RAND_MAX;
			if (p > 0.1) {
			 	if (gear == 8) {
					if (Q[state][level][0] > Q[state][level][1]) {
						action = 0;
					} else {
						action = 1;
					}
				} else if (gear == -8) {
					if (Q[state][level][1] > Q[state][level][2]) {
						action = 1;
					} else {
						action = 2;
					}
				} else {
					float max = Q[state][level][2];
					action = 2;
					for (i = 0; i < 2; i++) {
						if (Q[state][level][i] > max) {
							action = i;
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
			int next_state = state;

			gear += action - 1;
			DoCaculateByTime(s, velocity, gear, tStep, &count, &delta_s, &delta_v, &delta_e);
			if (s + delta_s > mGradients[state].end) {
				next_state++;
			}
			velocity += delta_v;
			int next_level = velocityToLevel(velocity);
			float max = Q[next_state][next_level][0];
			for (i = 1; i < 3; i++) {
				if (Q[next_state][next_level][i] > max) {
					max = Q[next_state][next_level][i];
				}
			}
			Q[state][level][action] = -delta_e + gamma * max;
			if ((int) (velocity + 0.5) < 0) {
				Q[state][level][action] -= 2;
				break;
			}

			state = next_state;
			s += delta_s;
		} while (s < mGradients[9].end);
	}

	for (i = 0; i < 10; i++) {
		for (j = 0; j < 3; j++) {
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
