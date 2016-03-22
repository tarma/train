#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "tractionCaculator.h"

#define tStep 0.5
#define gamma 0.8
#define alpha 1

int main() {
	float Q[11][3] = {0};
	int i, j, times;
	int count = 0;

	srand(time(NULL));

	LOCOPARAMETER* locoInfoPtr = initLocoInfo();
	OPTCONSTPARAM* optConstPtr = initOptConst();
	initModel(locoInfoPtr, optConstPtr);

	for (times = 0; times < 10; times++) {
		 int state = 0;
		 float s = mGradients[0].start;
		 int gear = 0;
		 float velocity = 0;
		 do {
		 	int action;
			float p = (float) rand() / RAND_MAX;
			if (p > 0.1) {
			 	if (gear == 8) {
					if (Q[state][0] < Q[state][1]) {
						action = 1;
					} else {
						action = 0;
					}
				} else if (gear == -8) {
					if (Q[state][1] < Q[state][2]) {
						action = 2;
					} else {
						action = 1;
					}
				} else {
					float max = Q[state][0];
					action = 0;
					for (i = 1; i < 2; i++) {
						if (Q[state][i] > max) {
							action = i;
							max = Q[state][i];
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

			printf("%d %d %f\n", gear, action, p);

			gear += action - 1;
			DoCaculateByTime(s, velocity, gear, tStep, &count, &delta_s, &delta_v, &delta_e);
			if (s + delta_s > mGradients[state].end) {
				next_state++;
			}
			float max = Q[next_state][0];
			for (i = 1; i < 3; i++) {
				if (Q[next_state][i] > max) {
					max = Q[next_state][i];
				}
			}
			Q[state][action] = -delta_e + gamma * max;
			if (delta_s < 0) {
				Q[state][action] -= 100;
				break;
			}
			state = next_state;
			s += delta_s;
			velocity += delta_v;
		 } while (s < 81868);
	}

	for (i = 0; i < 10; i++) {
		for (j = 0; j < 3; j++) {
			printf("%f ", Q[i][j]);
		}
		printf("\n");
	}

	dispose();

	return 0;
}
