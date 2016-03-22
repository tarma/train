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

	for (times = 0; times < 10000; times++) {
		 int state = 0;
		 float s = mGradients[0].start;
		 int gear = 0;
		 float velocity = 0;
		 do {
		 	int action;
		 	if (gear == 8) {
				action = rand() % 2;
			} else if (gear == -8) {
				action = rand() % 2 + 1;
			} else {
				action = rand() % 3;
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
			float max = Q[next_state][0];
			for (i = 1; i < 3; i++) {
				if (Q[next_state][i] > max) {
					max = Q[next_state][i];
				}
			}

			Q[state][action] = -delta_e + gamma * max;
			state = next_state;
			s += delta_s;
			velocity += delta_v;
		 } while (s < 81868 && s >= 79793);
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
