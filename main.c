#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "tractionCaculator.h"

int main() {
	/*int R[6][6] = {{-1, -1, -1, -1, 0, -1}, {-1, -1, -1, 0, -1, 100}, {-1, -1, -1, 0, -1, -1}, {-1, 0, 0, -1, 0, -1}, {0, -1, -1, 0, -1, 100}, {-1, 0, -1, -1, 0, 100}};
	int Q[6][6] = {0};
	int i, j, times;
	int change;
	int temp[6] = {0};
	int temp_len;

	srand(time(NULL));

	for (times = 0; times < 1000; times++) {
		 int state = rand() % 6;
		 do {	
			temp_len = 0;
			double p = (double) rand() / RAND_MAX;
			int maxn;
			int max = -1;
			for (i = 0; i < 6; i++) {
				if (R[state][i] >= 0) {
					temp[temp_len] = i;
					temp_len++;
				}
				if (Q[state][i] > max) {
					maxn = i;
					max = Q[state][i];
				}
			}

			maxn = temp[rand() % temp_len];
			max = -1;
			for (i = 0; i < 6; i++) {
				if (R[maxn][i] >= 0 && Q[maxn][i] > max) {
					max = Q[maxn][i];
				}
			}
			Q[state][maxn] = R[state][maxn] + 0.8 * max;
			state = maxn;
		 } while (state != 5);
	}

	for (i = 0; i < 6; i++) {
		for (j = 0; j < 6; j++) {
			printf("%d ", Q[i][j]);
		}
		printf("\n");
	}
*/

	LOCOPARAMETER* locoInfoPtr = initLocoInfo();
	OPTCONSTPARAM* optConstPtr = initOptConst();
	initModel(locoInfoPtr, optConstPtr);

	return 0;
}
