#include <stdio.h>

int myFunction(int, int);

int main(void) {
    int result = myFunction(2, 3);
    printf("Resultaat: %d\n", result);
    return 0;
}

int myFunction(int x, int y) {
    return x + y;
}