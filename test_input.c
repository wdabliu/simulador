#include <stdio.h>
#include <conio.h>
#include <windows.h>

int main() {
    printf("Press keys to test input. Press 'q' to exit.\n");
    while(1) {
        if (_kbhit()) {
            int c = getch();
            printf("Key pressed: %c (%d)\n", c, c);
            if (c == 'q') break;
        }
        Sleep(10);
    }
    return 0;
}
