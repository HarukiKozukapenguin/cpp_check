#include <iostream>
void check(int a, int b);

int main(void){
    int c = 1;
    int d = 2;
    check(c,d);
    check(b = c, a = d);
    return 0;
}

void check(int a, int b){
    std::cout << a << b << std::endl;
}