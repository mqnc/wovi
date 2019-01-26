
#include <iostream>
#include <iomanip>
#include <ctime>
#include <stack>

std::stack<clock_t> tictoc_stack;

void tic() {
	tictoc_stack.push(clock());
}

void toc() {
    std::cout << std::endl << "Time elapsed: "  <<
                 ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC << "s = " <<
                 ((double)(clock() - tictoc_stack.top())) * 1000.0 / CLOCKS_PER_SEC << "ms" <<std::endl;
    tictoc_stack.pop();
}
