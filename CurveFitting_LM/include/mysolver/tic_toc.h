//
// Created by alex on 2020/12/11.
//

#ifndef CURVEFITTING_LM_TIC_TOC_H
#define CURVEFITTING_LM_TIC_TOC_H



#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc {
public:
    TicToc() {
        tic();
    }

    void tic() {
        start = std::chrono::system_clock::now();
    }

    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};


#endif //CURVEFITTING_LM_TIC_TOC_H
