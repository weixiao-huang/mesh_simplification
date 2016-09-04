#include <iostream>
#include "Simplifier.h"

using std::string;

#define INF_DOUBLE 1e10

int main(int argc, char** argv) {
    std::string inputFile(argv[1]);
    std::string outputFile(argv[2]);
    double simplifyRate = atof(argv[3]);
    double threshold;
    if (argc == 5) {
        threshold = atof(argv[4]);
    } else {
        printf("Note: use default threshold = Inf\n");
        threshold = INF_DOUBLE;
    }

    Simplifier simplifier;
    simplifier.load_obj(inputFile);
    simplifier.simplify(simplifyRate, threshold);
    simplifier.output(outputFile);
    return 0;
}