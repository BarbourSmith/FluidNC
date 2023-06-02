#include "calibration.h";

void setup() {
  Serial.begin(115200);
}

void loop() {

    int populationSize = 50;
    
    
    double measurements[][4] = {
        //TL         TR        BL        BR
        {748.604, 2472.067, 1426.493, 2764.56},
        {2372.21, 1149.968, 2436.423, 238.632},
        {2087.219, 1898.011, 1714.555, 1452.135},
        {2847.807, 1492.287, 2543.025, 658.01},
        {1832.44, 2724.18, 846.658, 2179.185},
        {2068.196, 1496.784, 2043.238, 1437.005}
    };
    computeCalibration(measurements);
}