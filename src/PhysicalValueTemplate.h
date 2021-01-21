//
// Created by sejego on 10.01.21.
//

#ifndef SRC_PHYSICALVALUETEMPLATE_H
#define SRC_PHYSICALVALUETEMPLATE_H

#include <iostream>
#include <vector>
#include <string>
#include <array>
#include <map>
#include <unordered_map>

class PhysicalValueTemplate
{
public:

    PhysicalValueTemplate(int numberOfCols, int numberOfRows, int numberOfValues);
    void setStartTime(float newStartTime);
    void setTimeStep(float newStartTime);

protected:

    float timeStep, startTime;
    const int numOfCols;
    const int numOfRows;
    const int numOfVals;
    std::vector<std::vector<float>> timesVector;
    std::vector<std::vector<float>> valuesVector;
    void processDataToVectors(int numberOfValues, std::vector<std::vector<float>> dataToProcess);

};


#endif //SRC_PHYSICALVALUETEMPLATE_H
