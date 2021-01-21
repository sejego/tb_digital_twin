//
// Created by sejego on 10.01.21.
//

#include "PhysicalValueTemplate.h"

PhysicalValueTemplate::PhysicalValueTemplate(int numberOfCols, int numberOfRows,
                                             int numberOfValues) :  numOfVals(numberOfValues),
                                                                    numOfCols(numberOfCols),
                                                                    numOfRows(numberOfRows)
{
    std::cout << "Instantiation happened\n";
}
void PhysicalValueTemplate::processDataToVectors(int numberOfValues, std::vector<std::vector<float>> dataToProcess)
{
    int colCt = 0;

    for(auto row : dataToProcess)
    {
        for(int i = 0; i < numOfCols-1; i+2)
        {
            timesVector[colCt].push_back(row[i]);
            valuesVector[colCt].push_back(row[i+1]);
        }
        colCt++;
    }
}
void PhysicalValueTemplate::setStartTime(float newStartTime)
{
    startTime = newStartTime;
}
void PhysicalValueTemplate::setTimeStep(float newTimeStep)
{
    timeStep = newTimeStep;
}