//
// Created by sejego on 10/18/20.
//

#ifndef CATKIN_WS_PARSEDEWETRON_H
#define CATKIN_WS_PARSEDEWETRON_H

#include <iostream>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>

class parseDewetron
{
public:

    /* File parser */
    parseDewetron(std::string filename, int numberOfColumns);
    ~parseDewetron();

    /* Getters */

    std::vector<std::vector<float>> getProcessed2DVector();
    float getTimeStep() const;
    float getStartTime() const;
    int getNumberOfColumns();
    int getNumberOfRows();
    std::vector<std::string> getListOfValueNames();
    std::vector<std::vector<float>> getOnlyTimes();
    std::vector<std::vector<float>> getOnlyValues();

private:
    void separateToVectors();
    std::ifstream dewetronFile;
    float timeStep;
    float startTime;
    int numOfCols;
    int numOfRows;
    std::vector<std::vector<float>> parsedData;
    std::vector<std::string> listOfValueNames;
    std::vector<std::vector<float>> timeVector;
    std::vector<std::vector<float>> valuesVector;
    std::string line;
    void parseDewetronFile();
};

#endif //CATKIN_WS_PARSEDEWETRON_H
