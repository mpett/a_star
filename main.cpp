//
//  main.cpp
//  pathfinder
//
//  Created by Martin Pettersson on 16/02/17.
//  Copyright © 2017 Martin Pettersson. All rights reserved.
//

#include <iostream>
#include <cmath>
#include <sstream>
#include <stdlib.h>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <functional>
#include <queue>
#include <vector>
#include <map>
#include <algorithm>

using namespace std;

struct Node {
    int xPosition;
    int yPosition;
    int fScore;
    int gScore;
    
    Node(int xCoordinate, int yCoordinate) {
        this -> xPosition = xCoordinate;
        this -> yPosition = yCoordinate;
        this -> fScore = 0;
        this -> gScore = 0;
    }
    
    bool operator<(const Node& comparableNode) const {
        return this->fScore > comparableNode.fScore;
    }
    
    bool operator==(const Node& anotherNode) const {
        if (this -> xPosition == anotherNode.xPosition
            && this -> yPosition == anotherNode.yPosition) {
            return true;
        } else {
            return false;
        }
    }
    
    int inputIndex(const int width) const {
        return yPosition * width + xPosition;
    }
    
    bool isEqual(const Node & anotherNode) const {
        if (this -> xPosition == anotherNode.xPosition
            && this -> yPosition == anotherNode.yPosition
            && this -> fScore == anotherNode.fScore
            && this -> gScore == anotherNode.gScore) {
            return true;
        } else {
            return false;
        }
    }
    
    bool hasEqualCoordinates(const Node& anotherNode) const {
        if (this -> xPosition == anotherNode.xPosition
            && this -> yPosition == anotherNode.yPosition) {
            return true;
        } else {
            return false;
        }
    }
    
    int heuristicDistanceFunction(const Node& destinationNode) const {
        int xDistance = destinationNode.xPosition - this -> xPosition;
        int yDistance = destinationNode.yPosition - this -> yPosition;
        int manhattanDistance = std::abs(xDistance) + std::abs(yDistance);
        return manhattanDistance;
    }
};

template <typename T>
bool vectorContains(const vector<T>& inputVector, const T& candidateElement) {
    if (find(inputVector.begin(), inputVector.end(), candidateElement) != inputVector.end()) {
        return true;
    } else {
        return false;
    }
}

void outputMapAndPath(const vector<int>& totalPath, const unsigned char pMap[],
                      const int width, const int height) {
    for (int yPosition = height - 1; yPosition >= 0; yPosition--) {
        for (int xPosition = 0; xPosition < width; xPosition++) {
            int currentIndex = yPosition * width + xPosition;
            
            if (vectorContains(totalPath, currentIndex)) {
                cout << " x ";
                continue;
            }
            
            bool nodeIsPassable = (int) pMap[currentIndex];
            
            if (nodeIsPassable) {
                cout << " - ";
            } else {
                cout << " # ";
            }
        }
        cout << endl << endl;
    }
}

bool queueContains(priority_queue<Node> nodeQueue, const Node& possibleNode) {
    while (!nodeQueue.empty()) {
        auto topNode = nodeQueue.top();
        if (topNode.isEqual(possibleNode)) {
            return true;
        }
        nodeQueue.pop();
    }
    return false;
}

vector<int> reconstructPath(unordered_map<int, int>& cameFrom,
                            const int goalNodeIndex, const int startNodeIndex) {
    vector<int> totalPath;
    int nodeIndex = goalNodeIndex;
    totalPath.push_back(goalNodeIndex);
    while (true) {
        nodeIndex = cameFrom[nodeIndex];
        totalPath.push_back(nodeIndex);
        if (nodeIndex == startNodeIndex) {
            return totalPath;
        }
    }
    return totalPath;
}

vector<Node> getNeighbors(const Node& currentNode, const unsigned char * pMap,
                          const unordered_map<int,Node>& inputMap,
                          const int width, const int bufferSize) {
    vector<Node> neighbors;
    int mapSize = bufferSize;
    bool isNextToLeftWall = currentNode.xPosition == 0;
    bool isNextToRightWall = currentNode.xPosition == width - 1;
    int currentNodeIndex = currentNode.xPosition + currentNode.yPosition * width;
    
    if ((currentNodeIndex + 1) >= 0 && (currentNodeIndex + 1) < mapSize
        && !isNextToRightWall && static_cast<int>(pMap[currentNodeIndex + 1] != 0))
        neighbors.push_back(inputMap.at(currentNodeIndex + 1));
    if ((currentNodeIndex - 1) >= 0 && (currentNodeIndex - 1) < mapSize
        && !isNextToLeftWall && static_cast<int>(pMap[currentNodeIndex - 1] != 0))
        neighbors.push_back(inputMap.at(currentNodeIndex - 1));
    if ((currentNodeIndex + width) >= 0 && (currentNodeIndex + width) < mapSize
        && static_cast<int>(pMap[currentNodeIndex + width] != 0))
        neighbors.push_back(inputMap.at(currentNodeIndex + width));
    if ((currentNodeIndex - width) >= 0 && (currentNodeIndex - width) < mapSize
        && static_cast<int>(pMap[currentNodeIndex - width] != 0))
        neighbors.push_back(inputMap.at(currentNodeIndex - width));
    
    return neighbors;
}

int FindPath(const int nStartX, const int nStartY,
             const int nTargetX, const int nTargetY,
             const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
             int* pOutBuffer, const int nOutBufferSize) {
    
    unordered_map<int, Node> inputMap;
    
    for (int yCoordinate = 0; yCoordinate < nMapHeight; yCoordinate++) {
        for (int xCoordinate = 0; xCoordinate < nMapWidth; xCoordinate++) {
            Node nodeToBeInserted(xCoordinate, yCoordinate);
            int inputKey = yCoordinate * nMapWidth + xCoordinate;
            inputMap.insert(pair<int, Node>(inputKey, nodeToBeInserted));
        }
    }
    
    vector<Node> closedVector;
    priority_queue<Node> openSet;
    unordered_map<int, int> cameFrom;
    
    Node startNode(nStartX, nStartY);
    startNode.gScore = 0;
    
    Node goalNode(nTargetX, nTargetY);
    startNode.fScore = startNode.heuristicDistanceFunction(goalNode);
    
    openSet.push(startNode);
    
    while (!openSet.empty()) {
        auto currentNode = openSet.top();
        
        if (currentNode.hasEqualCoordinates(goalNode)) {
            int currentNodeIndex = currentNode.yPosition * nMapWidth + currentNode.xPosition;
            int startNodeIndex = nStartY * nMapWidth + nStartX;
            auto totalPath = reconstructPath(cameFrom, currentNodeIndex, startNodeIndex);
            outputMapAndPath(totalPath, pMap, nMapWidth, nMapHeight);
            return static_cast<int>(totalPath.size());
        }
        
        openSet.pop();
        closedVector.push_back(currentNode);
        auto currentNeighbors =
            getNeighbors(currentNode, pMap, inputMap, nMapWidth, nOutBufferSize);
        
        for (auto& neighbor : currentNeighbors) {
            if (vectorContains(closedVector, neighbor))
                continue;
            
            int tentativeGScore = currentNode.gScore + 1; // dist between?
            bool shouldPushToOpenSet = false;
            
            if (!queueContains(openSet, neighbor))
                shouldPushToOpenSet = true;
            else if (tentativeGScore >= neighbor.gScore)
                continue;
            
            neighbor.gScore = tentativeGScore;
            neighbor.fScore = neighbor.gScore + neighbor.heuristicDistanceFunction(goalNode);
            int keyNodeIndex = neighbor.yPosition * nMapWidth + neighbor.xPosition;
            int valueNodeIndex = currentNode.yPosition * nMapWidth + currentNode.xPosition;
            cameFrom[keyNodeIndex] = valueNodeIndex;
            
            if (shouldPushToOpenSet)
                openSet.push(neighbor);
        }
    }
    
    return -1;
}

int main(int argc, const char * argv[]) {
    unsigned char pMap[] = {1, 1, 0, 1, 1, 1, 1, 1,
        0, 1, 0, 0, 0, 0, 1, 1,
        0, 1, 1, 0, 0, 1, 1, 1,
        0, 1, 0, 0, 0, 1, 0, 0,
        0, 1, 1, 1, 1, 1, 1, 1,
        0, 1, 0, 0, 1, 1, 0, 1,
        1, 1, 1, 0, 1, 0, 0, 1,
        1, 1, 1, 1, 1, 1, 1, 1};
    int pOutBuffer[64];
    return FindPath(0, 0, 3, 0, pMap, 8, 8, pOutBuffer, 64);
}
