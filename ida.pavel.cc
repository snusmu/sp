// 2016 Pavel K. 
// Iterative deepening a-star search
// note: this is second program i wrote in c++ from scratch
// i am currently working on recursive best first search, which visits less nodes
// having visibility graph could be better but would take me more time to implement
#include <stdlib.h>
#include <iostream>
using namespace std;

int heuristic(const int from, const int to, const int mapWidth) {
  // TODO: improve, cost always increases in +2/-2
  int fromX = from % mapWidth;
  int fromY = from / mapWidth;
  int toX = to % mapWidth;
  int toY = to / mapWidth;
  return abs(toY - fromY) + abs(toX - fromX);
}

int ida(int** matrix, const int node, const int end, int currentCost,
        const int max, int* visited, const int mapWidth) {
  int cost = currentCost + heuristic(node, end, mapWidth);
  if (cost > max) return cost; 
  if (end == visited[currentCost - 1]) return currentCost;
  int min = -1;
  for (int i = 0; i < 4; i++) {
    if (matrix[node][i] == -1) continue;
    bool been = false;
    for (int j = 0; j < currentCost; j++) 
      if (matrix[node][i] == visited[j]) {
        been = true;
        break;
      }
    if (been) continue;
    visited[currentCost] = matrix[node][i];
    int pos = (ida(matrix, matrix[node][i], end, currentCost + 1, max, visited, mapWidth));
    if (pos == -1) continue;
    if (visited[pos - 1] == end) return pos;
    if ((min == -1) || (pos < min)) min = pos;
  }
  return min;
}

int FindPath(const int nStartX, const int nStartY, const int nTargetX,
             const int nTargetY, const unsigned char* pMap, const int nMapWidth,
             const int nMapHeight, int* pOutBuffer, const int nOutBufferSize) {
  int mapFieldCount = nMapWidth * nMapHeight;
  int** matrix = new int*[mapFieldCount];
  for (int i = 0; i < mapFieldCount; i++) {
    // constructing graph for iterative deepening depth-first search
    // unavailable locations marked with -1
    //    top -> 0
    //  left -> 3i1 <- right
    // bottom -> 2
    matrix[i] = new int[4];
    for (int j = 0; j < 4; j++) matrix[i][j] = -1;  // -1 is unavailable
    if (pMap[i] != 1) continue;
    if ((i >= nMapWidth) && (pMap[i - nMapWidth] == 1)) matrix[i][0] = i - nMapWidth;
    if (((i + 1) % nMapWidth != 0) && (pMap[i + 1] == 1)) matrix[i][1] = i + 1;
    if ((i < mapFieldCount - nMapWidth) && (pMap[i + nMapWidth] == 1)) matrix[i][2] = i + nMapWidth;
    if ((i % nMapWidth != 0) && (pMap[i - 1] == 1)) matrix[i][3] = i - 1;
  }
  int start = nStartY * nMapWidth + nStartX;
  int end = nTargetY * nMapWidth + nTargetX;
  int max = -1;
  while (true) {
    max = ida(matrix, start, end, 0, max, pOutBuffer, nMapWidth);
    if (max <= 0) break;
    if (max > nOutBufferSize){
      max = -1;
      break;
    }
    if (pOutBuffer[max - 1] == end) break;
  }
  delete matrix;
  return max;
}
