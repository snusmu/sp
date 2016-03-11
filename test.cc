// 2016 Pavel K. 
// note: this is second program i wrote in c++ from scratch
// with dense grids processing takes way too long...
// visibility graph + dijkstra should solve that but would take more time to implement
#include <iostream>
#include <ctime>
#include <stdlib.h> 
#include "ida.pavel.cc"
using namespace std;

void printResults(const int distance, int* pOutBuffer) {
  cout << "distance: " << distance << "\n"; 
  for(int i = 0; i < distance; i++)
    cout << "->" << pOutBuffer[i];
  cout << "\n";
  return;
}

using namespace std;
int main(){
  cout << "EXAMPLE 1\n";
  unsigned char pMap[] = {
    1, 1, 1, 1, 
    0, 0, 0, 0, 
    1, 1, 1, 1
  };
  int pOutBuffer[30];
  int distance = FindPath(0, 0, 3, 2, pMap, 4, 3, pOutBuffer, 12);
  printResults(distance, pOutBuffer);
  cout << "EXAMPLE 2\n";
  unsigned char pMap2[] = {
    0, 0, 1, 
    0, 1, 1, 
    1, 0, 1
  };
  distance = FindPath(2, 0, 0, 2, pMap2, 3, 3, pOutBuffer, 7);
  printResults(distance, pOutBuffer);
  cout << "EXAMPLE 3\n";
  unsigned char pMap3[] = {
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 
    1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 
    1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 
    0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1,
    0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
    0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
    0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
    0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
  };
  distance = FindPath(0, 0, 0, 15, pMap3, 16, 16, pOutBuffer, 100);
//  distance = FindPath(0, 0, 0, 10, pMap3, 16, 11, pOutBuffer, 50);
  printResults(distance, pOutBuffer);
  cout << "EXAMPLE 4\n";
  unsigned char pMap4[] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 
  };
  distance = FindPath(12, 14, 2, 2, pMap4, 16, 16, pOutBuffer, 100);
//  distance = FindPath(0, 0, 0, 10, pMap3, 16, 11, pOutBuffer, 50);
  printResults(distance, pOutBuffer);
  cout << "EXAMPLE 5\n";
  unsigned char pMap5[] = {
    1, 1
  };
  distance = FindPath(0, 0, 0, 0, pMap5, 1, 2, pOutBuffer, 100);
  printResults(distance, pOutBuffer);
  return 0;
}

