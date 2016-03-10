// 2016 Pavel K. 
// note: this is second program i wrote in c++ from scratch
// with dense grids processing takes way too long...
// visibility graph + dijkstra should solve that but would take more time to implement
#include <stdlib.h> 
// Iterative deepening depth-first search
bool iddfs(int** matrix, const int current, const int end, int position, const int distance, int * visited ) {
  if(position==distance)
    return current == end;
  for(int i=0;i<4;i++){
    if(matrix[current][i]==-1)
      continue;
    bool been=false;
    for(int j=0;j<position;j++){
      if(matrix[current][i]==visited[j]){
        been=true;
        break;
      }
    }
    if(been)
      continue;
    visited[position]=matrix[current][i];
    if(iddfs(matrix,matrix[current][i],end,position+1,distance,visited))
      return true;
  }
  return false;
}

int FindPath(const int nStartX, const int nStartY,
    const int nTargetX, const int nTargetY, 
    const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
    int* pOutBuffer, const int nOutBufferSize){
  int mapFieldCount = nMapWidth * nMapHeight;
  int** matrix = new int*[mapFieldCount]; 
  for(int i=0;i<mapFieldCount;i++){
    // constructing graph for iterative deepening depth-first search
    // unavailable locations marked with -1
    //    top -> 0
    //  left -> 3i1 <- right
    // bottom -> 2
    matrix[i] = new int[4];
    if(pMap[i]!=1){
      for(int j=0;j<4;j++)
        matrix[i][j]=-1;
      continue; 
    }
    if((i>=nMapWidth)&&(pMap[i-nMapWidth]==1))
      matrix[i][0]=i-nMapWidth;
    else
      matrix[i][0]=-1;
    if(((i+1)%nMapWidth!=0)&&(pMap[i+1]==1))
      matrix[i][1]=i+1;
    else
      matrix[i][1]=-1;
    if((i<mapFieldCount-nMapWidth)&&(pMap[i+nMapWidth]==1))
      matrix[i][2]=i+nMapWidth;
    else
      matrix[i][2]=-1;
    if((i%nMapWidth!=0)&&(pMap[i-1]==1))
      matrix[i][3]=i-1;
    else
      matrix[i][3]=-1;
  }
  int start = nStartY*nMapWidth + nStartX ;
  int end = nTargetY*nMapWidth + nTargetX ;
  int distance = abs(nTargetY - nStartY) + abs(nTargetX - nStartX);
  while(distance < nOutBufferSize){
    if(iddfs(matrix, start, end, 0, distance, pOutBuffer)){
      delete matrix;
      return distance;
    }
    distance+=2;
  }
  delete matrix;
  return -1;
}
