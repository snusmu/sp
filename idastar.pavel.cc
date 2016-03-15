// 2016 Pavel K.
// Base (slightly naive) visual graph and iterative deepening a-star search
// i hope its not too ugly. this is second program i wrote in c++ from scratch
// before that, i was only fixing others' code
#include <stdlib.h>
#include <iostream>
#include <vector>
using namespace std;

int counter = 0;

class Node {
  int nX, nY, nHeuristic;

public:
  int nLowestCost;
  Node* pLowestOrigin;
  Node *pPrevious;
  vector<Node *> rgNeighbors;
  vector<int> rgEdges;
  Node(const int nIndex, const int nMapWidth);
  int GetDistance(Node *pTarget, const unsigned char *pMap,
                  const int nMapWidth);
  int IdaStar(Node *pTarget, int nCost, const int nBound);
  void SetHeuristic(Node *pTarget);
  void FillPathTo(Node *pTarget, int nBound, int *pOutBuffer,
                  const int nMapWidth);
  void PrintPath();
};

Node::Node(const int x, const int y) {
  pPrevious = NULL;
  pLowestOrigin = NULL;
  nX = x;
  nY = y;
}

void Node::SetHeuristic(Node *pTarget) {
  nHeuristic = abs(pTarget->nY - nY) + abs(pTarget->nX - nX);
}

int Node::GetDistance(Node *pTarget, const unsigned char *pMap,
                      const int nMapWidth) {
  int nModY;
  if (nY > pTarget->nY) {
    nModY = -1;
  } else {
    nModY = 1;
  }
  int nModX;
  if (nX > pTarget->nX) {
    nModX = -1;
  } else {
    nModX = 1;
  }
  // return -1 if single 0 is found on the simple path
  // that always means there's keypoint somewhere in the way.
  for (int i = nY; i != pTarget->nY; i = i + nModY) {
    if (pMap[nMapWidth * i + nX] == 0)
      return -1;
  }
  for (int i = nX; i != pTarget->nX; i = i + nModX) {
    if (pMap[pTarget->nY * nMapWidth + i] == 0)
      return -1;
  }
  return abs(nY - pTarget->nY) + abs(nX - pTarget->nX);
}

void Node::FillPathTo(Node *pTarget, int nBound, int *pOutBuffer,
                      const int nMapWidth) {
  if (this == pTarget) {
    return;
  }
  int nModY;
  if (nY > pPrevious->nY) {
    nModY = -1;
  } else {
    nModY = 1;
  }
  int nModX;
  if (nX > pPrevious->nX) {
    nModX = -1;
  } else {
    nModX = 1;
  }
  // has to be same as GetDistance check
  for (int i = nX; i != pPrevious->nX; i = i + nModX) {
    nBound--;
    pOutBuffer[nBound] = nY * nMapWidth + i;
  }
  for (int i = nY; i != pPrevious->nY; i = i + nModY) {
    nBound--;
    pOutBuffer[nBound] = i * nMapWidth + pPrevious->nX;
  }
  pPrevious->FillPathTo(pTarget, nBound, pOutBuffer, nMapWidth);
  return;
}

void Node::PrintPath() {
  cout << nX << "," << nY << "->";
  if (pPrevious == NULL) {
    cout << "\n";
    return;
  }
  pPrevious->PrintPath();
  return;
}

// note: recursive best first search visits less nodes?
int Node::IdaStar(Node *pTarget, const int nCost, const int nBound) {
  counter++;
  if (pTarget == this){
    return nCost;
  }
  int nEstimate = nCost + nHeuristic;
  if (nEstimate > nBound)
    return nEstimate;
  int nMin = -1;
  for (int i = 0; i < rgNeighbors.size(); i++) {
    Node *pNeighbor = rgNeighbors[i];
    bool bVisited = false;
    for (Node *prevNode = this; prevNode->pPrevious != NULL;
         prevNode = prevNode->pPrevious) {
      if (prevNode->pPrevious == pNeighbor) {
        bVisited = true;
        break;
      }
    }
    if (bVisited)
      continue;
    pNeighbor->pPrevious = this;
    int nNewCost = pNeighbor->IdaStar(pTarget, nCost + rgEdges[i], nBound);
    if (pTarget->pPrevious != NULL)
      return nNewCost; // got there!
    if (nNewCost == -1)
      continue;
    if ((nMin == -1) || (nNewCost < nMin))
      nMin = nNewCost;
  }
  return nMin;
}

bool isCorner(const int nX, const int nY, const int nCornerX,
              const int nCornerY, const unsigned char *pMap,
              const int nMapWidth, const int nMapHeight) {
  if (nCornerX < 0 || nCornerY < 0 || nCornerX > nMapWidth - 1 ||
      nCornerY > nMapHeight - 1)
    return false;
  return (pMap[nY * nMapWidth + nCornerX] == 1 &&
          pMap[nCornerY * nMapWidth + nCornerX] == 0 &&
          pMap[nCornerY * nMapWidth + nX] == 1);
}

int FindPath(const int nStartX, const int nStartY, const int nTargetX,
             const int nTargetY, const unsigned char *pMap, const int nMapWidth,
             const int nMapHeight, int *pOutBuffer, const int nOutBufferSize) {
  if (nStartX == nTargetX && nStartY == nTargetY)
    return 0;
  // now collecting rgKeypoints for visibility graph
  // done by finding corners. could be optimized..
  vector<Node *> rgKeypoints;
  Node *pStart = new Node(nStartX, nStartY);
  Node *pTarget = new Node(nTargetX, nTargetY);
  rgKeypoints.push_back(pStart);
  rgKeypoints.push_back(pTarget);
  for (int y = 0; y < nMapHeight; y++) {
    for (int x = 0; x < nMapWidth; x++) {
      if ((x == nStartX && y == nStartY) || (x == nTargetX && y == nTargetY)) {
        continue; // already added
      }
      int i = y * nMapWidth + x;
      if (pMap[i] == 0)
        continue; // skip unavailable
      if (isCorner(x, y, x - 1, y - 1, pMap, nMapWidth, nMapHeight) ||
          isCorner(x, y, x + 1, y - 1, pMap, nMapWidth, nMapHeight) ||
          isCorner(x, y, x + 1, y + 1, pMap, nMapWidth, nMapHeight) ||
          isCorner(x, y, x - 1, y + 1, pMap, nMapWidth, nMapHeight)) {
        rgKeypoints.push_back(new Node(x, y));
      }
    }
  }
  // constructing graph for ida*
  for (int i = 0; i < rgKeypoints.size(); i++) {
    Node *pFrom = rgKeypoints[i];
    pFrom->SetHeuristic(pTarget);
    for (int j = 0; j < rgKeypoints.size(); j++) {
      if (j == i)
        continue; // could optimize to reduce twice
      Node *pTo = rgKeypoints[j];
      int nDistance = pFrom->GetDistance(pTo, pMap, nMapWidth);
      if (nDistance > 0) {
        pFrom->rgNeighbors.push_back(pTo);
        pFrom->rgEdges.push_back(nDistance);
      }
    }
  }
  int nBound = -1;
  while (true) {
    nBound = pStart->IdaStar(pTarget, 0, nBound);
    if (nBound <= 0)
      break;
    if (nBound > nOutBufferSize) {
      nBound = -1;
      break;
    }
    cout << nBound << "\n";
    cout << "cnt" << counter << "\n";
    counter = 0;
  for (int i = 0; i < rgKeypoints.size(); i++) {
    rgKeypoints[i]->pLowestOrigin = NULL;
  }
    if (pTarget->pPrevious != NULL) {
      pTarget->FillPathTo(pStart, nBound, pOutBuffer, nMapWidth);
      break;
    }
  }
  return nBound;
}
