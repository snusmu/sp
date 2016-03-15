// 2016 Pavel Krasnovskij
// Base visual graph and a-star search
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

class Node {
  int nX, nY;

public:
  bool bClosed, bOpen;
  int nToStart, nToTarget, nHeuristic;
  Node *pPrevious;
  vector<Node *> rgNeighbors;
  vector<int> rgEdges;
  Node(const int nIndex, const int nMapWidth);
  int GetDistance(Node *pTarget, const unsigned char *pMap,
                  const int nMapWidth);
  int IdaStar(Node *pTarget, int nCost, const int nBound);
  void SetHeuristic(Node *pTarget);
  void FillPathTo(Node *pTarget, int *pOutBuffer, const int nMapWidth);
  void PrintPath();
};

bool SortByDistanceToTarget(Node *pNode1, Node *pNode2) {
  return ((pNode2->nToTarget != -1) && (pNode1->nToTarget > pNode2->nToTarget));
}

Node::Node(const int x, const int y) {
  pPrevious = NULL;
  bClosed = false;
  bOpen = false;
  nToTarget = -1;
  nToStart = -1;
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
    if (nX == pTarget->nX)
      continue;
    if (pMap[nMapWidth * i + pTarget->nX] == 0)
      return -1;
  }
  for (int i = nX; i != pTarget->nX; i = i + nModX) {
    if (pMap[nY * nMapWidth + i] == 0)
      return -1;
    if (nY == pTarget->nY)
      continue;
    if (pMap[pTarget->nY * nMapWidth + i] == 0)
      return -1;
  }
  return abs(nY - pTarget->nY) + abs(nX - pTarget->nX);
}

void Node::FillPathTo(Node *pTarget, int *pOutBuffer, const int nMapWidth) {
  if (this == pTarget) {
    return;
  }
  int nBound = nToStart;
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
  pPrevious->FillPathTo(pTarget, pOutBuffer, nMapWidth);
  return;
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
  // visual graph for a*
  for (int i = 0; i < rgKeypoints.size(); i++) {
    Node *pFrom = rgKeypoints[i];
    pFrom->SetHeuristic(pTarget);
    for (int j = 0; j < rgKeypoints.size(); j++) {
      if (j == i)
        continue;
      Node *pTo = rgKeypoints[j];
      int nDistance = pFrom->GetDistance(pTo, pMap, nMapWidth);
      if (nDistance > 0) {
        pFrom->rgNeighbors.push_back(pTo);
        pFrom->rgEdges.push_back(nDistance);
      }
    }
  }
  // a*
  vector<Node *> rgOpen;
  pStart->nToStart = 0;
  pStart->nToTarget = pStart->nHeuristic;
  pStart->bOpen = true;
  rgOpen.push_back(pStart);
  while (rgOpen.size() > 0) {
    sort(rgOpen.begin(), rgOpen.end(), SortByDistanceToTarget);
    Node *pCurrent = rgOpen.back();
    if (pCurrent == pTarget) {
      pTarget->FillPathTo(pStart, pOutBuffer, nMapWidth);
      return pTarget->nToStart;
    }
    if (pCurrent->nToTarget > nOutBufferSize) {
      break;
    }
    rgOpen.pop_back();
    pCurrent->bOpen = false;
    pCurrent->bClosed = true;
    for (int i = 0; i < pCurrent->rgNeighbors.size(); i++) {
      Node *pNeighbor = pCurrent->rgNeighbors[i];
      if (pNeighbor->bClosed)
        continue;
      int nScore = pCurrent->nToStart + pCurrent->rgEdges[i];
      if (!pNeighbor->bOpen) {
        pNeighbor->bOpen = true;
        rgOpen.push_back(pNeighbor);
      } else if ((pNeighbor->nToStart == -1) ||
                 (nScore >= pNeighbor->nToStart)) {
        continue;
      }
      pNeighbor->pPrevious = pCurrent;
      pNeighbor->nToStart = nScore;
      pNeighbor->nToTarget = nScore + pNeighbor->nHeuristic;
    }
  }
  return -1;
}
