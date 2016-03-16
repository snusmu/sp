// 2016 Pavel Krasnovskij
// Base visual graph and a-star search
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <queue>
using namespace std;

class Node {
  int nX, nY;

 public:
  bool bClosed, bOpen;
  int nToStart, nToTarget, nHeuristic;
  Node *pPrevious;
  vector<Node *> rgNeighbors;
  vector<int> rgEdges;
  Node(const int x, const int y);
  Node(const int x, const int y, Node *pTarget);
  int GetDistance(Node *pTarget, const unsigned char *pMap,
                  const int nMapWidth);
  void FillPathTo(Node *pTarget, int *pOutBuffer, const int nMapWidth);
  void init(const int x, const int y);
  void PrintPath();
};

void Node::PrintPath() {
  cout << nX << "," << nY << "->";
  if (pPrevious == NULL) {
    cout << "\n";
    return;
  }
  pPrevious->PrintPath();
  return;
}

struct ByDistanceToTarget {
  bool operator()(Node *pNode1, Node *pNode2) {
    return ((pNode2->nToTarget != -1) &&
            (pNode1->nToTarget > pNode2->nToTarget));
  }
};

void Node::init(const int x, const int y) {
  pPrevious = NULL;
  bClosed = false;
  bOpen = false;
  nToTarget = -1;
  nToStart = -1;
  nHeuristic = 0;
  nX = x;
  nY = y;
}

Node::Node(const int x, const int y) { init(x, y); }

Node::Node(const int x, const int y, Node *pTarget) {
  init(x, y);
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
    if (pMap[nMapWidth * i + nX] == 0) return -1;
    if (nX == pTarget->nX) continue;
    if (pMap[nMapWidth * i + pTarget->nX] == 0) return -1;
  }
  for (int i = nX; i != pTarget->nX; i = i + nModX) {
    if (pMap[nY * nMapWidth + i] == 0) return -1;
    if (nY == pTarget->nY) continue;
    if (pMap[pTarget->nY * nMapWidth + i] == 0) return -1;
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
  if (nStartX == nTargetX && nStartY == nTargetY) return 0;
  // now collecting rgKeypoints for visibility graph
  // done by finding corners. could be optimized..
  vector<Node *> rgKeypoints;
  Node *pTarget = new Node(nTargetX, nTargetY);
  Node *pStart = new Node(nStartX, nStartY, pTarget);
  rgKeypoints.push_back(pStart);
  rgKeypoints.push_back(pTarget);
  for (int y = 0; y < nMapHeight; y++) {
    for (int x = 0; x < nMapWidth; x++) {
      if ((x == nStartX && y == nStartY) || (x == nTargetX && y == nTargetY)) {
        continue;  // already added
      }
      int i = y * nMapWidth + x;
      if (pMap[i] == 0) continue;  // skip unavailable
      if (isCorner(x, y, x - 1, y - 1, pMap, nMapWidth, nMapHeight) ||
          isCorner(x, y, x + 1, y - 1, pMap, nMapWidth, nMapHeight) ||
          isCorner(x, y, x + 1, y + 1, pMap, nMapWidth, nMapHeight) ||
          isCorner(x, y, x - 1, y + 1, pMap, nMapWidth, nMapHeight)) {
        rgKeypoints.push_back(new Node(x, y, pTarget));
      }
    }
  }
  // ida*
  priority_queue<Node *, vector<Node *>, ByDistanceToTarget> rgOpen;
  pStart->nToStart = 0;
  pStart->nToTarget = pStart->nHeuristic;
  pStart->bOpen = true;
  rgOpen.push(pStart);
  int nToStart, nToCurrent, nToTarget;
  while (rgOpen.size() > 0) {
    Node *pCurrent = rgOpen.top();
    if (pCurrent == pTarget) {
      pTarget->FillPathTo(pStart, pOutBuffer, nMapWidth);
      return pTarget->nToStart;
    }
    rgOpen.pop();
    pCurrent->bOpen = false;
    pCurrent->bClosed = true;
    for (int i = 0; i < rgKeypoints.size(); i++) {
      Node *pNeighbor = rgKeypoints[i];
      if (pNeighbor->bClosed) continue;  // checked already
      nToCurrent = pCurrent->GetDistance(pNeighbor, pMap, nMapWidth);
      if (nToCurrent == -1) continue;  // not a neighbour
      nToStart = pCurrent->nToStart + nToCurrent;
      nToTarget = nToStart + pNeighbor->nHeuristic;
      if (nToTarget > nOutBufferSize) {
        continue;  // dont close, we might encounter it again
      }
      if (!pNeighbor->bOpen) {
        pNeighbor->nToTarget = nToTarget;
        pNeighbor->bOpen = true;
        rgOpen.push(pNeighbor);
      } else if ((pNeighbor->nToStart == -1) ||
                 (nToStart >= pNeighbor->nToStart)) {
        continue;
      }
      pNeighbor->pPrevious = pCurrent;
      pNeighbor->nToStart = nToStart;
    }
  }
  return -1;
}
