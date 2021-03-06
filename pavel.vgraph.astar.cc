// 2016 Pavel Krasnovskij
// Base visual graph and a-star search
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <thread>
#include <mutex>
using namespace std;

class Node {
  int nX, nY;

 public:
  bool bClosed, bOpen;
  int nToStart, nTotal, nHeuristic;
  mutex Mutex;
  Node *pPrevious;
  vector<Node *> rgNeighbors;
  vector<int> rgEdges;
  Node(const int x, const int y);
  Node(const int x, const int y, Node *pTarget);
  int GetDistance(Node *pTarget, const unsigned char *pMap,
                  const int nMapWidth);
  void FillPathTo(Node *pTarget, int *pOutBuffer, const int nMapWidth);
  void Init(const int x, const int y);
  string GetPath();
  string ToString();
};

void SetEdges(const unsigned char *pMap, const int nMapWidth,
              vector<Node *> rgKeypoints, const int nFrom, const int nTo) {
  for (int i = nFrom; i < nTo; i++) {
    Node *pFrom = rgKeypoints[i];
    for (int j = i + 1; j < rgKeypoints.size(); j++) {
      Node *pTo = rgKeypoints[j];
      int nDistance = pFrom->GetDistance(pTo, pMap, nMapWidth);
      if (nDistance > 0) {
        {
          lock_guard<mutex> guardFrom(pTo->Mutex);
          pTo->rgNeighbors.push_back(pFrom);
          pTo->rgEdges.push_back(nDistance);
        }
        lock_guard<mutex> guardTo(pFrom->Mutex);
        pFrom->rgNeighbors.push_back(pTo);
        pFrom->rgEdges.push_back(nDistance);
      }
    }
  }
}

string Node::ToString() { return to_string(nX) + "," + to_string(nY); }

bool SortByDistanceToTarget(Node *pNode1, Node *pNode2) {
  return ((pNode2->nTotal != -1) && (pNode1->nTotal > pNode2->nTotal));
}

string Node::GetPath() {
  if (pPrevious == NULL) {
    return "";
  }
  return pPrevious->GetPath() + "->" + ToString();
}

struct ByDistanceToTarget {
  bool operator()(Node *pNode1, Node *pNode2) {
    return ((pNode2->nTotal != -1) && (pNode1->nTotal > pNode2->nTotal));
  }
};

void Node::Init(const int x, const int y) {
  pPrevious = NULL;
  bClosed = false;
  bOpen = false;
  nTotal = -1;
  nToStart = -1;
  nHeuristic = 0;
  nX = x;
  nY = y;
}

Node::Node(const int x, const int y) { Init(x, y); }

Node::Node(const int x, const int y, Node *pTarget) {
  Init(x, y);
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

bool IsCorner(const int nX, const int nY, const int nCornerX,
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
      if (IsCorner(x, y, x - 1, y - 1, pMap, nMapWidth, nMapHeight) ||
          IsCorner(x, y, x + 1, y - 1, pMap, nMapWidth, nMapHeight) ||
          IsCorner(x, y, x + 1, y + 1, pMap, nMapWidth, nMapHeight) ||
          IsCorner(x, y, x - 1, y + 1, pMap, nMapWidth, nMapHeight)) {
        rgKeypoints.push_back(new Node(x, y, pTarget));
      }
    }
  }
  int nThreads = thread::hardware_concurrency();
  if (rgKeypoints.size() / 50 < nThreads) {
    nThreads = rgKeypoints.size() / 50 + 1;
  }
  if (nThreads < 1) {
    // always run at least one
    SetEdges(ref(pMap), nMapWidth, ref(rgKeypoints), 0, rgKeypoints.size());
  } else {
    vector<thread> rgThreads(nThreads);
    int nVerticesPerThread = rgKeypoints.size() / nThreads;
    for (int i = 0; i < nThreads - 1; i++) {
      rgThreads[i] =
          thread(SetEdges, pMap, nMapWidth, ref(rgKeypoints),
                 i * nVerticesPerThread, (i + 1) * nVerticesPerThread);
    }
    rgThreads[nThreads - 1] =
        thread(SetEdges, pMap, nMapWidth, ref(rgKeypoints),
               (nThreads - 1) * nVerticesPerThread, rgKeypoints.size());
    for (int i = 0; i < nThreads; i++) {
      rgThreads[i].join();
    }
  }
  // ida*
  vector<Node *> rgOpen;
  pStart->nToStart = 0;
  pStart->nTotal = pStart->nHeuristic;
  pStart->bOpen = true;
  rgOpen.push_back(pStart);
  int nToStart, nToCurrent;
  while (rgOpen.size() > 0) {
    sort(rgOpen.begin(), rgOpen.end(), SortByDistanceToTarget);
    Node *pCurrent = rgOpen.back();
    if (pCurrent == pTarget) {
      pTarget->FillPathTo(pStart, pOutBuffer, nMapWidth);
      return pTarget->nToStart;
    }
    rgOpen.pop_back();
    pCurrent->bOpen = false;
    pCurrent->bClosed = true;
    for (int i = 0; i < pCurrent->rgNeighbors.size(); i++) {
      Node *pNeighbor = pCurrent->rgNeighbors[i];
      if (pNeighbor->bClosed) {
        continue;  // checked already
      }
      nToStart = pCurrent->nToStart + pCurrent->rgEdges[i];
      if (pNeighbor->bOpen &&
          ((pNeighbor->nToStart == -1) || (nToStart >= pNeighbor->nToStart))) {
        continue;
      }
      pNeighbor->nTotal = nToStart + pNeighbor->nHeuristic;
      if (pNeighbor->nTotal > nOutBufferSize) {
        continue;  // dont close, we might encounter it again
      }
      pNeighbor->pPrevious = pCurrent;
      pNeighbor->nToStart = nToStart;
      if (!pNeighbor->bOpen) {
        pNeighbor->bOpen = true;
        rgOpen.push_back(pNeighbor);
      }
    }
  }
  return -1;
}
