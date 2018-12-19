%module pointerheap
%{
#include <stdlib.h>
#include <limits.h>

typedef struct GearData {
    int gear;
    int delta;
    double timeToShift;
} GearData;

typedef struct HeapNode {
    double score;

    int dependents;
    int step;
    int parent;
    int decisionID;
    double time;
    double velocity;
    GearData gd;
} HeapNode;
 
extern HeapNode** workingHeap;
extern HeapNode** reserveHeap;
extern int maxHeapSize;
extern int workingHeapSize;
extern int reserveHeapSize;

extern int maxint;
extern int nodesMade;

extern void Init(int h);
extern void SwapHeaps();
extern void Destroy();
extern HeapNode*  MakeNode(int id, int dependents, int step, int parent, int decisionID, double time, double velocity, int gear, int delta, double timeToShift);
extern HeapNode* MakeInitNode();
extern void KillNode(HeapNode* node);
extern void FreeNode(HeapNode* node);
extern int Insert(HeapNode* node, int level, double score);
extern HeapNode* DeleteMin(int level);

extern int GetID(HeapNode* node);
extern void SetID(HeapNode* node, int id);
extern int GetDependents(HeapNode* node);
extern void SetDependents(HeapNode* node, int d);
extern int GetStep(HeapNode* node);
extern void SetStep(HeapNode* node, int s);
extern int GetDecision(HeapNode* node);
extern void SetDecision(HeapNode* node, int d);
extern int GetParent(HeapNode* node);
extern void SetParent(HeapNode* node, int parent);
extern double GetTime(HeapNode* node);
extern void SetTime(HeapNode* node, double d);
extern double GetVelocity(HeapNode* node);
extern void SetVelocity(HeapNode* node, double d);
extern int GetGear(HeapNode* node);
extern void SetGear(HeapNode* node, int d);
extern int GetDelta(HeapNode* node);
extern void SetDelta (HeapNode* node, int d);
extern double GetTimeToShift(HeapNode* node);
extern void SetTimeToShift(HeapNode* node, double d);
extern int GetScore(HeapNode* node);
extern void SetScore(HeapNode* node, double d);
%}

typedef struct GearData {
    int gear;
    int delta;
    double timeToShift;
} GearData;

typedef struct HeapNode {
    double score;

    int dependents;
    int step;
    int parent;
    int decisionID;
    double time;
    double velocity;
    GearData gd;
} HeapNode;
  
extern HeapNode** workingHeap;
extern HeapNode** reserveHeap;
extern int maxHeapSize;
extern int workingHeapSize;
extern int reserveHeapSize;

extern int maxint;
extern int nodesMade;

extern void Init(int h);
extern void SwapHeaps();
extern void Destroy();
extern HeapNode*  MakeNode(int id, int dependents, int step, int parent, int decisionID, double time, double velocity, int gear, int delta, double timeToShift);
extern HeapNode* MakeInitNode();
extern void KillNode(HeapNode* node);
extern void FreeNode(HeapNode* node);
extern int Insert(HeapNode* node, int level, double score);
extern HeapNode* DeleteMin(int level);

extern int GetID(HeapNode* node);
extern void SetID(HeapNode* node, int id);
extern int GetDependents(HeapNode* node);
extern void SetDependents(HeapNode* node, int d);
extern int GetStep(HeapNode* node);
extern void SetStep(HeapNode* node, int s);
extern int GetDecision(HeapNode* node);
extern void SetDecision(HeapNode* node, int d);
extern int GetParent(HeapNode* node);
extern void SetParent(HeapNode* node, int parent);
extern double GetTime(HeapNode* node);
extern void SetTime(HeapNode* node, double d);
extern double GetVelocity(HeapNode* node);
extern void SetVelocity(HeapNode* node, double d);
extern int GetGear(HeapNode* node);
extern void SetGear(HeapNode* node, int d);
extern int GetDelta(HeapNode* node);
extern void SetDelta (HeapNode* node, int d);
extern double GetTimeToShift(HeapNode* node);
extern void SetTimeToShift(HeapNode* node, double d);
extern int GetScore(HeapNode* node);
extern void SetScore(HeapNode* node, double d);