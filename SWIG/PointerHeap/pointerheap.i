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
    void* parent;
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
extern HeapNode*  MakeNode(int id, int dependents, int step, void* parent, int decisionID, double time, double velocity, int gear, int delta, double timeToShift);
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
extern int GetDecision(HeapNode* node);
extern HeapNode* GetParent(HeapNode* node);
extern void SetParent(HeapNode* node, HeapNode* parent);
extern double GetTime(HeapNode* node);
extern double GetVelocity(HeapNode* node);
extern int GetGear(HeapNode* node);
extern int GetDelta(HeapNode* node);
extern double GetTimeToShift(HeapNode* node);
extern int GetScore(HeapNode* node);
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
    void* parent;
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
extern HeapNode*  MakeNode(int id, int dependents, int step, void* parent, int decisionID, double time, double velocity, int gear, int delta, double timeToShift);
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
extern int GetDecision(HeapNode* node);
extern HeapNode* GetParent(HeapNode* node);
extern void SetParent(HeapNode* node, HeapNode* parent);
extern double GetTime(HeapNode* node);
extern double GetVelocity(HeapNode* node);
extern int GetGear(HeapNode* node);
extern int GetDelta(HeapNode* node);
extern double GetTimeToShift(HeapNode* node);
extern int GetScore(HeapNode* node);