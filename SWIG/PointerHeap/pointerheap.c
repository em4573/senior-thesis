/* https://stackoverflow.com/questions/21540037/how-to-convert-a-c-array-to-a-python-list-using-swig */

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>

typedef struct GearData {
    int gear;
    int delta;
    double timeToShift;
} GearData;

typedef struct HeapNode {
    double score;

    int id;
    int dependents;
    int step;
    void* parent;
    int decisionID;
    double time;
    double velocity;
    GearData gd;
} HeapNode;
 
HeapNode** workingHeap;
HeapNode** reserveHeap;
int maxHeapSize;
int workingHeapSize;
int workingReady;
int reserveReady;
int reserveHeapSize;

int maxint = INT_MAX;
int nodesMade = 0;

void Init(int h) {
    maxHeapSize = h;
    workingHeapSize = 0;
    reserveHeapSize = 0;

    workingHeap = malloc(h * sizeof(HeapNode*));
    if (workingHeap == NULL) {
        return;
    } else {
        workingReady = 1;
    }

    reserveHeap = malloc(h * sizeof(HeapNode*));
    if (reserveHeap == NULL) {
        return;
    } else {
        reserveReady = 1;
    }

    workingHeap[0] = malloc(sizeof(HeapNode));
    if (workingHeap[0] == NULL) {
        return;
    }
    workingHeap[0]->score = -INT_MAX;

    reserveHeap[0] = malloc(sizeof(HeapNode));
    if (reserveHeap[0] == NULL) {
        return;
    }
    reserveHeap[0]->score = -INT_MAX;
}

void SwapHeaps() {
    HeapNode** temp = workingHeap;
    workingHeap = reserveHeap;
    reserveHeap = temp;
}

void Destroy() {
    free(workingHeap);
    free(reserveHeap);
}

HeapNode* MakeNode(int id, int dependents, int step, void* parent, int decisionID, double time, double velocity, int gear, int delta, double timeToShift) {
    nodesMade++;
    HeapNode *node = malloc(sizeof(HeapNode));
    
    if (node == NULL) {
        return 0;
    }

    node->id = id;
    node->dependents = dependents;
    node->step = step;
    node->parent = parent;
    node->decisionID = decisionID;
    node->time = time;
    node->velocity = velocity;
    node->gd.gear = gear;
    node->gd.delta = delta;
    node->gd.timeToShift = timeToShift;

    return node;    
}

HeapNode* MakeInitNode() {
    return MakeNode(0, 0, 0, 0, 0, 0.0, 0.0, 0, 0, 0.0);
}

void KillNode(HeapNode* node) {
    if (node->dependents <= 0) {
        HeapNode* k = node->parent;
        k->dependents -= 1;
        KillNode(node->parent);
        free(node);
    }
}

void FreeNode(HeapNode* node) {
    free(node);
}
 
int Insert(HeapNode* node, int level, double score) {
    int pos = 0;
    if (level == 0) {
        if (workingHeapSize >= maxHeapSize) {
            return -1;
        }
        workingHeapSize++;

        pos = workingHeapSize;
        while (workingHeap[pos / 2]->score > score) {
            workingHeap[pos] = workingHeap[pos / 2];
            pos /= 2;
        }

        workingHeap[pos] = node;
        workingHeap[pos]->score = score;
    } else {
        if (reserveHeapSize >= maxHeapSize) {
            return -2;
        }
        reserveHeapSize++;

        pos = reserveHeapSize;
        while (reserveHeap[pos / 2]->score > score) {
            reserveHeap[pos] = reserveHeap[pos / 2];
            pos /= 2;
        }

        /*fprintf(stderr, "rf-%d\n", pos);*/
        reserveHeap[pos] = node;
        reserveHeap[pos]->score = score;
    }

    return 0;
}
 
HeapNode* DeleteMin(int level) {
    if (level == 0) {
        if (workingHeapSize <= 0) return 0;

        HeapNode *minElement, *lastElement;
        int child, pos;

        minElement = workingHeap[1];
        lastElement = workingHeap[workingHeapSize];
        workingHeapSize--;

        for (pos = 1; pos * 2 <= workingHeapSize; pos = child) {
            child = pos * 2;

            if (child != workingHeapSize && workingHeap[child + 1]->score < workingHeap[child]->score) {
                child++;
            }

            if (lastElement->score > workingHeap[child]->score) {
                workingHeap[pos] = workingHeap[child];
            } else break;
        }

        workingHeap[pos] = lastElement;
        return minElement;
    } else {
        if (reserveHeapSize <= 0) return 0;

        HeapNode *minElement, *lastElement;
        int child, pos;

        minElement = reserveHeap[1];
        lastElement = reserveHeap[reserveHeapSize];
        reserveHeapSize--;

        for (pos = 1; pos * 2 <= reserveHeapSize; pos = child) {
            child = pos * 2;

            if (child != reserveHeapSize && reserveHeap[child + 1]->score < reserveHeap[child]->score) {
                child++;
            }

            if (lastElement->score > reserveHeap[child]->score) {
                reserveHeap[pos] = reserveHeap[child];
            } else break;
        }

        reserveHeap[pos] = lastElement;
        return minElement;
    }
}

int GetID(HeapNode* node) {
    return node->id;
}

void SetID(HeapNode* node, int id) {
    node->id = id;
}

int GetDependents(HeapNode* node) {
    return node->dependents;
}

void SetDependents(HeapNode* node, int d) {
    node->dependents = d;
}

int GetStep(HeapNode* node) {
    return node->step;
}

void SetStep(HeapNode* node, int s) {
    node->step = s;
}

int GetDecision(HeapNode* node) {
    return node->decisionID;
}

void SetDecision(HeapNode* node, int d) {
    node->decisionID = d;
}

HeapNode* GetParent(HeapNode* node) {
    return node->parent;
}

void SetParent(HeapNode* node, HeapNode* parent) {
    node->parent = parent;
}

double GetTime(HeapNode* node) {
    return node->time;
}

void SetTime(HeapNode* node, double d) {
    node->time = d;
}

double GetVelocity(HeapNode* node) {
    return node->velocity;
}

void SetVelocity(HeapNode* node, double d) {
    node->velocity = d;
}

int GetGear(HeapNode* node) {
    return node->gd.gear;
}

void SetGear(HeapNode* node, int d) {
    node->gd.gear = d;
}

int GetDelta(HeapNode* node) {
    return node->gd.delta;
}

void SetDelta (HeapNode* node, int d) {
    node->gd.delta = d;
}

double GetTimeToShift(HeapNode* node) {
    return node->gd.timeToShift;
}

void SetTimeToShift(HeapNode* node, double d) {
    node->gd.timeToShift = d;
}

double GetScore(HeapNode* node) {
    return node->score;
}

void SetScore(HeapNode* node, double d) {
    node->score = d;
}

/* try passing back chars and converting to double in python */