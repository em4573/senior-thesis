#include <stdlib.h>
#include <limits.h>

typedef struct GearData {
    int gear;
    int delta;
    float timeToShift;
} GearData;

typedef struct HeapNode {
    float score;

    int ID;
    int step;
    int parentID;
    int decisionID;
    float time;
    float velocity;
    GearData gd;
} HeapNode;
 
HeapNode *heap[500];
int axes;
int heapSize;

void Init(int a) {
    axes = a;
    heapSize = 0;

    heap[0] = malloc(sizeof(HeapNode));
    heap[0]->score = -INT_MAX;
}

void Destroy() {
    free(heap[0]);
}

HeapNode* MakeNode(int ID, int step, int parentID, int decisionID, float time, float velocity, int gear, int delta, float timeToShift) {
    HeapNode *node = malloc(sizeof(HeapNode));

    node->ID = ID;
    node->step = step;
    node->parentID = parentID;
    node->decisionID = decisionID;
    node->time = time;
    node->velocity = velocity;
    node->gd.gear = gear;
    node->gd.delta = delta;
    node->gd.timeToShift = timeToShift;

    return node;
}

void KillNode(HeapNode* node) {
    free(node);
}
 
void Insert(HeapNode* node, float score) {
    heapSize++;

    int pos = heapSize;
    while (heap[pos / 2]->score > score) {
        heap[pos] = heap[pos / 2];
        pos /= 2;
    }

    heap[pos] = node;
    heap[pos]->score = score;
}
 
HeapNode* DeleteMin() {
    if (heapSize <= 0) return 0;

    HeapNode *minElement, *lastElement;
    int child, pos;

    minElement = heap[1];
    lastElement = heap[heapSize];
    heapSize--;

    for (pos = 1; pos * 2 <= heapSize; pos = child) {
        child = pos * 2;

        if (child != heapSize && heap[child + 1]->score < heap[child]->score) {
            child++;
        }

        if (lastElement->score > heap[child]->score) {
            heap[pos] = heap[child];
        } else break;
    }

    heap[pos] = lastElement;
    return minElement;
}


int GetScore(HeapNode* node) {
    return node->score;
}

int GetID(HeapNode* node) {
    return node->ID;
}