#include <vision/structures/Blob.h>

bool sortBlobAreaPredicate(Blob* left, Blob* right) {
  return left->dx * left->dy < right->dx * right->dy;
}

bool BlobCompare(Blob a, Blob b) {
    return a.dx * a.dy > b.dx * b.dy;
}

