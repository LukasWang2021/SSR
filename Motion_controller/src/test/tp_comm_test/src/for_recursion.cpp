#include <stdio.h>
#include <iostream>
using namespace std;

#define MAX_SIZE 10

//recursion
#if 0
struct tree_t {
    struct tree_t *left;
    struct tree_t *right;
    struct tree_t *parent;
    int key;
};
typedef struct tree_t tree_t;
void store(int);

void print_tree(tree_t *tree) {
    store(tree->key);

    if (tree->left)
        print_tree(tree->left);

    if (tree->right)
        print_tree(tree->right);
}

int keys[MAX_SIZE];
int count = 0;

void reset_storage() {
    count = 0;
}

void store(int key) {
    keys[count++] = key;
}

#endif

#if 0
//not recursion
struct tree_t {
    struct tree_t *left;
    struct tree_t *right;
    struct tree_t *parent;
    int key;
};

typedef struct tree_t tree_t;
void store(int);

void print_tree(tree_t *tree) {
    tree_t *stack[MAX_SIZE];
    int count = 0;

    stack[count++] = tree;

    while (count) {
        tree = stack[--count];

        store(tree->key);

        if (tree->right)
            stack[count++] = tree->right;

        if (tree->left)
            stack[count++] = tree->left;
    }
}

int keys[MAX_SIZE];
int count = 0;

void reset_storage() {
    count = 0;
}

void store(int key) {
    keys[count++] = key;
}

#endif

#if 1
struct tree_t {
    struct tree_t *left;
    struct tree_t *right;
    struct tree_t *parent;
    int key;
};
typedef struct tree_t tree_t;
void store(int);

void print_tree(tree_t *tree) {
    tree_t *prev;
    prev = 0;

    while (tree) {
        if (prev == tree->parent) {
            store(tree->key);
            prev = tree;
            tree = tree->left  ? tree->left :
                   tree->right ? tree->right :
                                 tree->parent;
        } else if (prev == tree->left && tree->right) {
            prev = tree;
            tree = tree->right;
        } else {
            prev = tree;
            tree = tree->parent;
        }
    }
}

#define MAX_SIZE 10
int keys[MAX_SIZE];
int count = 0;

void reset_storage() {
    count = 0;
}

void store(int key) {
    keys[count++] = key;
}

#endif