#ifndef __MYKDTREE_H__
#define __MYKDTREE_H__

#include <vector>

using namespace std;

#define KDTREE_DIM 3 // data dimensions

struct KDPoint
{
    float coords[KDTREE_DIM];
};

class KDNode
{
public:
    KDNode()
    {
        parent = NULL;
        left = NULL;
        right = NULL;
        split_value = -1;
        _parent = -1;
        _left = -1;
        _right = -1;
    }

    int id; // for GPU
    int level;
    KDNode *parent, *left, *right;
    int _parent, _left, _right; // for GPU
    float split_value;
    vector <int> indexes; // index to points
};

class KDtree
{
public:
    KDtree();
    ~KDtree();
    void Create(vector <KDPoint> &pts, int max_levels = 99 /* You can limit the search depth if you want */);
    void Search(const KDPoint &query, int *ret_index, float *ret_sq_dist);
    int GetNumNodes() const { return m_id; }
    KDNode* GetRoot() const { return m_root; }

    static bool SortPoints(const int a, const int b);

private:
    vector <KDPoint> *m_pts;
    KDNode *m_root;
    int m_current_axis;
    int m_levels;
    int m_cmps; // count how many comparisons were made in the tree for a query
    int m_id; // current node ID

    void Split(KDNode *cur, KDNode *left, KDNode *right);
    void SearchAtNode(KDNode *cur, const KDPoint &query, int *ret_index, float *ret_dist, KDNode **ret_node);
    void SearchAtNodeRange(KDNode *cur, const KDPoint &query, float range, int *ret_index, float *ret_dist);
    inline float Distance(const KDPoint &a, const KDPoint &b) const;
};

float KDtree::Distance(const KDPoint &a, const KDPoint &b) const
{
    float dist = 0;

    for(int i=0; i < KDTREE_DIM; i++) {
        float d = a.coords[i] - b.coords[i];
        dist += d*d;
    }

    return dist;
}

#endif
