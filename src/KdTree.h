#ifndef KDTREE_H
#define KDTREE_H

struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
    :   point(arr), id(setId), left(NULL), right(NULL)
    {}
};



class KdTree
{
private:
    Node* root;
    int level;
public:    
    KdTree();
    void insertHelper(Node *&node, std::vector<float> point, int id, int level);
    void insert(std::vector<float> point, int id);
    void searchHelper(Node* root, std::vector<float> &target, float &distanceTol, std::vector<int> &ids, int depth);

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol);

    void proximity(const std::vector<std::vector<float>>& points, std::vector<int> &cluster, int point_idx, std::vector<bool> &is_processed, KdTree* tree, float distanceTol);

    std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

    
};
#endif