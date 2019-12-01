#include "kdtree.h"


KdTree::KdTree(){
    root = NULL;
    level = 0;
}

void KdTree::insertHelper(Node *&node, std::vector<float> point, int id, int level){
        if(node == NULL){
            node = new Node(point,id);
        }
        else if(level%point.size() == 0){
            if(point[0] < node->point[0]){
                insertHelper(node->left, point, id, level+1);
            }
            else{
                insertHelper(node->right, point, id, level+1);
            }           
        }
        else if(level%point.size() == 1){
            if(point[1] < node->point[1]){
                insertHelper(node->left, point, id, level+1);
            }
            else{
                insertHelper(node->right, point, id, level+1);
            }           
        }
        else{
            if(point[2] < node->point[2]){
                insertHelper(node->left, point, id, level+1);
            }
            else{
                insertHelper(node->right, point, id, level+1);
            }               
        }
}


void KdTree::insert(std::vector<float> point, int id)
{
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the root 
    if(root == NULL){
        Node* new_node = new Node(point,id);
        root = new_node;
    }
    else if(point[0] < root->point[0]){
        insertHelper(root->left, point, id, 1);
    }
    else{
        insertHelper(root->right, point, id, 1);
    }
}

void KdTree::searchHelper(Node* root, std::vector<float> &target, float &distanceTol, std::vector<int> &ids, int depth){
    if(root){
        float root_x = root->point[0];
        float root_y = root->point[1];
        float root_z = root->point[2];
        if((root_x <= target[0]+distanceTol && root_x >= target[0]-distanceTol) && (root_y <= target[1]+distanceTol && root_y >= target[1]-distanceTol ) && (root_z <= target[2]+distanceTol && root_z >= target[2]-distanceTol )){
            float x_diff = root_x - target[0];
            float y_diff = root_y - target[1];
            float z_diff = root_z - target[2];
            float distance = sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);
            if(distance <= distanceTol){
                ids.push_back(root->id);
            }
        }
        int compared_coord_idx = depth%target.size();
        int compared_coord_val = root->point[compared_coord_idx];

        if(compared_coord_val > target[compared_coord_idx]-distanceTol){
            searchHelper(root->left,target,distanceTol,ids,depth+1);
        }

        if(compared_coord_val < target[compared_coord_idx]+distanceTol){
            searchHelper(root->right,target,distanceTol,ids,depth+1);
        }
    }
}

// return a list of point ids in the tree that are within distance of target
std::vector<int> KdTree::search(std::vector<float> target, float distanceTol)
{
    std::vector<int> ids;
    searchHelper(root,target,distanceTol,ids,0);
    return ids;
}


void KdTree::proximity(const std::vector<std::vector<float>>& points, std::vector<int> &cluster, int point_idx, std::vector<bool> &is_processed, KdTree* tree, float distanceTol){
    is_processed[point_idx] = true;
    cluster.push_back(point_idx);
    std::vector<int> nearby_points = tree->search(points[point_idx], distanceTol);
    for(int i = 0; i < nearby_points.size(); i++){
        if(!is_processed[nearby_points[i]]){
            proximity(points,cluster,nearby_points[i],is_processed,tree,distanceTol);
        }
    }
}

std::vector<std::vector<int>> KdTree::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;
    std::vector<bool> is_processed(points.size(),false);
    for(int i = 0; i < points.size(); i++){
        if(!is_processed[i]){
            std::vector<int> new_cluster;
            proximity(points,new_cluster,i,is_processed,tree,distanceTol);
            clusters.push_back(new_cluster);
        }
    }
    return clusters;

}

