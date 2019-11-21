/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;
	int level;
	KdTree()
	: root(NULL), level(0)
	{}

	void insertHelper(Node *&node, std::vector<float> point, int id, int level){
		if(node == NULL){
			node = new Node(point,id);
		}
		else if(level%2 == 0){
			if(point[0] < node->point[0]){
				insertHelper(node->left, point, id, level+1);
			}
			else{
				insertHelper(node->right, point, id, level+1);
			}			
		}
		else{
			if(point[1] < node->point[1]){
				insertHelper(node->left, point, id, level+1);
			}
			else{
				insertHelper(node->right, point, id, level+1);
			}			
		}
	}

	void insert(std::vector<float> point, int id)
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

	void searchHelper(Node* root, std::vector<float> &target, float &distanceTol, std::vector<int> &ids, int depth){
		if(root){
			float root_x = root->point[0];
			float root_y = root->point[1];
			if((root_x <= target[0]+distanceTol && root_x >= target[0]-distanceTol) && (root_y <= target[1]+distanceTol && root_y >= target[1]-distanceTol )){
				float x_diff = root_x - target[0];
				float y_diff = root_y - target[1];
				float distance = sqrt(x_diff*x_diff+y_diff*y_diff);
				if(distance <= distanceTol){
					ids.push_back(root->id);
				}
			}
			int compared_coord_idx = depth%2;
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
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root,target,distanceTol,ids,0);
		return ids;
	}
	

};




