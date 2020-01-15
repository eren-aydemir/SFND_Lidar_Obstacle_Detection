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

	KdTree()
	: root(NULL)
	{}

	void insertNode(Node **node, std::vector<float> point, int id, uint depth) 
	{

		int cd = depth % 2;
		//root
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else if (point[cd] < (*node)->point[cd])
		{
			insertNode(&((*node)->left), point, id, depth+1);
		}
		else
		{
			insertNode(&((*node)->right), point, id, depth+1);
		}

	} 

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertNode(&root, point, id, 0); 
	}

	void searchKDTree(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int>& ids){

		int cd = depth % 2;
		if (node != NULL){
			
			if ((node->point[0] <= target[0]+distanceTol) && (node->point[0] >= target[0]-distanceTol) && (node->point[1] <= target[1]+distanceTol) && (node->point[1] >= target[1]-distanceTol))
			{
				ids.push_back(node->id);
			}

			if (target[cd]-distanceTol < node->point[cd]) //very left point of box
			{
				searchKDTree(target, node->left, depth+1, distanceTol, ids);
			}
			if (target[cd]+distanceTol > node->point[cd]) //very right point
			{
				searchKDTree(target, node->right, depth+1, distanceTol, ids);
			}
			
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchKDTree(target, root, 0, distanceTol, ids);

		return ids;
	}
	
};