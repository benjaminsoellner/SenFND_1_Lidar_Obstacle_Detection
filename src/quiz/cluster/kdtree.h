/* \author Aaron Brown, Benjamin Söllner */
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

	void insertNode(Node** node, uint depth, std::vector<float> point, int id)
	{
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			uint dimension = depth % 2;
			if (point[dimension] < ((*node)->point[dimension]))
			{
				insertNode(&((*node)->left), depth+1, point, id);
			}
			else
			{
				insertNode(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertNode(&root, 0, point, id);
	}

	std::vector<int> searchNode(Node* node, uint depth, std::vector<float> target, float distanceTol)
	{
		uint dimension = depth % 2;
		std::vector<int> found;
		std::vector<float> point = node->point;
		bool boxLeft = target[dimension]-distanceTol*2 < point[dimension];
		bool boxRight = target[dimension]+distanceTol*2 >= point[dimension];
		bool inBox = boxLeft && boxRight;
		if (inBox)
		{
			float distance = sqrt((point[0]-target[0])*(point[0]-target[0]) + (point[1]-target[1])*(point[1]-target[1]));
			if (distance < distanceTol)
			{
				found.push_back(node->id);
			} 
		}
		if (boxLeft && node->left != NULL) {
			std::vector<int> foundLeft = searchNode(node->left, depth+1, target, distanceTol);
			found.insert( found.end(), foundLeft.begin(), foundLeft.end() );
		}
		if (boxRight && node->right != NULL) {
			std::vector<int> foundRight = searchNode(node->right, depth+1, target, distanceTol);
			found.insert( found.end(), foundRight.begin(), foundRight.end() );
		}
		return found;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		return searchNode(root, 0, target, distanceTol);
	}
	

};




