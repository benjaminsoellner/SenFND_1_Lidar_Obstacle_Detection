/* \author Aaron Brown, Benjamin Söllner */

#ifndef KDTREE_H_
#define KDTREE_H_

#include <pcl/common/common.h>


template<typename PointT>
int getDimension(const PointT& point, uint dimension) {
	switch (dimension) {
		case 0:
			return point.x;
			break;
		case 1:
			return point.y;
			break;
		case 2:
			return point.z;
			break;
	}
}

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT p, int setId)
	:	point(p), id(setId), left(NULL), right(NULL)
	{}
};


template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insertNode(Node<PointT>** node, uint depth, PointT point, int id)
	{
		if (*node == NULL)
		{
			*node = new Node<PointT>(point, id);
		}
		else
		{
			uint dimension = depth % 3;
			if (getDimension(point, dimension) < getDimension((*node)->point, dimension))
			{
				insertNode(&((*node)->left), depth+1, point, id);
			}
			else
			{
				insertNode(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert(PointT point, int id)
	{
		insertNode(&root, 0, point, id);
	}

	std::vector<int> searchNode(Node<PointT>* node, uint depth, PointT target, float distanceTol)
	{
		uint dimension = depth % 3;
		std::vector<int> found;
		PointT point = node->point;
		bool boxLeft = getDimension(target, dimension)-distanceTol*2 < getDimension(point, dimension);
		bool boxRight = getDimension(target, dimension)+distanceTol*2 >= getDimension(point, dimension);
		bool inBox = boxLeft && boxRight;
		if (inBox)
		{
			float distance = sqrt((point.x-target.x)*(point.x-target.x)
					+ (point.y-target.y)*(point.y-target.y)
					+ (point.z-target.z)*(point.z-target.z));
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
	std::vector<int> search(PointT target, float distanceTol)
	{
		return searchNode(root, 0, target, distanceTol);
	}
	

};

#endif /* KDTREE_H_ */