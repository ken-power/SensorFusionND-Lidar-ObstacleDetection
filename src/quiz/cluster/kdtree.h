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


	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
    {
	    std::string location = "";

	    if(*node==NULL) // Tree is empty
        {
	        *node = new Node(point, id);
	        location = "root";
        }
	    else
        {
	        // Calculate current dimension of the tree - is it even or odd?
	        uint dimension = depth % 2;

	        if(point[dimension] < ((*node)->point[dimension]))
            {
	            insertHelper(&((*node)->left), depth+1, point, id);
                location = "left";
            }
	        else
            {
                insertHelper(&((*node)->right), depth+1, point, id);
                location = "right";
            }
        }

	    std::cout << "Inserted (" << point[0] << "," << point[1] << ") at depth " << depth << " on the " << location << std::endl;
    }


	void insert(std::vector<float> point, int id)
	{
        insertHelper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}

};




