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

	    //std::cout << "Inserted (" << point[0] << "," << point[1] << ") at depth " << depth << " on the " << location << std::endl;
    }


	void insert(std::vector<float> point, int id)
	{
        insertHelper(&root, 0, point, id);
	}


	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTolerance, std::vector<int>& ids)
    {
	    if(node!=NULL)
        {
	        if(
	                (node->point[0] >= (target[0]-distanceTolerance) && node->point[0] <=(target[0]+distanceTolerance)) &&
	                (node->point[1] >= (target[1]-distanceTolerance) && node->point[1] <=(target[1]+distanceTolerance))
	        )
            {
	            float distance = sqrt(
	                    (node->point[0]-target[0]) * (node->point[0]-target[0]) +
	                    (node->point[1]-target[1]) * (node->point[1]-target[1])
	                    );

	            if(distance <= distanceTolerance)
                {
	                ids.push_back(node->id);
                }
            }

	        // check across boundary to determine if we flow to the left or right
	        if((target[depth%2] - distanceTolerance) < node->point[depth%2])
            {
	            searchHelper(target, node->left, depth+1, distanceTolerance, ids);
            }
            if((target[depth%2] + distanceTolerance) > node->point[depth%2])
            {
                searchHelper(target, node->right, depth+1, distanceTolerance, ids);
            }
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTolerance)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTolerance, ids);

		return ids;
	}
};




