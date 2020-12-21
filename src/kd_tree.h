#ifndef CUSTOM_KDTREE_H_
#define CUSTOM_KDTREE_H_

#include "render/render.h"

// Structure to represent node of a KD-tree
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


// Implementing the KD-Tree
template <typename PointT>
struct KdTree
{
	KdTree()
	: root(NULL)
	{}

    void Insert(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        for(unsigned int id =0 ; id < cloud->points.size(); id++){
            InsertHelper(&root, 0, cloud->points[id], id);
        }
    }


    // return a list of point ids in the tree that are within distance of target
    std::vector<int> Search(PointT target, float distanceTolerance)
    {
        std::vector<int> ids;
        SearchHelper(target, root, 0, distanceTolerance, ids);

        return ids;
    }


private:

    void InsertHelper(Node** node, uint depth, PointT point, int id)
    {
	    std::string location = "";

	    if(*node==NULL) // Tree is empty
        {
            std::vector<float> point_vector (point.data, point.data+3);
	        *node = new Node(point_vector, id);
	        location = "root";
        }
	    else
        {
	        // Calculate current dimension of the tree - is it even or odd?
	        uint dimension = depth % 2;

	        if(point.data[dimension] < ((*node)->point[dimension]))
            {
                InsertHelper(&((*node)->left), depth + 1, point, id);
                location = "left";
            }
	        else
            {
                InsertHelper(&((*node)->right), depth + 1, point, id);
                location = "right";
            }
        }

	    //std::cout << "Inserted (" << point[0] << "," << point[1] << ") at depth " << depth << " on the " << location << std::endl;
    }


    void SearchHelper(PointT target, Node* node, int depth, float distanceTolerance, std::vector<int>& ids)
    {
        if(node!=NULL)
        {
            if(
                    (node->point[0] >= (target.data[0]-distanceTolerance) && node->point[0] <=(target.data[0]+distanceTolerance)) &&
                    (node->point[1] >= (target.data[1]-distanceTolerance) && node->point[1] <=(target.data[1]+distanceTolerance))
                    )
            {
                float distance = sqrt(
                        (node->point[0]-target.data[0]) * (node->point[0]-target.data[0]) +
                        (node->point[1]-target.data[1]) * (node->point[1]-target.data[1])
                );

                if(distance <= distanceTolerance)
                {
                    ids.push_back(node->id);
                }
            }

            // check across boundary to determine if we flow to the left or right
            if((target.data[depth%2] - distanceTolerance) < node->point[depth%2])
            {
                SearchHelper(target, node->left, depth + 1, distanceTolerance, ids);
            }
            if((target.data[depth%2] + distanceTolerance) > node->point[depth%2])
            {
                SearchHelper(target, node->right, depth + 1, distanceTolerance, ids);
            }
        }
    }

    Node* root;
};


#endif /* CUSTOM_KDTREE_H_ */


