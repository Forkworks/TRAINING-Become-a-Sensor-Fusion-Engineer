/* \author Aaron Brown */
// Quiz on implementing kd tree

//#include "../../render/render.h"


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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node** node,uint depth , std::vector<float> point, int id)
    {
        if(*node == NULL)
        {
            *node = new Node(point,id);
        }
        else
        {

            uint cd = depth%2;  //0 or 1 (x or y)
            if (point[cd] < (*node)->point[cd])
            {
                insertHelper(&((*node)->left), depth+1, point, id);
            }
            else
            {
                insertHelper(&((*node)->right), depth+1, point, id);
            }

        }
    }

    void insertHelper2(Node* node,uint depth , std::vector<float> point, int id)
    {
        if(node == NULL)
        {
            node = new Node(point,id);    //Node node(point,id);
        }
        else
        {

            uint cd = depth%2;  //0 or 1 (x or y)
            if (point[cd] < node->point[cd])
            {
                insertHelper2(node->left, depth+1, point, id);
            }
            else
            {
                insertHelper2(node->right, depth+1, point, id);
            }

        }
    }

    void insertHelper3(Node* &node,uint depth , std::vector<float> point, int id)
    {
        if(node == NULL)
        {
            node = new Node(point,id);    //Node node(point,id);
        }
        else
        {

            uint cd = depth%3;  //0 or 1 (x or y)
            //(depth = 0 -> cd=0)
            //(depth = 1 -> cd=1)
            //(depth = 2 -> cd=2)
            //(depth = 3 -> cd=0)


            if (point[cd] < node->point[cd])
            {
                insertHelper3(node->left, depth+1, point, id);
            }
            else
            {
                insertHelper3(node->right, depth+1, point, id);
            }

        }
    }


	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
        //insertHelper(&root,0,point,id);
        insertHelper3(root, 0, point, id);
	}

	void searchHelper(Node* node, std::vector<float> target, float distanceTol, std::vector<int> &ids, uint depth)
    {



        if(node == NULL)
        {

        }
        else
        {
            float side_x = node->point[0]-target[0];
            float side_y = node->point[1]-target[1];
            float side_z = node->point[2]-target[2];
            if (fabs(side_x) < distanceTol and fabs(side_y) < distanceTol and fabs(side_z) < distanceTol)
            {
                if (sqrt(pow(side_x, 2) + pow(side_y, 2) + pow(side_z,2)) < distanceTol)
                    ids.push_back(node->id);
            }

            uint cd = depth % 3;  //0 or 1 (x or y)

            if ((target[cd] - distanceTol) < node->point[cd])  //((-node->point[cd] + target[cd]) < distanceTol)
            {
                searchHelper(node->left, target, distanceTol, ids, depth + 1);
            }
            if ((target[cd] + distanceTol) > node->point[cd])
            {
                searchHelper(node->right, target, distanceTol, ids, depth + 1);
            }


        }




    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

        searchHelper(root, target, distanceTol, ids,0);

    	return ids;
	}
	

};




