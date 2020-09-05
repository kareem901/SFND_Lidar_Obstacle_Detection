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
	void insert_helper(Node** node,uint depth,std::vector<float> point, int id)
	{
		auto itr=point.begin();
		if (*node==NULL)
		{
			*node=new Node(point,id);
		}
		else 
		{
			uint cd=depth%2;
			if (point[cd]<((*node)->point[cd]))
			{
				insert_helper(&((*node)->left),depth+1,point,id);
			}
			else 
			{
				insert_helper(&((*node)->right),depth+1,point,id);
			}

		}
		

	}

	void insert(std::vector<float> point, int id)
	{
		insert_helper(&root,0,point,id);

		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

	}
	void search_helper(std::vector<float> target, Node* node,uint depth,float distanceTol,std::vector<int>&ids)
{

	if (node !=NULL)
	{
		float A,B,C,D;
		A=(target[0]-distanceTol);
		B=(target[0]+distanceTol);
		C=(target[1]-distanceTol);
		D=(target[1]+distanceTol);
        //std::cout<<node->id<<" , "<<node->point[0]<<" , "<<A<<" ,"<<B<<" , "<<node->point[1]<<" , "<<C<<" , "<<D<<endl;
		if (((node->point[0]>=A)
		&&(node->point[0]<=B))
		&&((node->point[1]>=C)
		&&(node->point[1]<=D)))		
		{
			float x=(node->point[0]-target[0])*(node->point[0]-target[0]);
			float y=(node->point[1]-target[1])*(node->point[1]-target[1]);
			float distance =sqrt(x+y);
		    		 
	 		
	//		std::cout<<"distance"<<distance<<std::endl;
	//		std::cout<<"distanceTol"<<distanceTol<<std::endl;
			if (distance <= distanceTol)
			{
				
ids.push_back(node->id);

			}
			
		}
		if ((target[depth%2]-distanceTol)<node->point[depth%2] )
		{
			search_helper(target,node->left,depth+1,distanceTol,ids);
		}
		 if ((target[depth%2]+distanceTol) > node->point[depth%2])
		{
			search_helper(target,node->right,depth+1,distanceTol,ids);
		}

	}

}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(target,root,0,distanceTol,ids);
		return ids;
	}
	

};




