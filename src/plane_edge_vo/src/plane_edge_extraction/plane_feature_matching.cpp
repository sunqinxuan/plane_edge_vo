/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-09-26 19:05
#
# Filename:		plane_feature_matching.cpp
#
# Description: 
#
===============================================*/
#include "plane_feature_matching.h"

namespace ulysses
{
	// construct interpretation tree from two plane set
	// each path down to the leaf represent a hypothesis
	// *************************************************
	// consider: the circumstances returning false
	bool InterpTree::Construct(std::vector<Plane*> &planes_ref_,std::vector<Plane*> &planes_cur_)
	{
//		std::cout<<"Construct: debug="<<debug<<std::endl;
		fp.open("plane_matching.txt",std::ios::app);
		if(debug)
			fp<<"**************************************************************"<<std::endl;
		// destruct the existing interpretation tree
		Release();
		root=new Node_InterpTree;
		Node_InterpTree *tmp_node, *insert_node, *node_attach, *interp_node2, *interp_node3;
		std::vector<Node_InterpTree*> nodes_tmp;
		bool flag_consistent=true;
		tmp_node=new Node_InterpTree();
		int interp_length_max=0, interp_length;
//		leaf_max_interp=root;
		//fp<<"nodes:"<<nodes.size()<<std::endl;
		if(debug)
		{
			fp<<"plane_cur:"<<planes_cur_.size()<<std::endl;
			for(int i=0;i<planes_cur_.size();i++)
			{
				fp<<planes_cur_[i]->index<<", "<<planes_cur_[i]->normal.transpose()<<", "<<planes_cur_[i]->d<<std::endl;
			}
			fp<<"plane_ref:"<<planes_ref_.size()<<std::endl;
			for(int i=0;i<planes_ref_.size();i++)
			{
				fp<<planes_ref_[i]->index<<", "<<planes_ref_[i]->normal.transpose()<<", "<<planes_ref_[i]->d<<std::endl;
			}
		}
		// for any two planes out of planes_cur_ and planes_ref_
		for(std::vector<Plane*>::iterator it_cur=planes_cur_.begin();it_cur!=planes_cur_.end();it_cur++)
		{
			for(std::vector<Plane*>::iterator it_ref=planes_ref_.begin();it_ref!=planes_ref_.end();it_ref++)
			{
				tmp_node->plane_cur=*it_cur;
				tmp_node->plane_ref=*it_ref;
				if(debug)
				{
					fp<<std::endl<<"now in: <"<<tmp_node->plane_cur->index<<","<<tmp_node->plane_ref->index<<"> "<<std::endl;
				}
				if(!consistent_1(tmp_node))
					continue;
				// it has to be ensured that the plane index 0 in planes_cur_ must have a correspondence in planes_ref_
				// and there consistent_1 must be satisfied
				// otherwise there will be no node insert to the root
				//if(it_cur==planes_cur_.begin())
				{
					insert_node=new Node_InterpTree(*it_ref,*it_cur);
					root->insertChild(insert_node);
					nodes.push_back(insert_node);
					if(debug)
					{
						fp<<"insert <"<<insert_node->plane_cur->index<<","<<insert_node->plane_ref->index<<"> to root"<<std::endl;
					}
					//continue;
				}
				if(debug)
				{
					fp<<"nodes in the tree: ";
					for(std::vector<Node_InterpTree*>::iterator it=nodes.begin();it!=nodes.end();it++)
					{
						fp<<"<"<<(*it)->plane_cur->index<<","<<(*it)->plane_ref->index<<">,";
					}
					fp<<std::endl;
				}
				for(int i_node=nodes.size()-1;i_node>=0;i_node--)
				{
					flag_consistent=true;
					if(debug)
					{
						fp<<"node: "<<nodes[i_node]->plane_cur->index<<","<<nodes[i_node]->plane_ref->index<<std::endl;
					}

					node_attach=nodes[i_node];
					interp_node2=node_attach;
					
					interp_length=0;
					while(interp_node2!=root)
					{
						if(!flag_consistent)
							break;
						//interp_node2=*it_interp;
						if(interp_node2->plane_ref==tmp_node->plane_ref)
						{
							flag_consistent=false;
							break;
						}
						if(!consistent_2(tmp_node,interp_node2))
						{
							flag_consistent=false;
							break;
						}
						else
						{
							interp_node3=node_attach;
							while(interp_node3!=root)
							//for(std::vector<Node_InterpTree*>::iterator ite_interp=Interp.begin();ite_interp!=Interp.end();ite_interp++)
							{
								if(interp_node3==interp_node2)
								{
									interp_node3=interp_node3->parent;
									continue;
								}
								//interp_node3=*ite_interp;
								if(!consistent_3(tmp_node,interp_node2,interp_node3))
								{
									flag_consistent=false;
									break;
								}
								interp_node3=interp_node3->parent;
							}
						}
						interp_length++;
						interp_node2=interp_node2->parent;
					}//end while interp_node2;
					if(flag_consistent)
					{
						if(debug)
						{
							fp<<"insert <"<<tmp_node->plane_cur->index<<","<<tmp_node->plane_ref->index<<"> to <"<<node_attach->plane_cur->index<<","<<node_attach->plane_ref->index<<">"<<std::endl;
						}
						insert_node=new Node_InterpTree(*it_ref,*it_cur);
						node_attach->insertChild(insert_node);
						nodes_tmp.push_back(insert_node);
						//Interp.push_back(insert_node);
						if(interp_length>interp_length_max)
						{
							interp_length_max=interp_length;
							leaf_max_interp.resize(1);
							leaf_max_interp[0]=insert_node;
						}
						else if(interp_length==interp_length_max)
						{
							leaf_max_interp.push_back(insert_node);
						}
					}
				}// end for i_node;
				for(int i_node=0;i_node<nodes_tmp.size();i_node++)
				{
					nodes.push_back(nodes_tmp[i_node]);
				}
				nodes_tmp.clear();
			}// end for it_ref;
		}// end for it_cur;
		if(leaf_max_interp.size()==0)
		{
			for(size_t i=0;i<nodes.size();i++)
			{
				leaf_max_interp.push_back(nodes[i]);
			}
		}
		if(debug)
		{
			fp<<"final interp:"<<std::endl;
			for(size_t i=0;i<leaf_max_interp.size();i++)
			{
				fp<<i<<" - ";
				node_attach=leaf_max_interp[i];
				while(node_attach!=root)
				{
					fp<<"<"<<node_attach->plane_cur->index<<","<<node_attach->plane_ref->index<<">,";
					node_attach=node_attach->parent;
				}
				fp<<std::endl;
			}
		}
		delete tmp_node;
		fp.close();
		return true;
	}

	bool InterpTree::isLeaf(Node_InterpTree *node)
	{
		if(node->parent!=NULL && node->children.size()==0)
			return true;
		else
			return false;
	}

	bool InterpTree::consistent_1(Node_InterpTree *node)
	{
		if(debug)
		{
			fp<<"\tconsistent_1_color: <"<<node->plane_cur->index<<","<<node->plane_ref->index<<"> "
									     <<node->plane_ref->similarity_color(node->plane_cur)<<std::endl;
			fp<<"\tconsistent_1_angle: <"<<node->plane_cur->index<<","<<node->plane_ref->index<<"> "
								   	     <<node->plane_ref->similarity_angle(node->plane_cur)<<std::endl;
			fp<<"\tconsistent_1_dist : <"<<node->plane_cur->index<<","<<node->plane_ref->index<<"> "
								   	     <<node->plane_ref->similarity_dist(node->plane_cur)<<std::endl;
		}
		if(node->plane_ref->similarity_color(node->plane_cur)<thres_color && 
		   node->plane_ref->similarity_angle(node->plane_cur)<thres_delta_angle &&
		   node->plane_ref->similarity_dist(node->plane_cur)<thres_delta_d)
			return true;
		else
			return false;
	}

	bool InterpTree::consistent_2(Node_InterpTree *node1, Node_InterpTree *node2)
	{
		double cos_angle_cur, cos_angle_ref, angle_cur, angle_ref, dist_cur, dist_ref;
		cos_angle_cur=node1->plane_cur->normal.transpose()*node2->plane_cur->normal;
		if(cos_angle_cur>0.9999)
			cos_angle_cur=0.9999;
		cos_angle_ref=node1->plane_ref->normal.transpose()*node2->plane_ref->normal;
		if(cos_angle_ref>0.9999)
			cos_angle_ref=0.9999;
		angle_cur=acos(cos_angle_cur);
		angle_ref=acos(cos_angle_ref);
		if(angle_cur<thres_delta_angle)
		{
			dist_cur=fabs(node1->plane_cur->d-node2->plane_cur->d);
		}
		else
		{
			dist_cur=0;
		}
		if(angle_ref<thres_delta_angle)
		{
			dist_ref=fabs(node1->plane_ref->d-node2->plane_ref->d);
		}
		else
		{
			dist_ref=0;
		}
		if(debug)
		{
			fp<<"\tconsistent_2_added: <"<<node1->plane_cur->index<<","<<node1->plane_ref->index<<"> "
								   <<", <"<<node2->plane_cur->index<<","<<node2->plane_ref->index<<"> - "
								   <<cos_angle_cur<<", "<<cos_angle_ref<<"; "
								   <<angle_cur<<", "<<angle_ref<<"; "<<dist_cur<<", "<<dist_ref<<std::endl;
			fp<<"\tconsistent_2: <"<<node1->plane_cur->index<<","<<node1->plane_ref->index<<"> "
								   <<", <"<<node2->plane_cur->index<<","<<node2->plane_ref->index<<"> - "
								   <<fabs(angle_cur-angle_ref)<<", "<<fabs(dist_cur-dist_ref)<<std::endl;
		}
		if(fabs(angle_cur-angle_ref)<thres_delta_angle && fabs(dist_cur-dist_ref)<thres_delta_d)
			return true;
		else
			return false;
	}

	bool InterpTree::consistent_3(Node_InterpTree *node1, Node_InterpTree *node2, Node_InterpTree *node3)
	{
		return true;
	}


	bool PlaneFeatureMatching::match(std::vector<Plane*> planes_ref,std::vector<Plane*> planes_cur, 
										std::vector<PlanePair> &matched_planes)
	{
		//fp<<"in the match:"<<interp_tree->getRoot()->children.size()<<std::endl;
		interp_tree->Construct(planes_ref, planes_cur);
		if(interp_tree->getMaxInterp()==0)
			return false;
		PlanePair tmp_plane_pair;
		Node_InterpTree *node_tmp=interp_tree->getMaxInterp();
		matched_planes.clear();
		while(node_tmp!=interp_tree->getRoot())
		{
			tmp_plane_pair.cur=node_tmp->plane_cur;
			tmp_plane_pair.ref=node_tmp->plane_ref;
			node_tmp=node_tmp->parent;
			matched_planes.push_back(tmp_plane_pair);
		}
		return true;
	}

	//void plane_feature_matching::depthFirstSearch()
	//{
	//	//fp<<"node num:"<<interp_tree->getNodeNum()<<std::endl;
	//	std::stack<Node_InterpTree*> nodeStack;
	//	nodeStack.push(interp_tree->getRoot());
	//	Node_InterpTree *node;
	//	fp<<"depthFirstSearch: ";
	//	while(!nodeStack.empty())
	//	{
	//		//fp<<"here"<<std::endl;
	//		node = nodeStack.top();
	//		nodeStack.pop();
	//		for(std::vector<Node_InterpTree*>::iterator it=node->children.begin();it!=node->children.end();++it)
	//		{
	//			nodeStack.push(*it);
	//		}
	//		if(node!=interp_tree->getRoot())
	//		{
	//			fp<<"<"<<node->plane_cur->index<<","<<node->plane_ref->index<<">("<<node->layer<<"); ";
	//		}
	//	}
	//	fp<<std::endl;
	//}

	//void plane_feature_matching::breadthFirstTravel()
	//{
	//	std::queue<Node_InterpTree*> nodeQueue;
	//	nodeQueue.push(interp_tree->getRoot());
	//	Node_InterpTree *node;
	//	fp<<"breadthFirstTravel:";
	//	while(!nodeQueue.empty())
	//	{
	//		node = nodeQueue.front();
	//		nodeQueue.pop();
	//		for(std::vector<Node_InterpTree*>::iterator it=node->children.begin();it!=node->children.end();++it)
	//		{
	//			nodeQueue.push(*it);
	//		}
	//		if(node!=interp_tree->getRoot())
	//		{
	//			fp<<"<"<<node->plane_cur->index<<","<<node->plane_ref->index<<">; ";
	//		}
	//	}
	//	fp<<std::endl;
	//}
}


