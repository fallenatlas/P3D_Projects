#include "rayAccelerator.h"
#include "algorithm"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
			//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {
			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB world_bbox = AABB(min, max);

			for (Object* obj : objs) {
				AABB bbox = obj->GetBoundingBox();
				world_bbox.extend(bbox);
				objects.push_back(obj);
			}
			world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
			world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
			root->setAABB(world_bbox);
			nodes.push_back(root);
			//std::cout << "phase 1" << std::endl;
			build_recursive(0, objects.size(), root); // -> root node takes all the 
		}

int BVH::findSplitIndex(int dim, int left_index, int right_index, float split_value) {
	int split_index = left_index;

	for (int i = left_index; i < right_index; i++) {
		if (objects[i]->getCentroid().getAxisValue(dim) > split_value) {
			// add to left
			return split_index;
		}
		split_index++;
	}

	return split_index;
}

int BVH::findMedianSplitIndex(int left_index, int right_index) {
	int size = right_index - left_index;

	if (size % 2 == 0) {
		return left_index + (size / 2);
	}
	return left_index + (int ((float) size / 2.0F + 1.0F));
}

void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {
	   //PUT YOUR CODE HERE
	//std::cout << "left: " << left_index << " right: " << right_index << std::endl;
	if (right_index - left_index <= Threshold) {
		node->makeLeaf(left_index, right_index - left_index);
		//std::cout << "leaf" << std::endl;
	}
	else {
		// find longest axis && find mid point in axis
		int dim;

		AABB aabb = node->getAABB();
		Vector len = aabb.max - aabb.min;
		if (len.x >= len.y && len.x >= len.z) {
			dim = 0;
		}
		else if (len.y >= len.x && len.y >= len.z) {
			dim = 1;
		}
		else {
			dim = 2;
		}

		// sort them for the longest axis
		Comparator cmp = Comparator();
		cmp.dimension = dim;
		std::sort(objects.begin() + left_index, objects.begin() + right_index, cmp);

		// divide objects
		float split_value = (aabb.min.getAxisValue(dim) + aabb.max.getAxisValue(dim)) / 2.0F;
		int split_index = findSplitIndex(dim, left_index, right_index, split_value);

		// check if any empty
		if (split_index == left_index || split_index == right_index) {
			split_index = findMedianSplitIndex(left_index, right_index);
			//std::cout << "median: " << split_index << std::endl;
		}

		Vector min_left = Vector(FLT_MAX, FLT_MAX, FLT_MAX);
		Vector min_right = Vector(FLT_MAX, FLT_MAX, FLT_MAX);
		Vector max_left = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		Vector max_right = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		AABB left_bbox = AABB(min_left, max_left);
		AABB right_bbox = AABB(min_right, max_right);
		for (int i = left_index; i < split_index; i++) {
			AABB bbox = objects[i]->GetBoundingBox();
			left_bbox.extend(bbox);
		}
		for (int i = split_index; i < right_index; i++) {
			AABB bbox = objects[i]->GetBoundingBox();
			right_bbox.extend(bbox);
		}

		// 
		BVHNode* leftNode = new BVHNode();
		leftNode->setAABB(left_bbox);
		BVHNode* rightNode = new BVHNode();
		rightNode->setAABB(right_bbox);

		node->makeNode(nodes.size());

		nodes.push_back(leftNode);
		nodes.push_back(rightNode);

		build_recursive(left_index, split_index, leftNode);
		//std::cout << "left finished finished" << std::endl;
		build_recursive(split_index, right_index, rightNode);
		//std::cout << "right finished finished" << std::endl;
	}

		//right_index, left_index and split_index refer to the indices in the objects vector
	   // do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	    // node.index can have a index of objects vector or a index of nodes vector
			
		
	}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
			float tmp;
			float tmin = FLT_MAX;  //contains the closest primitive intersection
			bool hit = false;

			BVHNode* currentNode = nodes[0];

			//PUT YOUR CODE HERE

			if (!currentNode->getAABB().intercepts(ray, tmp)) {
				return false;
			}

			while (true) {
				if (!currentNode->isLeaf()) {
					float tl;
					float tr;
					BVHNode* left_node = nodes[currentNode->getIndex()];
					BVHNode* right_node = nodes[currentNode->getIndex() + 1];
					bool leftHit = left_node->getAABB().intercepts(ray, tl);
					bool rightHit = right_node->getAABB().intercepts(ray, tr);

					
					if (left_node->getAABB().isInside(ray.origin)) {
						tl = 0;
					}
					if (right_node->getAABB().isInside(ray.origin)) {
						tr = 0;
					}
					

					if (leftHit && rightHit) {
						if (tl <= tr) {
							StackItem item = StackItem(right_node, tr);
							hit_stack.push(item);
							currentNode = left_node;
						}
						else {
							StackItem item = StackItem(left_node, tl);
							hit_stack.push(item);
							currentNode = right_node;
						}
						continue;
					}
					else if (leftHit) {
						currentNode = left_node;
						continue;
					}
					else if (rightHit) {
						currentNode = right_node;
						continue;
					}
				}
				else { // is leaf
					int index = currentNode->getIndex();
					int numObjs = currentNode->getNObjs();
					for (int i = index; i < index + numObjs; i++) {
						if (objects[i]->intercepts(ray, tmp) && tmp < tmin) {
							tmin = tmp;
							*hit_obj = objects[i];
							//hit_point = ray.origin + ray.direction * tmin;
							hit = true;
						}
					}
				}

				bool newNode = false;
				while (!hit_stack.empty()) {
					StackItem item = hit_stack.top();
					hit_stack.pop();
					if (item.t < tmin) {
						//std::cout << "here " << item.t << " " << tmin << std::endl;
						currentNode = item.ptr;
						newNode = true;
						break;
					}
				}

				if (newNode) continue;
				
				if (hit_stack.empty()) {
					if (hit) {
						hit_point = ray.origin + ray.direction * tmin;
					}
					return hit;
				}
			}
			
			//return hit;
	}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
			float tmp;

			double length = ray.direction.length(); //distance between light and intersection point
			ray.direction.normalize();

			bool hit = false;
			BVHNode* currentNode = nodes[0];

			//PUT YOUR CODE HERE

			if (!currentNode->getAABB().intercepts(ray, tmp)) {
				return false;
			}

			while (true) {
				if (!currentNode->isLeaf()) {
					float tl;
					float tr;
					BVHNode* left_node = nodes[currentNode->getIndex()];
					BVHNode* right_node = nodes[currentNode->getIndex() + 1];
					bool leftHit = left_node->getAABB().intercepts(ray, tl);
					bool rightHit = right_node->getAABB().intercepts(ray, tr);

					
					if (left_node->getAABB().isInside(ray.origin)) {
						tl = 0;
					}
					if (right_node->getAABB().isInside(ray.origin)) {
						tr = 0;
					}

					if ((leftHit && tl <= length) && (rightHit && tr <= length)) {
						if (tl <= tr) {
							StackItem item = StackItem(right_node, tr);
							hit_stack.push(item);
							currentNode = left_node;
						}
						else {
							StackItem item = StackItem(left_node, tl);
							hit_stack.push(item);
							currentNode = right_node;
						}
						continue;
					}
					else if (leftHit && tl <= length) {
						currentNode = left_node;
						continue;
					}
					else if (rightHit && tr <= length) {
						currentNode = right_node;
						continue;
					}
				}
				else { // is leaf
					int index = currentNode->getIndex();
					int numObjs = currentNode->getNObjs();
					for (int i = index; i < index + numObjs; i++) {
						if (objects[i]->intercepts(ray, tmp) && tmp <= length) {
							return true;
						}
					}
				}

				bool newNode = false;
				while (!hit_stack.empty()) {
					StackItem item = hit_stack.top();
					hit_stack.pop();
					if (item.t <= length) {
						currentNode = item.ptr;
						newNode = true;
						break;
					}
				}

				if (newNode) continue;

				if (hit_stack.empty()) {
					return false;
				}
			}

			return false;
	}		
