//
// Created by Mike Huang on 2016/6/20.
//

#ifndef MESH_SIMPLIFICATION3_SIMPLIFIER_H
#define MESH_SIMPLIFICATION3_SIMPLIFIER_H


#include "SimpleObject.h"
#include "glm/glm.hpp"

#include <vector>
#include <string>
#include <set>
#include <queue>

#define EPS 1e-20

using std::set;

using std::vector;
using std::pair;
using std::priority_queue;
using SimpleOBJ::Vec3f;

struct Vertex {
	Vertex(int index = 0, Vec3f data = Vec3f(0.f, 0.f, 0.f))
		: index(index), data(data), isDeleted(false), Q(glm::dmat4(0)) {}
	int index;
	bool isDeleted;
	Vec3f data;
	vector<int> linkEdges;
	vector<pair<int, int> > linkTriangles;

	glm::dmat4 Q;
};

struct Edge {
	Edge(int v1 = -1, int v2 = -1, pair<double, Vec3f> pair1 = { 0, Vec3f() })
		: v1(v1), v2(v2), error(pair1.first), shrinkVec(pair1.second) {}
	int v1, v2;
	double error;
	Vec3f shrinkVec;

	bool operator< (const Edge& e) const {
	/*	if (error < e.error) return true;
		else if (e.error < error) return false;
		else if (v1 < e.v1) return true;
		else if (e.v1 < v1) return false;
		else if (v2 < e.v2) return true;
		else return false;*/
		//return error - e.error < -EPS;
		return error < e.error;
	}
};

class Simplifier : SimpleOBJ::CSimpleObject {
	vector<Vertex> vertexes;
	set<Edge> minSet;
	priority_queue<Edge> heap;

	glm::dmat4 computeKp(Vertex v1, Vertex v2, Vertex v3);
	pair<double, Vec3f> computeErrorAndShrinkVec(Vertex v1, Vertex v2);

	void uniqueVector(vector<int>& v);
	void deleteLink(Vertex& v, int edge);
	void deleteLink(Vertex& v, pair<int, int> tri);
	void addLink(int i, int j);

	void addEdgeInMinSet(int i, int j, double threshold);
	void addEdgeToHeap(int i, int j, double threshold);
	void updateVertexQ(int i);
	double computeCost(glm::dvec4, glm::dmat4);

public:
	void load_obj(std::string);
	void simplify(double rate, double threshold);
	void output(std::string);
};


#endif //MESH_SIMPLIFICATION3_SIMPLIFIER_H
