//
// Created by Mike Huang on 2016/6/20.
//

#include <algorithm>
#include "Simplifier.h"

void Simplifier::load_obj(std::string filename) {
	LoadFromObj(filename.c_str());

	printf("Loading Vertices: \n");
	for (int i = 0; i < m_nVertices; ++i) {
//		double progress = (double)i / (double)(m_nVertices-1) * 100;
//		printf("%5.2f %%\r", progress);
		vertexes.push_back(Vertex(i, m_pVertexList[i]));
	}
	
	printf("\nLoading Triangles: \n");
	for (int i = 0; i < m_nTriangles; ++i) {
//		double progress = (double)i / (double)(m_nTriangles-1) * 100;
//		printf("%5.2f %%\r", progress);
		int v[3] = { m_pTriangleList[i][0],
			m_pTriangleList[i][1],
			m_pTriangleList[i][2] };
		for (int j = 0; j < 3; ++j) {
			addLink(v[j], v[(j + 1) % 3]);
			vertexes[v[j]].linkTriangles.push_back({ v[(j + 1) % 3], v[(j + 2) % 3] });
		}
		glm::dmat4 Kp = computeKp(vertexes[v[0]], vertexes[v[1]], vertexes[v[2]]);
		for (int j = 0; j < 3; ++j) {
			vertexes[v[j]].Q += Kp;
		}
	}
	printf("\n");
}

void Simplifier::simplify(double rate, double threshold) {
	// build minSet
	//if (!minSet.empty()) minSet.clear();
	while (!heap.empty()) heap.pop();
	printf("Building heap: \n");
	for (int i = 0; i < m_nVertices; ++i) {
//		double progress = (double)i / (double)(m_nVertices-1) * 100;
//		printf("%5.2f %%\r", progress);
		if (vertexes[i].isDeleted) continue;
		uniqueVector(vertexes[i].linkEdges);
		for (int j : vertexes[i].linkEdges) {
			if (i < j){
				addEdgeToHeap(i, j, threshold);
				//addEdgeInMinSet(i, j, threshold);
			}
		}
	}
	printf("\n");

	//int iter_n = (int)ceil(m_nVertices * rate);
	int target = (int)ceil(m_nTriangles * rate);
	int distance = m_nTriangles - target;
	printf("Simplifying: \n");
	//for (int iter = 0; iter < iter_n; ++iter) {
	while (m_nTriangles > target){
		//double progress = (double)iter / (double)(iter_n-1) * 100;
//		double progress = (double)(distance - (m_nTriangles - target)) / (double)(distance) * 100;
//		printf("%5.2f %%\r", progress);

		Edge deletedEdge;
		do {
			deletedEdge = heap.top();
			heap.pop();
			//deletedEdge = *minSet.begin();
			//minSet.erase(minSet.begin());
		} while (vertexes[deletedEdge.v1].isDeleted || vertexes[deletedEdge.v2].isDeleted ||
			abs(computeErrorAndShrinkVec(vertexes[deletedEdge.v1], vertexes[deletedEdge.v2]).first -deletedEdge.error) > 1e-8);

		for (int i : vertexes[deletedEdge.v2].linkEdges) {
			deleteLink(vertexes[i], deletedEdge.v2);
			if (i == deletedEdge.v1) continue;
			addLink(deletedEdge.v1, i);
			uniqueVector(vertexes[i].linkEdges);
		}
		uniqueVector(vertexes[deletedEdge.v1].linkEdges);

		for (auto tri : vertexes[deletedEdge.v2].linkTriangles) {
			deleteLink(vertexes[tri.first], pair<int, int>(tri.second, deletedEdge.v2));
			deleteLink(vertexes[tri.second], pair<int, int>(deletedEdge.v2, tri.first));
			m_nTriangles--;
			if (tri.first == deletedEdge.v1 || tri.second == deletedEdge.v1) continue;
			vertexes[deletedEdge.v1].linkTriangles.push_back(tri);
			vertexes[tri.first].linkTriangles.push_back(pair<int, int>(tri.second, deletedEdge.v1));
			vertexes[tri.second].linkTriangles.push_back(pair<int, int>(deletedEdge.v1, tri.first));
			m_nTriangles++;
		}

		vertexes[deletedEdge.v2].isDeleted = true;
		vertexes[deletedEdge.v1].data = deletedEdge.shrinkVec;
		updateVertexQ(deletedEdge.v1);

		vector<int> updateVertexes = vertexes[deletedEdge.v1].linkEdges;
		for (int i : updateVertexes) {
			updateVertexQ(i);
		}

		for (int i : updateVertexes) {
			for (int j : vertexes[i].linkEdges) {
				heap.push(Edge(i, j, computeErrorAndShrinkVec(vertexes[i], vertexes[j])));
				//minSet.insert(Edge(i, j, computeErrorAndShrinkVec(vertexes[i], vertexes[j])));
			}
		}
		m_nVertices--;
	}
	printf("\n");
}

void Simplifier::output(std::string filename) {
	FILE* fp = fopen(filename.c_str(), "w");
	if (fp == NULL)	{
		printf("Error: Failed opening %s to write\n", filename.c_str());
		return;
	}

	fprintf(fp, "# %d vertices\n", m_nVertices);
	fprintf(fp, "# %d triangles\n", m_nTriangles);
	int vIndex = 0, tIndex = 0;
	for (int i = 0; i < vertexes.size(); ++i) {
		if (vertexes[i].isDeleted) continue;
		fprintf(fp, "v %f %f %f\n", vertexes[i].data.x,
			vertexes[i].data.y,
			vertexes[i].data.z);
		vertexes[i].index = vIndex;
		vIndex++;
	}
	for (int i = 0; i < vertexes.size(); ++i) {
		if (vertexes[i].isDeleted) continue;
		for (auto tri : vertexes[i].linkTriangles) {
			if (i < tri.first && i < tri.second) {
				fprintf(fp, "f %d %d %d\n", vertexes[i].index + 1,
					vertexes[tri.first].index + 1,
					vertexes[tri.second].index + 1);
				tIndex++;
			}
		}
	}

	//fprintf(fp, "# %d vertices\n", vIndex);
	//fprintf(fp, "# %d triangles\n", tIndex);
	fclose(fp);
	printf("Writing to %s successfully\n", filename.c_str());
}

void Simplifier::uniqueVector(vector<int> &v) {
	std::sort(v.begin(), v.end());
	auto last = std::unique(v.begin(), v.end());
	v.erase(last, v.end());
}

glm::dmat4 Simplifier::computeKp(Vertex v1, Vertex v2, Vertex v3) {
	Vec3f v12 = (v2.data - v1.data), v23 = v3.data - v2.data;
	//v12.Normalize(); v23.Normalize();
	Vec3f normal = v12.cross(v23);
	normal.Normalize();
	glm::dvec4 p((double)normal.x, (double)normal.y, (double)normal.z, -(double)(v1.data.dot(normal)));
	return glm::dmat4(p.x * p, p.y * p, p.z * p, p.w * p);
}

pair<double, Vec3f> Simplifier::computeErrorAndShrinkVec(Vertex v1, Vertex v2) {
	glm::dmat4 Q = v1.Q + v2.Q, tmpQ = Q;
	tmpQ[3] = glm::dvec4(0., 0., 0., 1.);
	glm::dvec4 vec4;
	if (glm::determinant(tmpQ) == 0) {
		Vec3f tmpV = (v1.data + v2.data) / 2;
		vec4 = glm::dvec4(tmpV.x, tmpV.y, tmpV.z, 1);
	} else {
		vec4 = glm::transpose(glm::inverse(tmpQ))[3];
	}
	Vec3f v(float(vec4.x / vec4.w), float(vec4.y / vec4.w), float(vec4.z / vec4.w));
	double cost = glm::dot(vec4, glm::transpose(Q) * vec4);
	return{ -cost, v };
}

void Simplifier::deleteLink(Vertex &v, int edge) {
	vector<int>::iterator iter = std::find(v.linkEdges.begin(), v.linkEdges.end(), edge);
	if (iter == v.linkEdges.end())return;
	v.linkEdges.erase(iter);
}

void Simplifier::deleteLink(Vertex &v, pair<int, int> tri) {
	vector<pair<int, int> >::iterator iter = std::find(v.linkTriangles.begin(), v.linkTriangles.end(), tri);
	if (iter == v.linkTriangles.end()) {
		iter = std::find(v.linkTriangles.begin(), v.linkTriangles.end(), pair<int, int>(tri.second, tri.first));
	}
	v.linkTriangles.erase(iter);
}

void Simplifier::addEdgeInMinSet(int i, int j, double threshold) {
	if (sqrtf((vertexes[i].data - vertexes[j].data).L2Norm_Sqr()) > threshold) return;
	Edge edge(i, j, computeErrorAndShrinkVec(vertexes[i], vertexes[j]));
	minSet.insert(edge);
}

void Simplifier::addLink(int i, int j) {
	vertexes[i].linkEdges.push_back(j);
	vertexes[j].linkEdges.push_back(i);
}

void Simplifier::updateVertexQ(int i) {
	vertexes[i].Q = glm::dmat4(0);
	for (auto tri : vertexes[i].linkTriangles) {
		vertexes[i].Q += computeKp(vertexes[i], vertexes[tri.first], vertexes[tri.second]);
	}
}

void Simplifier::addEdgeToHeap(int i, int j, double threshold) {
	Edge edge(i, j, computeErrorAndShrinkVec(vertexes[i], vertexes[j]));
	heap.push(edge);
}
