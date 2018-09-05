/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>

namespace AStar
{
    struct Vec3i
    {
		int x, y, z;

        bool operator == (const Vec3i& coordinates_);
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec3i, Vec3i)>;
    using CoordinateList = std::vector<Vec3i>;

    struct Node
    {
        uint G, H;
        Vec3i coordinates;
        Node *parent;

        Node(Vec3i coord_, Node *parent_ = nullptr);
        uint getScore();
    };

    using NodeSet = std::set<Node*>;

    class Generator
    {
        bool detectCollision(Vec3i coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, Vec3i coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void setWorldSize(Vec3i worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(Vec3i source_, Vec3i target_);
        void addCollision(Vec3i coordinates_);
        void removeCollision(Vec3i coordinates_);
        void clearCollisions();

		void setH_Min_H_Max(int H_Low, int H_High);
		//void setParkingAndEndPoint(std::vector<Vec3i>&Point);

		int H_Min = 0;
		int H_Max = worldSize.z - 1;
		//std::vector<Vec3i> Point_;
		
		//std::vector<std::vector<std::vector<bool>>> map;
		bool ***map;

    private:
        HeuristicFunction heuristic;
		CoordinateList direction, /*walls,*/ /*direction1,*/ direction2, direction3;
        Vec3i worldSize;
		uint directions, /*directions1,*/ directions2, directions3;
		
    };

    class Heuristic
    {
        static Vec3i getDelta(Vec3i source_, Vec3i target_);

    public:
        static uint manhattan(Vec3i source_, Vec3i target_);
        static uint euclidean(Vec3i source_, Vec3i target_);
        static uint octagonal(Vec3i source_, Vec3i target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__