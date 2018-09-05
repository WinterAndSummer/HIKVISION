#include "AStar.hpp"
#include <algorithm>
#include <math.h>

using namespace std::placeholders;

bool AStar::Vec3i::operator == (const Vec3i& coordinates_)
{
	return (x == coordinates_.x && y == coordinates_.y&&z == coordinates_.z);
}

AStar::Vec3i operator + (const AStar::Vec3i& left_, const AStar::Vec3i& right_)
{
	return{ left_.x + right_.x, left_.y + right_.y, left_.z + right_.z };
}

AStar::Node::Node(Vec3i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
	direction = {
        { 0, 1, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { -1, 0, 0 },
		{ -1, -1, 0 }, { 1, 1, 0 }, { -1, 1, 0 }, { 1, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 }
    };
	/*direction1 = {
		{ 0, 1, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { -1, 0, 0 },
		{ -1, -1, 0 }, { 1, 1, 0 }, { -1, 1, 0 }, { 1, -1, 0 }, { 0, 0, 1 }
	};*/
	direction2 = {
		 { 0, 0, -1 }, { 0, 0, 1 }
	};
	direction3 = {
		{ 0, 1, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { -1, 0, 0 },
		{ -1, -1, 0 }, { 1, 1, 0 }, { -1, 1, 0 }, { 1, -1, 0 }, { 0, 0, -1 }
	};
}

void AStar::Generator::setWorldSize(Vec3i worldSize_)
{
    worldSize = worldSize_;
	map = new bool**[worldSize.x];
	for (int k1 = 0; k1 < worldSize.x; k1++)
	{
		map[k1] = new bool*[worldSize.y];
	}

	for (int k2 = 0; k2 < worldSize.x; k2++)
	{
		for (int k3 = 0; k3 < worldSize.y; k3++)
		{
			map[k2][k3] = new bool[worldSize.z];
		}
	}

	
	for (int i = 0; i < worldSize.x; i++)
	{
		for (int j = 0; j < worldSize.y; j++)
		{
			for (int k = 0; k < worldSize.z; k++)
			{
				map[i][j][k] = false;
			}
		}
	}
}

void AStar::Generator::setH_Min_H_Max(int H_Low, int H_High)
{
	H_Min = H_Low;
	H_Max = H_High;
}

//void AStar::Generator::setParkingAndEndPoint(std::vector<Vec3i>&Point)
//{
//	Point_ = Point;
//}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 10 : 6);
	//directions1 = (enable_ ? 9 : 5);
	directions2 = 2;
	directions3 = (enable_ ? 9 : 5);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec3i coordinates_)
{
	map[coordinates_.x][coordinates_.y][coordinates_.z] = true;
    //walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec3i coordinates_)
{
	map[coordinates_.x][coordinates_.y][coordinates_.z] = false;
    /*auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }*/
}

void AStar::Generator::clearCollisions()
{
	
	for (int i = 0; i < worldSize.x; i++){
		for (int j = 0; j < worldSize.y; j++){
			for (int k = 0; k < worldSize.z; k++){
				map[i][j][k] = false;
			}
			
		}
		
	}
    //walls.clear();
}

AStar::CoordinateList AStar::Generator::findPath(Vec3i source_, Vec3i target_)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    openSet.insert(new Node(source_));

	CoordinateList direct;
	uint directs;

    while (!openSet.empty()) {
        current = *openSet.begin();
        for (auto node : openSet) {
            if (node->getScore() <= current->getScore()) {
                current = node;
            }
        }

        if (current->coordinates == target_) {
            break;
        }

        closedSet.insert(current);
        openSet.erase(std::find(openSet.begin(), openSet.end(), current));

		
		if (H_Max > current->coordinates.z&&current->coordinates.z >= H_Min){
			direct = direction;
			directs = directions;
		}
		else if (current->coordinates.z < H_Min){
			direct = direction2;
			directs = directions2;
		}
		else if (current->coordinates.z == H_Max){
			direct = direction3;
			directs = directions3;
		}
		for (uint i = 0; i < directs; ++i) {
			Vec3i newCoordinates(current->coordinates + direct[i]);
			if (detectCollision(newCoordinates) ||
				findNodeOnList(closedSet, newCoordinates)) {
				continue;
			}

			uint totalCost = current->G + ((i < 4) ? 10 : 14);

			Node *successor = findNodeOnList(openSet, newCoordinates);
			if (successor == nullptr) {
				successor = new Node(newCoordinates, current);
				successor->G = totalCost;
				successor->H = heuristic(successor->coordinates, target_);
				openSet.insert(successor);
			}
			else if (totalCost < successor->G) {
				successor->parent = current;
				successor->G = totalCost;
			}
		}
    }

    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec3i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec3i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
		coordinates_.z < 0 || coordinates_.z >= worldSize.z ||
		map[coordinates_.x][coordinates_.y][coordinates_.z] == true
        /*std::find(walls.begin(), walls.end(), coordinates_) != walls.end()*/) {
        return true;
    }
    return false;
}

AStar::Vec3i AStar::Heuristic::getDelta(Vec3i source_, Vec3i target_)
{
	return{ abs(source_.x - target_.x), abs(source_.y - target_.y), abs(source_.z - target_.z) };
}

AStar::uint AStar::Heuristic::manhattan(Vec3i source_, Vec3i target_)
{
    auto delta = std::move(getDelta(source_, target_));
	return static_cast<uint>(14 * delta.x + 14*delta.y+10*delta.z);
}

AStar::uint AStar::Heuristic::euclidean(Vec3i source_, Vec3i target_)
{
    auto delta = std::move(getDelta(source_, target_));
	return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2) + pow(delta.z, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec3i source_, Vec3i target_)
{
    auto delta = std::move(getDelta(source_, target_));
	int temp = (-6) * std::min(delta.x, delta.y);

	return 10 * (delta.x + delta.y + delta.z) + (-6) * std::min(temp, delta.z);
}