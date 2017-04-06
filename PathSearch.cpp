#include "PathSearch.h"
#include <iostream>

namespace fullsail_ai { namespace algorithms {

	PathSearch::Edge::Edge(SearchNode * _node, float _cost)
	{
		SNode = _node;
		Cost = _cost;
	}

	PathSearch::Edge::~Edge()
	{
		
	}

	bool isGreater(PathSearch::PlannerNode* const &lhs, PathSearch::PlannerNode* const &rhs)
	{
		//return (lhs->HeuristicCost > rhs->HeuristicCost);
		return (lhs->Cost > rhs->Cost);
		//return true;
	}

	double estimate(Tile * start, Tile * End)
	{
		double X1 = (double)start->getXCoordinate();
		double Y1 = (double)start->getYCoordinate();

		double X2 = (double)End->getXCoordinate();
		double Y2 = (double)End->getYCoordinate();

		double X2X1 = ((X2 - X1) * (X2 - X1));
		double Y2Y1 = ((Y2 - Y1) * (Y2 - Y1));

		return sqrt(X2X1 + Y2Y1);
	}

	PathSearch::PathSearch() : open(isGreater)
	{
		isFinished = false;
	}
	
	PathSearch::~PathSearch()
	{

	}

	//Checks if AdjacentTile is adjacent to the the CurrentTile
	bool PathSearch::CheckIfAdjacent(Tile * CurrentTile, Tile * AdjacentTile)
	{
		int CurrRow = CurrentTile->getRow();
		int CurrColumn = CurrentTile->getColumn();

		int AdjRow = AdjacentTile->getRow();
		int AdjColumn = AdjacentTile->getColumn();

		if (CurrentTile->getRow() % 2 == 0)
		{
			if (AdjRow == CurrRow - 1 && AdjColumn == CurrColumn)
				return true; //UP
			else if (AdjRow == CurrRow + 1 && AdjColumn == CurrColumn)
				return true; //Down
			else if (AdjRow == CurrRow && AdjColumn == CurrColumn - 1)
				return true; //Left
			else if (AdjRow == CurrRow && AdjColumn == CurrColumn + 1)
				return true; //Right
			else if (AdjRow == CurrRow - 1 && AdjColumn == CurrColumn - 1)
				return true; //Up,Left								  
			else if (AdjRow == CurrRow + 1 && AdjColumn == CurrColumn - 1)
				return true; //Down,Left

			return false;
		}
		else
		{
			if (AdjRow == CurrRow - 1 && AdjColumn == CurrColumn)
				return true; //UP
			else if (AdjRow == CurrRow + 1 && AdjColumn == CurrColumn)
				return true; //Down
			else if (AdjRow == CurrRow && AdjColumn == CurrColumn - 1)
				return true; //Left
			else if (AdjRow == CurrRow && AdjColumn == CurrColumn + 1)
				return true; //Right
			else if (AdjRow == CurrRow - 1 && AdjColumn == CurrColumn + 1)
				return true; //Up,Right								  
			else if (AdjRow == CurrRow + 1 && AdjColumn == CurrColumn + 1)
				return true; //Down,Right

			return false;
		}
	}

	void PathSearch::initialize(TileMap* _tileMap)
	{
		MyTileMap = _tileMap;
		std::map<Tile*, Edge::SearchNode*>::iterator MapGraphIter;
		Tile * TempTile;

		int ColumnNum = MyTileMap->getColumnCount();
		int RowNum = MyTileMap->getRowCount();

		for (int i = 0; i < ColumnNum; i++)
		{
			for (int j = 0; j < RowNum; j++)
			{
				//create search nodes for each valid spot

				//create searchNode
				Edge::SearchNode * EdgeSearchNode = new Edge::SearchNode();
				//Set tile
				EdgeSearchNode->tile = MyTileMap->getTile(j, i);

				//Add to Search node container
				MyMapGraph[EdgeSearchNode->tile] = EdgeSearchNode;

			}
		}

		//iterate through the tiles, checking for adjacent tiles and assigning weights to those adjacent tiles
		for (MapGraphIter = MyMapGraph.begin(); MapGraphIter != MyMapGraph.end(); MapGraphIter++)
		{
			//nested for loops from -1 -> 2 in order to check all around the current tile
			for (int row = -1; row < 2; row++)
			{
				for (int column = -1; column < 2; column++)
				{
					TempTile = MyTileMap->getTile(MapGraphIter->second->tile->getRow() + (row), MapGraphIter->second->tile->getColumn() + (column));

					if (TempTile != NULL)
					{
						//If TempTile is adjacent to MapGraphIter->second->tile
						//add that tile to the edges list and assign that tile a weight.
						if (CheckIfAdjacent(TempTile, MapGraphIter->second->tile))
						{
							Edge * myEdge = new Edge(MyMapGraph[TempTile], estimate(TempTile, MapGraphIter->second->tile) * TempTile->getWeight());
							MapGraphIter->second->myEdges.push_back(myEdge);
						}
					}
				}
			}
		}
	}

	void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
	{
		isFinished = false;
		PlannerNode* StartNode = new PlannerNode();
		std::map<Tile*, Edge::SearchNode*>::iterator MapSearchIter;
		
		for (MapSearchIter = MyMapGraph.begin(); MapSearchIter != MyMapGraph.end(); MapSearchIter++)
		{
			if (MapSearchIter->second->tile->getRow() == startRow
				&& MapSearchIter->second->tile->getColumn() == startColumn)
			{
				StartNode->vertex = MapSearchIter->second;
				break;
			}
		}

		for (MapSearchIter = MyMapGraph.begin(); MapSearchIter != MyMapGraph.end(); MapSearchIter++)
		{
			if (MapSearchIter->second->tile->getRow() == goalRow
				&& MapSearchIter->second->tile->getColumn() == goalColumn)
			{
				GoalTile = MapSearchIter->second->tile;
				break;
			}
		}

		CurrentPath.clear();
		open.push(StartNode);
		Visited[StartNode->vertex] = open.front();

		open.front()->GivenCost = 0;
		open.front()->HeuristicCost = estimate(StartNode->vertex->tile, GoalTile);
		open.front()->Cost = open.front()->HeuristicCost * 1.0f;
	}

	void PathSearch::update(long timeslice)
	{
		do
		{
			//get the front and pop it off
			PlannerNode * current = open.front();
			open.pop();
		
			//if we made it to the goal
			if (current->vertex->tile == GoalTile)
			{
				isFinished = true;

				while (current != nullptr)
				{
					CurrentPath.push_back(current->vertex->tile);
					//moving back along the path
					current = current->Parent;
				}

				//std::cout << CurrentPath.size() << "\n";
				exit();
				return; //we're done
			}
		
			//for each (Edge * var in current->vertex)
			std::vector<Edge*>::iterator EdgesIter;
			for (EdgesIter = current->vertex->myEdges.begin(); EdgesIter != current->vertex->myEdges.end(); EdgesIter++)
			{
				Edge::SearchNode* successor = (*EdgesIter)->SNode;
				float TempCost = current->GivenCost + (*EdgesIter)->Cost;

				if (successor->tile->getWeight() == 0)
					continue;
			        
				//If the current search node has been visited
				//check it's cost, compare the cost of the visited node to the cost
				//of the current tile plus the curent edge. If it's less, then update the 
				//planner node and add it to the list.
				if (Visited[successor] != NULL)
				{
					PlannerNode* node = Visited[successor];

					if (TempCost < node->GivenCost)
					{
						open.remove(node);
						node->GivenCost = TempCost;
						node->Cost = node->GivenCost + node->HeuristicCost * 1.0f;
						node->Parent = current;
						open.push(node);
					}
				}
				else
				{
					PlannerNode* node = new PlannerNode();
					node->vertex = successor;
					node->GivenCost = TempCost;
					node->HeuristicCost = estimate(successor->tile, GoalTile);
					node->Cost = node->GivenCost + node->HeuristicCost * 1.0f;
					node->Parent = current;
					node->vertex->tile->setFill(0xFF0000FF);
					Visited[successor] = node;
					open.push(node);
				}
			}

			if (timeslice == 0)
				break;

		} while (!open.empty());
	}

	void PathSearch::exit()
	{
		while (!open.empty())
				open.pop();

		std::unordered_map<Edge::SearchNode*, PlannerNode*>::iterator VisitedTileIter;
		for (VisitedTileIter = Visited.begin(); VisitedTileIter != Visited.end(); VisitedTileIter++)
		{
			delete VisitedTileIter->second;
		}

		Visited.clear();
		GoalTile = nullptr;
	}

	void PathSearch::shutdown()
	{
		exit();

		std::map<Tile*, Edge::SearchNode*>::iterator MapGraphIter;
		for (MapGraphIter = MyMapGraph.begin(); MapGraphIter != MyMapGraph.end(); MapGraphIter++)
		{
			for (size_t i = 0; i < MapGraphIter->second->myEdges.size(); i++)
			{
				delete MapGraphIter->second->myEdges[i];
			}

			delete MapGraphIter->second;
		}

		MyMapGraph.clear();

		open.clear();
		CurrentPath.clear();
	}

	bool PathSearch::isDone() const
	{
		return isFinished;
	}

	std::vector<Tile const*> const PathSearch::getSolution() const
	{
		std::vector<Tile const*> temp;

		for (unsigned int i = 0; i < CurrentPath.size(); i++)
			temp.push_back(CurrentPath[i]);

		return temp;
	}
}}  // namespace fullsail_ai::algorithms

