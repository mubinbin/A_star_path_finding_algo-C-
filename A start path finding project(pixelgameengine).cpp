#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#define OLC_PGE_APPLICATION
#include"olcPixelGameEngine.h"

// #include "olcConsoleGameEngine.h"




/*
steps
1. create project's class
    a. project's name
    b. struct Node (since this algo is based on node struture)
    c. build console functions (1. OnUserCreate(), 2. OnUserUpdate(float fElapsedTime))

2. create nodes' grid 2D array and 1D array in OnUserCreate()

3. build nodes in OnUserUpdate(float fElapsedTime), including:
    a. NodeSize, NodeEdge, clear the screen, fill grid color first
    b. make mouse click event (obstacle), update fill grid color if node cells became an obstacle

4. make connections between node cells in OnUserCreate() via vecNeighbors vector

5. draw connection lines from cell to cell in OnUserUpdate(float fElapsedTime)
    a. normal nodes
    b. parent nodes

6. assign defaul values for start node and end node
    a. green for start node nad red for end node as special cases when fill color for all node cells

7. A star algo solver
    a. reset all node initial condition (parent = nullptr, bVisited = false, fGlobal = INT_MAX, fLocal = INT_MAX)
    b. use lambda function to setup heuristic (global distance between current node cell and start node cell)
    c. setup starting conditions (CurrentNode = StartNode, Current->fGlobal = heuristic(StartNode, EndNode), Current->fLocal = 0.0f)
    d.create a list for node cells not being examine, it has start node in it initially
    e. main A-star algo
*/


using namespace std;

class a_starpathfinding : public olc::PixelGameEngine
{
public:
    // constructor, give project name
    a_starpathfinding()
    {
       sAppName = "A_Start_Path_Algo";
    }

// Nodes for A star algo
private:
    struct sNode
    {
        bool bObstacle = false;     // is the node an obstruction?
        bool bVisited = false;      // have we search this node before?
        float fGobalGoal;           // distance to goal so far
        float fLocalGoal;          // distance between current node to end node
        int n;                      // nodes position in 2D space
        int m;
        vector<sNode*> vecNeighbors; // a vector of pointers to other nodes structer to represent the neighbors
        sNode* parent;             // pointer to parent node

    };

    // aesthetically pleasing
    int NodeSize = 16; // the pitch bewteen individual node
    int NodeEdge = 4; // how many pixels not to draw around the edge

    // pointer to nodes array (nodes grid)
    sNode* nodes = nullptr;
    int MapWidth = ScreenWidth() / NodeSize;
    int MapHeight = ScreenHeight()+1 / NodeSize;
    sNode* StartNode = nullptr;
    sNode* EndNode = nullptr;

//Pixel game engine required functions to start
private:
    bool OnUserCreate() override
    {
        // create a 2D array of nodes (nodes grid) with initial status
        nodes = new sNode [MapWidth * MapHeight];
        for (int x = 0; x < MapWidth; x++)
        {
            for (int y = 0; y < MapHeight; y++)
            {
                // example 2 by 4 grid 
                // order --> x,y coordinates
                // 0 1 2 3 --> {0,0} {0,1} {0,2} {0,3} 
                // 4 5 6 7 --> {1,0} {1,1} {1,2} {1,3}
                // nodes[6] = nodes [y * MapWidth + x] = nodes [1 * 4 + 2 ]
                nodes[y * MapWidth + x].n = x;
                nodes[y * MapWidth + x].m = y;
                nodes[y * MapWidth + x].parent = nullptr;
                nodes[y * MapWidth + x].bObstacle = false;
                nodes[y * MapWidth + x].bVisited = false;
            }
        }
        
        // make connections between node cells via vecNeighbors vector
        for (int x = 0; x < MapWidth; x++)
        {
            for (int y = 0; y < MapHeight; y++)
            {
                if (y > 0) // connect with the above node cell
                    nodes[y * MapWidth + x].vecNeighbors.push_back(&nodes[(y - 1) * MapWidth + (x + 0)]);
                if (y < MapWidth - 1) // connect with the below node cell
                    nodes[y * MapWidth + x].vecNeighbors.push_back(&nodes[(y + 1) * MapWidth + (x + 0)]);
                if (x > 0) // connect with the left node cell
                    nodes[y * MapWidth + x].vecNeighbors.push_back(&nodes[(y + 0) * MapWidth + (x - 1)]);
                if (x < MapHeight - 1) // connect with the right node cell
                    nodes[y * MapWidth + x].vecNeighbors.push_back(&nodes[(y + 0) * MapWidth + (x + 1)]);
                // connect diagonally
                if (y > 0 && x >0)
                    nodes[y * MapWidth + x].vecNeighbors.push_back(&nodes[(y - 1) * MapWidth + (x - 1)]);
                if (y < MapWidth -1 && x > 0)
                    nodes[y * MapWidth + x].vecNeighbors.push_back(&nodes[(y + 1) * MapWidth + (x - 1)]);
                if (x < MapWidth - 1 && y > 0)
                    nodes[y * MapWidth + x].vecNeighbors.push_back(&nodes[(y - 1) * MapWidth + (x + 1)]);
                if (y < MapWidth - 1 && x < MapWidth - 1)
                    nodes[y * MapWidth + x].vecNeighbors.push_back(&nodes[(y + 1) * MapWidth + (x + 1)]);
            }
        }

        // default values for start and end node cells
        EndNode = &nodes[11 * MapWidth + 15];
        StartNode = &nodes[1 * MapWidth + 0];

        return true;
    }

    // A-star algo
    void solver_astaralgo()
    {
        for (int x = 0; x < MapWidth; x++)
        {
            for (int y = 0; y < MapHeight; y++)
            {
                // set all node initial states
                nodes[y * MapWidth + x].parent = nullptr;
                nodes[y * MapWidth + x].bVisited = false;
                nodes[y * MapWidth + x].fGobalGoal = INT_MAX;
                nodes[y * MapWidth + x].fLocalGoal = INT_MAX;
            }
        }
        
        // use lambda function to setup heuristic (global distance between current node cell and start node cell)
        auto distance = [](sNode* a, sNode* b) // distance between two node cells
        {
            return sqrtf((a->n - b->n) * (a->n - b->n) + (a->m - b->m) * (a->m - b->m));
        };
        auto heuristic = [distance](sNode* a, sNode* b)
        {
            return distance(a, b);
        };  

        // setup starting conditions
        sNode* CurrentNode = StartNode; // start from start node cell
        StartNode->fGobalGoal = heuristic(EndNode, StartNode); // global distance between start node cell and end node cell
        StartNode->fLocalGoal = 0.0f; // local for start node is 0

        // create a list for node cells not being examine
        // it has start node in it initially
        list<sNode*> notExamnineNode;
        notExamnineNode.push_back(StartNode);

        // main A-star algo                // CurrentNode != EndNode will stop searching when reach the end, so that it wont search all nodes to find abstly shortest path, just relately short path
        while (!notExamnineNode.empty() && CurrentNode != EndNode)
        {
            // sort the list, so that the front node cell will be the next one to examine
            notExamnineNode.sort([](sNode* s, sNode* l) {return (s->fGobalGoal < l->fGobalGoal); });
            
            // remove the fornt node cell if it has been visited
            while (!notExamnineNode.empty() && notExamnineNode.front()->bVisited)
            {
                notExamnineNode.pop_front();
            }

            // if list is empty, stop
            if (notExamnineNode.empty())
                break;        
            
            // examine the front node of the list, and set it to be visited
            CurrentNode = notExamnineNode.front();
            CurrentNode->bVisited = true;

            // check all neighbors which have not been visited or not a block
            for (auto neighbor : CurrentNode->vecNeighbors)
            {
                if (!neighbor->bVisited && !neighbor->bObstacle)
                {
                    notExamnineNode.push_back(neighbor);
                }

                // temporary variable to store local goal between current node and neighbor node
                float possiblelowerlocal = CurrentNode->fLocalGoal + distance(CurrentNode, neighbor);

                // if the temporary local goal is less than that specific neighbor local goal,
                // set neighbor local goal to be the smaller one,
                // update that neighbor parent node to be current node, and global goal value
                if (possiblelowerlocal < neighbor->fLocalGoal)
                {
                    neighbor->fLocalGoal = possiblelowerlocal;
                    neighbor->parent = CurrentNode;
                    neighbor->fGobalGoal = neighbor->fLocalGoal + heuristic(neighbor, EndNode);
                } 
            }
        }
    }

    bool OnUserUpdate(float fElapsedTime) override
    {
        

        // use integer division to get cursor position in node map
        int MouseX = GetMouseX() / NodeSize;
        int MouseY = GetMouseY() / NodeSize;

        // left click mouse to draw obstacle
        if (GetMouse(0).bReleased)
        {
            // cursor is in the node map (grid)
            /*if (MouseX >= 0 && MouseX <= MapWidth)*/
            if (MouseX >= 0 && MouseX <= MapWidth)
            {
                if (MouseY >= 0 && MouseY < MapHeight)
                {
                    // hold shift + left click to set start node cell
                    if (GetKey(olc::Key::SHIFT).bHeld)
                    {
                        StartNode = &nodes[MouseY * MapWidth + MouseX];
                    }
                    // hold control + left click to set end node cell
                    else if (GetKey(olc::Key::CTRL).bHeld)
                    {
                        EndNode = &nodes[MouseY * MapWidth + MouseX];
                    }
                    else {
                        // certain node's bObstacle statuse is equal to what is was not before
                        nodes[MouseY * MapWidth + MouseX].bObstacle = !nodes[MouseY * MapWidth + MouseX].bObstacle;
                    }

        //            // begin the solver when clicking the mouse
                    solver_astaralgo();
                }
            }
        }


        // clear the screen first
        Clear(olc::BLACK);

        // draw node cell connections, it will be from cell center to neighbor cell center
        /*for (int x = 0; x < MapWidth; x++)
        {
            for (int y = 0; y < MapWidth; y++)
            {
                for (auto i : nodes[y * MapWidth + x].vecNeighbors)
                {
                    DrawLine(x * NodeSize + NodeSize / 2, y * NodeSize + NodeSize / 2, \
                        i->n * NodeSize + NodeSize / 2, i->m * NodeSize + NodeSize / 2, \
                        olc::DARK_BLUE);
                }
            }
        }*/

        // draw nodes on top (retangular to represent the nodes)    
        for (int x = 0; x < MapWidth; x++)
        {
            for (int y = 0; y < MapWidth; y++)
            {
                // if an obstacle were made (left click on a node), that node will be grey color, otherwise blue
                FillRect(x*NodeSize, y*NodeSize, \
                    NodeSize - NodeEdge, NodeSize - NodeEdge, olc::DARK_BLUE);

                // draw cell as brighter blue when cell has been visited
                if (nodes[y * MapWidth + x].bVisited)
                {
                    FillRect(x * NodeSize, y * NodeSize, \
                        NodeSize - NodeEdge, NodeSize - NodeEdge, \
                        olc::BLUE);
                }
                //draw cell grey when cell is a block
                if (nodes[y * MapWidth + x].bObstacle)
                {
                    FillRect(x * NodeSize, y * NodeSize, \
                        NodeSize - NodeEdge, NodeSize - NodeEdge, \
                        olc::GREY);
                }

                // draw green for StartNode and Red for EndNode
                if (&nodes[y * MapWidth + x] == StartNode)
                {
                    FillRect(x * NodeSize, y * NodeSize, \
                        NodeSize - NodeEdge, NodeSize - NodeEdge,\
                        olc::GREEN);
                }
                if (&nodes[y * MapWidth + x] == EndNode)
                {
                    FillRect(x * NodeSize, y * NodeSize, \
                        NodeSize - NodeEdge, NodeSize - NodeEdge,\
                        olc::RED);
                }
            }
        }

        // if there is an end node cell, starting from the end node cell, when there is a parent node,
        // draw yellow line to connect them
       if (EndNode != nullptr)
       {
            sNode* temp = EndNode;
            while (temp->parent != nullptr) // at start node cell, there will be no parent node
            {
                DrawLine(temp->n * NodeSize + NodeSize / 2, temp->m * NodeSize + NodeSize / 2, \
                    temp->parent->n * NodeSize + NodeSize / 2, temp->parent->m * NodeSize + NodeSize / 2, \
                    olc::YELLOW);

                temp = temp->parent; // move to the node cell 1 lvl before (parent node's parent node)
            }
       }

        return true;
    }   
};

int main()
{   
    a_starpathfinding game;
    game.Construct(320, 320, 4, 4);
    game.Start();
    return 0;
 }