
#include "Trailblazer.h"
#include "queue.h"
#include "pqueue.h"
#include "set.h"
using namespace std;

static const double SUFFICIENT_DIFFERENCE = 0.2;

Path breadthFirstSearch(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
    Path path;
    path.add(start);
    if(start==end){
        return path;
    }
    Queue<Path> queue;
    queue.add(path);
    RoadNode* currentNode;
    Set<RoadNode*> visited;
    Path currentPath;
    Path newPath;
    Set<RoadNode*> neighbours;

    start->setColor(Color::YELLOW);
    while(!queue.isEmpty()){
        currentPath=queue.dequeue();
        currentNode=currentPath.get(currentPath.size()-1);
        neighbours=graph.neighborsOf(currentNode);

        for (RoadNode* node:neighbours){
            newPath=currentPath;
            newPath.add(node);
            if(node==end){
                node->setColor(Color::YELLOW);
                return newPath;
            }else if (!visited.contains(node)){
                node->setColor(Color::YELLOW);
                queue.enqueue(newPath);
            }
        }
        visited.add(currentNode);
        currentNode->setColor(Color::GREEN);
    }
    path.clear();
    return path;
}

Path dijkstrasAlgorithm(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
    Path path;
    path.add(start);
    if(start==end){
        return path;
    }
    PriorityQueue<Path> pQueue;
    pQueue.add(path,0);
    RoadNode* currentNode;
    Set<RoadNode*> visited;
    Path currentPath;
    Path newPath;
    Set<RoadNode*> neighbours;
    double currentCost;
    double totalCost;
    double minCost=-1;
    RoadEdge* edge;
    start->setColor(Color::YELLOW);
    path.clear();
    while(!pQueue.isEmpty()){
        currentCost=pQueue.peekPriority();

        if(!path.isEmpty()){
            if(currentCost>minCost){
                break;
            }
        }

        currentPath=pQueue.dequeue();
        currentNode=currentPath.get(currentPath.size()-1);
        neighbours=graph.neighborsOf(currentNode);
        for (RoadNode* node:neighbours){
            edge=graph.edgeBetween(currentNode,node);
            totalCost=edge->cost()+currentCost;
            newPath=currentPath;
            newPath.add(node);
            if(path.isEmpty() || totalCost<minCost){
                if(node==end){
                    node->setColor(Color::YELLOW);
                    path=newPath;
                    minCost=totalCost;
                }else if(!visited.contains(node)){
                    node->setColor(Color::YELLOW);
                    pQueue.add(newPath,totalCost);
                }
            }
        }
        visited.add(currentNode);
        currentNode->setColor(Color::GREEN);
    }
    return path;
}

double restrictedAStar(const RoadGraph& graph, RoadNode* start, RoadNode* end,
                       RoadEdge* ignoredEdge,Path& path){
    path.clear();
    path.add(start);
    if(start==end){
        return 0;
    }
    PriorityQueue<Path> pQueue;
    pQueue.add(path,0);
    RoadNode* currentNode;
    Set<RoadNode*> visited;
    Path currentPath;
    Path newPath;
    Set<RoadNode*> neighbours;
    double currentCost;
    double newCost;
    double heuristicCost;
    double totalCost;
    double minCost=-1;
    RoadEdge* newEdge;

    start->setColor(Color::YELLOW);
    path.clear();
    while(!pQueue.isEmpty()){
        currentCost=pQueue.peekPriority();
        currentPath=pQueue.dequeue();
        currentNode=currentPath.get(currentPath.size()-1);
        if(currentCost!=0 && currentNode!=end){
            currentCost-=graph.crowFlyDistanceBetween(currentNode,end)/graph.maxRoadSpeed();
        }

        if(!path.isEmpty()){
            if(currentCost>minCost){
                continue;
            }
        }

        neighbours=graph.neighborsOf(currentNode);
        for (RoadNode* node:neighbours){
            newEdge=graph.edgeBetween(currentNode,node);
            if(newEdge==ignoredEdge){
                continue;
            }
            newCost=newEdge->cost();
            if(node==end){
                heuristicCost=0;
            }else{
                heuristicCost=graph.crowFlyDistanceBetween(node,end)/graph.maxRoadSpeed();
            }

            totalCost=newCost+heuristicCost+currentCost;
            newPath=currentPath;
            newPath.add(node);

            if(path.isEmpty() || totalCost<minCost){
                if(node==end){
                    node->setColor(Color::YELLOW);
                    path=newPath;
                    minCost=totalCost;
                }else if(!visited.contains(node)){
                    node->setColor(Color::YELLOW);
                    pQueue.add(newPath,totalCost);

                }
            }

        }
        visited.add(currentNode);
        currentNode->setColor(Color::GREEN);
    }
    return minCost;
}

Path aStar(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
    Path path;
    restrictedAStar(graph,start,end,NULL,path);
    return path;
}

Set<RoadNode*> pathToSet(Path& path){
    Set<RoadNode*> pathSet;
    for(RoadNode* node :path){
        pathSet.add(node);
    }
    return pathSet;
}

Path alternativeRoute(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
    double minCost=-1;
    double cost;
    Path shortest = aStar(graph, start, end);
    Set<RoadNode*> minPathSet=pathToSet(shortest);
    Set<RoadNode*> altPathSet;
    RoadEdge* ignoredEdge;
    Path altPath;
    double ratio;
    string nameTest;
    for(int i=0;i<shortest.size()-1;i++){
        ignoredEdge=graph.edgeBetween(shortest[i],shortest[i+1]);
        cost=restrictedAStar(graph,start,end,ignoredEdge,altPath);
        altPathSet=pathToSet(altPath);
        altPathSet=altPathSet-altPathSet*minPathSet;
        for(RoadNode* node:altPathSet){
            nameTest=node->nodeName();
        }
        ratio=altPathSet.size()*1.0/minPathSet.size();
        if(minCost==-1 || (minCost!=-1 && cost<minCost)){
            if(ratio>SUFFICIENT_DIFFERENCE){
                minCost=cost;
            }
        }
    }

    if(minCost==-1){
        altPath.clear();
    }
    return altPath;
}

