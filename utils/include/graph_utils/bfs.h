#include <list>
#include <bits/stdc++.h>

template <typename T>
uint32_t
bfs(T const& graph, 
    uint32_t start, 
    uint32_t end,
    uint32_t* parent)
{
  // vector with the distances to each node from start
  uint32_t distance[graph.size()];

  // visited nodes, ie expanded
  bool visited[graph.size()];

  // init values
  for(int i = 0; i < graph.size(); i++)
  {
    visited[i] = false;
    distance[i] = std::numeric_limits<uint32_t>::infinity();
    parent[i] = -1;
  }

  distance[start] = 0;
  std::list<uint32_t> queue;
  queue.push_back(start);

  bool found = false;
  while(queue.empty() == false && found == false)
  {
    uint32_t vertex = queue.front();
    queue.pop_front();

    auto neighbors = graph.neighbors(vertex);
    for(auto neighbor : neighbors)
    {
      if(visited[neighbor] == false) 
      {
        visited[neighbor] = true;
        distance[neighbor] = distance[vertex] + 1;
        parent[neighbor] = vertex;
        
        if(end == neighbor)
        {
          found = true;
          break;
        }

        queue.push_back(neighbor);
      }
    }
  }
  return distance[end];
}
