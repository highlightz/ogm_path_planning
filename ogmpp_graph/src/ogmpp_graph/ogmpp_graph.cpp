#include "ogmpp_graph.hpp"

namespace ogmpp_graph
{

  /**
   * @brief Default constructor
   */
  Graph::Graph(float map_resolution):
    _resolution(map_resolution)
  {
    ros::NodeHandle nh;
    _visualization_enabled = false;
    _visualization_delay_ms = 0; 
    _node_visualization_size = 0;
    int rviz_delay_sec = 0;

    if(nh.hasParam("enable_visualization"))
    {
      nh.getParam("enable_visualization", _visualization_enabled);
    }
    if(nh.hasParam("visualization_delay_ms"))
    {
      nh.getParam("visualization_delay_ms", _visualization_delay_ms);
    }
    if(nh.hasParam("rviz_delay_sec"))
    {
      nh.getParam("rviz_delay_sec", rviz_delay_sec);
    }
    if(nh.hasParam("nodes_visualization_size"))
    {
      nh.getParam("nodes_visualization_size", _node_visualization_size);
    }

    ROS_INFO_STREAM( "Visualization: " << _visualization_enabled );
    ROS_INFO_STREAM( "Visualization delay: " << _visualization_delay_ms << " ms" );

    if(_visualization_enabled)
    {
      _visualization_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "visualization_marker_array", 0);
      ROS_INFO_STREAM ("Waiting for rviz to open");
      sleep(rviz_delay_sec);
      ROS_INFO_STREAM ("Graph initializing");
    }
  }

  /**
   * @brief Initializes the Graph with a root node
   * @param cell [Cell] The coordinates to create the first node
   */
  Graph::Graph(Cell cell, float map_resolution):
    _resolution(map_resolution)
  {
    Node *n = new Node(cell);
    _nodes.insert( std::pair<unsigned long, Node*>(n->getId(), n) );

    if(_visualization_enabled) visualize();
  }

  /** 
   * @brief Prints information about the whole graph
   */
  void Graph::print(void)
  {
    for ( nodes_it it = _nodes.begin() ; it != _nodes.end() ; it++)
    {
      ROS_INFO_STREAM( "------------" << std::endl );
      it->second->print();
    } 
  }

  /**
   * @brief Deallocates the allocated nodes and clears the graph
   */
  void Graph::clean(void)
  {
    for (nodes_it it = _nodes.begin() ; it != _nodes.end() ; it++)
    {
      delete it->second;
    }
    _nodes.clear();
  }

  /**
   * @brief Removes a cell from the graph. It also removes it from all its 
   * neighbors
   * @param cell [Cell] The cell to be removed
   */
  void Graph::removeNode(Cell cell)
  {
    for (nodes_it it = _nodes.begin() ; it != _nodes.end() ; it++)
    {
      it->second->removeNeighbor(cell);
    }
    _nodes.erase(Node::createCantorPairing(cell));

    if(_visualization_enabled) visualize();
  }

  /**
   * @brief Adds a node in the graph without assigning neighbors
   * @param c [Cell] The new node's coordinates
   * @return Node * : The new node
   */
  Node* Graph::addNode(Cell cell)
  {
    Node *n = new Node(cell);
    _nodes.insert( std::pair<unsigned long, Node*>(n->getId(), n) );

    if(_visualization_enabled) visualize();
  }

  /**
   * @brief Creates a neighboring relation between two cells. If one of them
   * or both do not exist as nodes, it creates them
   * @param cp [Cell] The first cell
   * @param cq [Cell] The second cell
   * @param first_is_parent [bool] True if the first is the parent of the second
   */
  void Graph::makeNeighbor(Cell cp, Cell cq, bool first_is_parent, 
    float weight, bool reverse_neighborhood)
  {
    Node *np, *nq;
    // Check if cp exists
    if (_nodes.find(Node::createCantorPairing(cp)) == _nodes.end())
    {
      np = addNode(cp);
    }
    else
    {
      np = _nodes[Node::createCantorPairing(cp)];
    }

    // Check if cp exists
    if (_nodes.find(Node::createCantorPairing(cp)) == _nodes.end())
    {
      nq = addNode(cp);
    }
    else
    {
      nq = _nodes[Node::createCantorPairing(cq)];
    }

    nq->addNeighbor(np, first_is_parent, weight, reverse_neighborhood);

    if(_visualization_enabled) visualize();
  }

  /**
   * @brief Returns a node from a specific cell (if the node exists)
   * @param cell [Cell] The node's cell
   * @return Node* : The node corresponding to the specific cell
   */
  Node* Graph::getNode(Cell cell)
  {
    unsigned long id = Node::createCantorPairing(cell);
    if (_nodes.find(id) == _nodes.end())
    {
      return NULL;
    }
    return _nodes[id];
  }

  /**
   * @brief Returns all nodes of the graph
   * @return std::map<unsigned long, Node*> : All nodes
   */
  std::map<unsigned long, Node*> Graph::getNodes(void)
  {
    return _nodes;
  }

  /**
   * @brief Function to visualize the graph in rviz
   */
  void Graph::visualize(void)
  {
    usleep(_visualization_delay_ms * 1000);

    // Visualize the nodes
    visualization_msgs::MarkerArray marr;
    for(nodes_it it = _nodes.begin() ; it != _nodes.end() ; it++)
    {
      visualization_msgs::Marker m;

      m.header.frame_id = "map";
      m.header.stamp = ros::Time();
      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::ADD;
      m.id = it->first;
      m.ns = "ogmpp_graph_ns";
      m.pose.position.x = it->second->getPose().x * _resolution;
      m.pose.position.y = it->second->getPose().y * _resolution;
      m.scale.x = _node_visualization_size;
      m.scale.y = _node_visualization_size;
      m.scale.z = _node_visualization_size;
      m.color.a = 1.0;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;

      marr.markers.push_back(m);
    }

    _visualization_pub.publish(marr);

    // Visualize the connections
    for(nodes_it it = _nodes.begin() ; it != _nodes.end() ; it++)
    {
      std::map<unsigned long, Node*> nmap = it->second->getNeighbors();
      for(nodes_it itn = nmap.begin() ; itn != nmap.end() ; itn++)
      {
        
      }
    }
  }
}
