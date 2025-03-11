using System.Collections.Generic;
using UnityEngine;

// The AStarPathfinder class implements the A* algorithm on a grid-based map.
public class AStarPathfinder : MonoBehaviour
{
    // Transforms used to define start and target positions
    public Transform startTransform;
    public Transform targetTransform;

    // Layer used to determine obstacles (non-walkable nodes)
    public LayerMask obstacleLayer;

    // Grid parameters: overall size and node dimensions.
    public Vector2 gridWorldSize;
    public float nodeRadius;
    private Node[,] grid;

    float nodeDiameter;
    int gridSizeX, gridSizeY;

    // For visualization of the path (optional)
    private List<Node> path;

    void Start()
    {
        nodeDiameter = nodeRadius * 2;
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);
        CreateGrid();
    }

    // Build the grid of nodes based on the world size and node dimensions.
    void CreateGrid()
    {
        grid = new Node[gridSizeX, gridSizeY];
        // Calculate bottom-left point of the grid relative to the GameObject's position.
        Vector3 worldBottomLeft = transform.position - Vector3.right * gridWorldSize.x / 2 - Vector3.forward * gridWorldSize.y / 2;

        // Loop through each cell in the grid and determine its world position and walkability.
        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                Vector3 worldPoint = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) +
                                     Vector3.forward * (y * nodeDiameter + nodeRadius);
                // Use a sphere check to mark obstacles.
                bool walkable = !(Physics.CheckSphere(worldPoint, nodeRadius, obstacleLayer));
                grid[x, y] = new Node(walkable, worldPoint, x, y);
            }
        }
    }

    // Main A* search method. Returns a list of nodes representing the path.
    public List<Node> FindPath(Vector3 startPos, Vector3 targetPos)
    {
        Node startNode = NodeFromWorldPoint(startPos);
        Node targetNode = NodeFromWorldPoint(targetPos);

        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            // Find the node with the lowest fCost in the open set.
            Node currentNode = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < currentNode.fCost ||
                   (openSet[i].fCost == currentNode.fCost && openSet[i].hCost < currentNode.hCost))
                {
                    currentNode = openSet[i];
                }
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            // If we've reached the target node, retrace the path.
            if (currentNode == targetNode)
            {
                return RetracePath(startNode, targetNode);
            }

            // Evaluate each neighbor of the current node.
            foreach (Node neighbor in GetNeighbors(currentNode))
            {
                if (!neighbor.walkable || closedSet.Contains(neighbor))
                    continue;

                // Calculate tentative cost from start node to neighbor.
                int newMovementCostToNeighbor = currentNode.gCost + GetDistance(currentNode, neighbor);
                if (newMovementCostToNeighbor < neighbor.gCost || !openSet.Contains(neighbor))
                {
                    neighbor.gCost = newMovementCostToNeighbor;
                    neighbor.hCost = GetDistance(neighbor, targetNode);
                    neighbor.parent = currentNode;

                    if (!openSet.Contains(neighbor))
                        openSet.Add(neighbor);
                }
            }
        }
        // Return null if no path is found.
        return null;
    }

    // Retrace the path from end to start by following parent links.
    List<Node> RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;
        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        path.Reverse();
        return path;
    }

    // Convert a world position to a corresponding node in the grid.
    Node NodeFromWorldPoint(Vector3 worldPosition)
    {
        float percentX = (worldPosition.x + gridWorldSize.x / 2) / gridWorldSize.x;
        float percentY = (worldPosition.z + gridWorldSize.y / 2) / gridWorldSize.y;
        percentX = Mathf.Clamp01(percentX);
        percentY = Mathf.Clamp01(percentY);

        int x = Mathf.RoundToInt((gridSizeX - 1) * percentX);
        int y = Mathf.RoundToInt((gridSizeY - 1) * percentY);
        return grid[x, y];
    }

    // Retrieve all valid neighboring nodes (including diagonals).
    List<Node> GetNeighbors(Node node)
    {
        List<Node> neighbors = new List<Node>();

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                // Skip the current node.
                if (x == 0 && y == 0)
                    continue;

                int checkX = node.gridX + x;
                int checkY = node.gridY + y;

                if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
                {
                    neighbors.Add(grid[checkX, checkY]);
                }
            }
        }

        return neighbors;
    }

    // Compute the distance between two nodes.
    // Uses 10 for straight moves and 14 for diagonal moves.
    int GetDistance(Node nodeA, Node nodeB)
    {
        int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        if (dstX > dstY)
            return 14 * dstY + 10 * (dstX - dstY);
        return 14 * dstX + 10 * (dstY - dstX);
    }

    // Optional: Draw the grid and path in the Unity Editor for debugging.
    void OnDrawGizmos()
    {
        Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x, 1, gridWorldSize.y));

        if (grid != null)
        {
            foreach (Node n in grid)
            {
                Gizmos.color = (n.walkable) ? Color.white : Color.red;
                if (path != null && path.Contains(n))
                {
                    Gizmos.color = Color.black;
                }
                Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - 0.1f));
            }
        }
    }

    // For testing: Update the path each frame if start and target are assigned.
    void Update()
    {
        if (startTransform != null && targetTransform != null)
        {
            path = FindPath(startTransform.position, targetTransform.position);
        }
    }
}

//// The Node class represents a single cell in the grid.
//public class Node
//{
//    public bool walkable;
//    public Vector3 worldPosition;
//    public int gridX;
//    public int gridY;

//    // Cost from the start node to this node.
//    public int gCost = int.MaxValue;
//    // Heuristic cost estimate to the target node.
//    public int hCost;
//    // Reference to the parent node (used to retrace the path).
//    public Node parent;

//    public Node(bool _walkable, Vector3 _worldPos, int _gridX, int _gridY)
//    {
//        walkable = _walkable;
//        worldPosition = _worldPos;
//        gridX = _gridX;
//        gridY = _gridY;
//    }

//    // fCost is the total cost (gCost + hCost)
//    public int fCost
//    {
//        get { return gCost + hCost; }
//    }
//}
