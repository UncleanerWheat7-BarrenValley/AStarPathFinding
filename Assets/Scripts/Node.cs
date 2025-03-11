// The Node class represents a single cell in the grid.
using UnityEngine;

public class Node
{
    public bool walkable;
    public Vector3 worldPosition;
    public int gridX;
    public int gridY;

    // Cost from the start node to this node.
    public int gCost = int.MaxValue;
    // Heuristic cost estimate to the target node.
    public int hCost;
    // Reference to the parent node (used to retrace the path).
    public Node parent;

    public Node(bool _walkable, Vector3 _worldPos, int _gridX, int _gridY)
    {
        walkable = _walkable;
        worldPosition = _worldPos;
        gridX = _gridX;
        gridY = _gridY;
    }

    // fCost is the total cost (gCost + hCost)
    public int fCost
    {
        get { return gCost + hCost; }
    }
}