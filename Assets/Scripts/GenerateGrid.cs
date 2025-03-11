using System.Collections.Generic;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

public class GenerateGrid : MonoBehaviour
{
    public LayerMask obstacleLayer;
    public Vector2 gridWorldSize;
    public Node[,] grid;
    public GameObject[] enemies;

    public float nodeDiameter;
    public float nodeRadius;
    public int gridSizeX, gridSizeY;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        nodeDiameter = nodeRadius * 2;
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);
        GenerateEnviroment();

    }

    void GenerateEnviroment()
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
    void OnDrawGizmos()
    {
        Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x, 1, gridWorldSize.y));

        if (grid != null)
        {
            foreach (Node n in grid)
            {
                Gizmos.color = (n.walkable) ? Color.white : Color.red;               
                Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - 0.1f));
            }
        }

        foreach (GameObject enemy in enemies) 
        {
            EnemyController EC = enemy.GetComponent<EnemyController>();
            if (EC.path != null) 
            {
                Gizmos.color = Color.black;
                foreach (Node n in EC.path) 
                {
                    Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - 0.1f));
                }
            }
        }
    }

}
