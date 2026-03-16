using System.Collections.Generic;
using UnityEngine;

public class Node : MonoBehaviour
{
    public LayerMask obstacleLayer;
    public readonly List<Vector2> availableDirections = new();

    // Neighboring nodes discovered at startup, used by A* pathfinding
    public readonly List<Node> neighbors = new();

    private void Start()
    {
        availableDirections.Clear();
        neighbors.Clear();

        CheckAvailableDirection(Vector2.up);
        CheckAvailableDirection(Vector2.down);
        CheckAvailableDirection(Vector2.left);
        CheckAvailableDirection(Vector2.right);
    }

    private void CheckAvailableDirection(Vector2 direction)
    {
        RaycastHit2D hit = Physics2D.BoxCast(transform.position, Vector2.one * 0.5f, 0f, direction, 1f, obstacleLayer);

        if (hit.collider == null)
        {
            availableDirections.Add(direction);

            // Cast further to find the next node in this direction and store it
            // as a neighbor so A* can walk the graph without repeated raycasts
            RaycastHit2D nodeHit = Physics2D.BoxCast(transform.position, Vector2.one * 0.5f, 0f, direction, 100f, ~obstacleLayer);
            if (nodeHit.collider != null)
            {
                Node neighbor = nodeHit.collider.GetComponent<Node>();
                if (neighbor != null && neighbor != this)
                {
                    neighbors.Add(neighbor);
                }
            }
        }
    }

}
