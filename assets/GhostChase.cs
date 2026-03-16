using System.Collections.Generic;
using UnityEngine;

public class GhostChase : GhostBehavior
{
    private void OnDisable()
    {
        ghost.scatter.Enable();
    }

    private void OnTriggerEnter2D(Collider2D other)
    {
        Node node = other.GetComponent<Node>();

        if (node != null && enabled && !ghost.frightened.enabled)
        {
            Vector2 direction = FindBestDirection(node);
            ghost.movement.SetDirection(direction);
        }
    }

    /// <summary>
    /// Runs A* from the current node toward the node closest to Pac-Man,
    /// then returns the first step direction of the resulting path.
    /// Falls back to the greedy one-step pick if no path is found.
    /// </summary>
    private Vector2 FindBestDirection(Node startNode)
    {
        Node targetNode = FindNearestNodeToTarget();

        if (targetNode == null || targetNode == startNode)
        {
            return GreedyDirection(startNode);
        }

        List<Node> path = AStar(startNode, targetNode);

        // path[0] is the start node itself, so path[1] is the first step
        if (path != null && path.Count >= 2)
        {
            Vector2 direction = (Vector2)(path[1].transform.position - startNode.transform.position);
            return direction.normalized;
        }

        // Fallback: greedy one-step lookahead (original behaviour)
        return GreedyDirection(startNode);
    }

    /// <summary>
    /// Finds the node on the map whose world position is closest to the
    /// ghost's target (Pac-Man). Uses an overlap circle so only actual Node
    /// GameObjects are considered.
    /// </summary>
    private Node FindNearestNodeToTarget()
    {
        // Collect all nodes in scene via a broad overlap (large radius)
        Collider2D[] hits = Physics2D.OverlapCircleAll(ghost.target.position, 1.5f);

        Node nearest = null;
        float minDist = float.MaxValue;

        foreach (Collider2D hit in hits)
        {
            Node n = hit.GetComponent<Node>();
            if (n == null) continue;

            float dist = (ghost.target.position - n.transform.position).sqrMagnitude;
            if (dist < minDist)
            {
                minDist = dist;
                nearest = n;
            }
        }

        // If nothing within 1.5 units, widen search
        if (nearest == null)
        {
            hits = Physics2D.OverlapCircleAll(ghost.target.position, 50f);
            foreach (Collider2D hit in hits)
            {
                Node n = hit.GetComponent<Node>();
                if (n == null) continue;

                float dist = (ghost.target.position - n.transform.position).sqrMagnitude;
                if (dist < minDist)
                {
                    minDist = dist;
                    nearest = n;
                }
            }
        }

        return nearest;
    }

    /// <summary>
    /// Standard A* search between two nodes.
    /// g = steps taken so far (uniform cost because tiles are 1 unit apart).
    /// h = Manhattan distance heuristic.
    /// Returns the full path from start to goal inclusive, or null if unreachable.
    /// </summary>
    private List<Node> AStar(Node start, Node goal)
    {
        // Maps each node to the node we came from
        Dictionary<Node, Node> cameFrom = new Dictionary<Node, Node>();

        // g-score: cheapest path cost found so far from start
        Dictionary<Node, float> gScore = new Dictionary<Node, float>();

        // f-score: gScore + heuristic
        Dictionary<Node, float> fScore = new Dictionary<Node, float>();

        // Simple open set — for a typical Pac-Man map (~100-200 nodes) a list
        // is fast enough without a priority queue dependency
        List<Node> openSet = new List<Node>();

        gScore[start] = 0f;
        fScore[start] = Heuristic(start, goal);
        openSet.Add(start);

        while (openSet.Count > 0)
        {
            Node current = GetLowestFScore(openSet, fScore);

            if (current == goal)
            {
                return ReconstructPath(cameFrom, current);
            }

            openSet.Remove(current);

            foreach (Node neighbor in current.neighbors)
            {
                float tentativeG = GetScore(gScore, current) + 1f;

                if (tentativeG < GetScore(gScore, neighbor))
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeG;
                    fScore[neighbor] = tentativeG + Heuristic(neighbor, goal);

                    if (!openSet.Contains(neighbor))
                    {
                        openSet.Add(neighbor);
                    }
                }
            }
        }

        return null; // No path found
    }

    private float Heuristic(Node a, Node b)
    {
        // Manhattan distance — admissible for a grid with axis-aligned movement
        Vector2 delta = a.transform.position - b.transform.position;
        return Mathf.Abs(delta.x) + Mathf.Abs(delta.y);
    }

    private float GetScore(Dictionary<Node, float> scores, Node node)
    {
        return scores.TryGetValue(node, out float score) ? score : float.MaxValue;
    }

    private Node GetLowestFScore(List<Node> openSet, Dictionary<Node, float> fScore)
    {
        Node best = openSet[0];
        float bestScore = GetScore(fScore, best);

        for (int i = 1; i < openSet.Count; i++)
        {
            float s = GetScore(fScore, openSet[i]);
            if (s < bestScore)
            {
                bestScore = s;
                best = openSet[i];
            }
        }

        return best;
    }

    private List<Node> ReconstructPath(Dictionary<Node, Node> cameFrom, Node current)
    {
        List<Node> path = new List<Node> { current };

        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            path.Insert(0, current);
        }

        return path;
    }

    /// <summary>
    /// Original greedy one-step lookahead — used as a fallback when A* can't
    /// find a path (e.g. nodes not yet initialised).
    /// </summary>
    private Vector2 GreedyDirection(Node node)
    {
        Vector2 direction = Vector2.zero;
        float minDistance = float.MaxValue;

        foreach (Vector2 availableDirection in node.availableDirections)
        {
            Vector3 newPosition = transform.position + new Vector3(availableDirection.x, availableDirection.y);
            float distance = (ghost.target.position - newPosition).sqrMagnitude;

            if (distance < minDistance)
            {
                direction = availableDirection;
                minDistance = distance;
            }
        }

        return direction;
    }

}