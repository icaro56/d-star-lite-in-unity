using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System;

public class Pathfinding : MonoBehaviour {

    public Camera camera;
    public GameObject dynamicObstacle;

    PathRequestManager requestManager;
    Grid grid;

    Vector3 nextMove;
    private double C1 = 1.0;
    private double k_m;
    private int maxSteps = 80000;

    private Node startNode;
    private Node targetNode;
    private Node lastNode;

    private Heap<Node> openSet;
    Hashtable cellHash = new Hashtable();
    //HashSet<Pair<double, double>> cellHash = new HashSet<Pair<double, double>>();
    public List<Node> updatedNodesTrigger = new List<Node>();
    List<Node> updatedNodes = new List<Node>();
    List<Node> updatedObservedNodes = new List<Node>();
    List<Node> path = new List<Node>();

    private double M_SQRT2 = Mathf.Sqrt(2.0f);

    public static UInt32 visitedCount;
    public static UInt32 updatedCount;

    void Awake()
    {
        requestManager = GetComponent<PathRequestManager>();
        grid = GetComponent<Grid>();

        InvokeRepeating("FindUpdatedNodes", 0.25f, 0.25f);
    }

    public void StartFindPath(Vector3 startPos, Vector3 targetPos, bool isFollowing)
    {
        //FindUpdatedNodes();
        StartCoroutine(FindPath(startPos, targetPos, isFollowing));
    }

    void FindUpdatedNodes()
    {
        updatedObservedNodes = grid.ListUpdateNodesInGrid();
    }

    void Update()
    {
        if (Input.GetKeyUp(KeyCode.U))
        {
            updatedObservedNodes = grid.ListUpdateNodesInGrid();
        }


        if (Input.GetMouseButtonUp(0))
        {
            RaycastHit hit;
            Ray ray = camera.ScreenPointToRay(Input.mousePosition);

            if (Physics.Raycast(ray, out hit, 100000))
            {
                Node nodeHit = grid.NodeFromWorldPoint(hit.point);
                if (nodeHit != null && (nodeHit.neq(startNode)) && (nodeHit.neq(targetNode)))
                {
                    if (!double.IsPositiveInfinity(nodeHit.cost))
                    {
                        updatedNodes.Add(nodeHit);
                        
                        Instantiate(dynamicObstacle, nodeHit.worldPosition, Quaternion.identity);
                    }

                    int tempX = nodeHit.gridX;
                    int tempY;
                    for (int y = -3; y < 3; y++)
                    {
                        if (y == 0)
                            continue;

                        tempY = nodeHit.gridY + y;
                        Node node = grid.NodeFromPos(tempX, tempY);
                        updatedNodes.Add(node);

                        Instantiate(dynamicObstacle, node.worldPosition, Quaternion.identity);
                    }
                }
            }
        }

        if (Input.GetMouseButtonUp(1))
        {
            RaycastHit hit;
            Ray ray = camera.ScreenPointToRay(Input.mousePosition);

            if (Physics.Raycast(ray, out hit, 100000))
            {
                Node nodeHit = grid.NodeFromWorldPoint(hit.point);
                if (nodeHit != null && (nodeHit.neq(startNode)) && (nodeHit.neq(targetNode)))
                {
                    if (!double.IsPositiveInfinity(nodeHit.cost))
                    {
                        updatedNodes.Add(nodeHit);

                        Instantiate(dynamicObstacle, nodeHit.worldPosition, Quaternion.identity);
                    }

                    int tempX;
                    int tempY = nodeHit.gridY;
                    for (int x = -3; x < 3; x++)
                    {
                        if (x == 0)
                            continue;

                        tempX = nodeHit.gridX + x;
                        Node node = grid.NodeFromPos(tempX, tempY);
                        updatedNodes.Add(node);

                        Instantiate(dynamicObstacle, node.worldPosition, Quaternion.identity);
                    }
                }
            }
        }
    }

    //D* Lite methods

    public bool KeyCompare(Pair<double, double> key1, Pair<double, double> key2)
    {
        if (key1.First + 0.000001 < key2.First)
            return true;
        else if (key1.First - 0.000001 > key2.First)
            return false;
        else if (key1.Second + 0.000001 < key2.Second)
            return true;
        else if (key1.Second - 0.000001 > key2.Second)
            return false;

        return false;
    }

    private bool close(double x, double y)
    {
        if (double.IsPositiveInfinity(x) && double.IsPositiveInfinity(y))
            return true;

        return (Math.Abs(x - y) < 0.00001);
    }

    private Pair<double, double> calculateKey(Node u)
    {
        double val = Math.Min(RHS(u), G(u));

        Pair<double, double> k = new Pair<double, double>();
        k.First = val + Heuristic(startNode, u) + k_m;
        k.Second = val;

        return k;
    }

    private double RHS(Node node, double value = double.MinValue)
    {
        if (node.eq(targetNode))
            return 0.0;

        makeNewCell(node);

        Pair<double, double> g_rhs = (Pair<double, double>)cellHash[node];

        if (value != double.MinValue)
        {
            g_rhs.Second = value;
            //node.k.Second = value;
            cellHash[node] = g_rhs;
        }

        return g_rhs.Second;
    }

    private double G(Node node, double value = double.MinValue)
    {
        makeNewCell(node);

        Pair<double, double> g_rhs = (Pair<double, double>)cellHash[node];

        if (value != double.MinValue)
        {
            g_rhs.First = value;
            cellHash[node] = g_rhs;
            //node.k.First = value;
        }

        return g_rhs.First;
    }

    private void makeNewCell(Node u)
    {
        if (cellHash.Contains(u))
            return;

        /*u.g = u.rhs = Heuristic(u, targetNode);
        u.cost = C1;*/

        double h = double.PositiveInfinity;
        //u.initFirstAndSecond(h, h);
        cellHash.Add(u, new Pair<double, double>(h, h));
    }

    private void listInsert(Node node)
    {
        openSet.Add(node);
    }

    private void listUpdate(Node node)
    {
        openSet.UpdateItem(node);
    }

    private void listRemove(Node node)
    {
        openSet.Remove(node);
    }

    private bool init(Vector3 startPos, Vector3 targetPos)
    {
        startNode = grid.NodeFromWorldPoint(startPos);
        targetNode = grid.NodeFromWorldPoint(targetPos);

        if (startNode.walkable && targetNode.walkable)
        {
            openSet = new Heap<Node>(grid.MaxSize);

            k_m = 0;

            //para todos os estados, fazemos rhs e g = infinito. Porém isso já foi feito na criação do grid
            //grid.ClearDStarParams();

            //objetivo
            targetNode.rhs = 0.0;
            targetNode.cost = C1;

            Pair<double, double> k = calculateKey(targetNode);
            targetNode.k = k;

            listInsert(targetNode);

            lastNode = startNode;

            return true;
        }
        else
        {
            return false;
        }
    }

    private void updateVertex(Node u)
    {
        bool diff = G(u) != RHS(u);
        bool exists = openSet.Contains(u);

        if (diff && exists)
        {
            u.k = calculateKey(u);
            listUpdate(u);
        }
        else if (diff && !exists)
        {
            u.k = calculateKey(u);
            listInsert(u);
        }
        else if (!diff && exists)
        {
            listRemove(u);
        }
    }

    private int computeShortestPath()
    {
        if (openSet.Count == 0)
            return 1;

        int numStep = 0;

        Node u;
        Pair<double, double> kOld;
        Pair<double, double> kNew;
        List<Node> nodes = new List<Node>();
        double gOld;
        double tempG;
        double tempRHS;
        
        while ( openSet.Count > 0 && (
               (KeyCompare(openSet.First().k, calculateKey(startNode))) ||
               (!Math.Equals(RHS(startNode), G(startNode))))) {

            if (++numStep > maxSteps)
                return -1;

            u = openSet.First();
            kOld = u.k;
            kNew = calculateKey(u);

            tempRHS = RHS(u);
            tempG = G(u);

            visitedCount++;

            if (KeyCompare(kOld, kNew)) 
            {
                u.k = kNew;
                listUpdate(u);
            }
            else if (tempG - 0.000001 > tempRHS)
            {
                G(u, tempRHS);
                tempG = tempRHS;

                listRemove(u);

                nodes = grid.GetPred(u);
                for(int i = 0; i < nodes.Count; i++)
                {
                    Node temp = nodes[i];

                    if (temp.neq(targetNode))
                        RHS(temp, Math.Min(RHS(temp), Cost(temp, u) + tempG));

                    updateVertex(temp);
                }
            }
            else //g <= rhs, o estado piorou
            {
                gOld = tempG;
                G(u, double.PositiveInfinity);

                if (u.neq(targetNode))
                {
                    RHS(u, MinSuccessor(u).Second);
                }

                updateVertex(u);

                nodes = grid.GetPred(u);

                for (int i = 0; i < nodes.Count; i++)
                {
                    Node temp = nodes[i];

                    if (Math.Equals(RHS(temp), (Cost(temp, u) + gOld)))
                    {
                        if (temp.neq(targetNode))
                        {
                            RHS(temp, MinSuccessor(temp).Second);
                        }
                    }

                    updateVertex(temp);
                }
            }
        }

        return 0;
    }

    private Pair<Node, double> MinSuccessor(Node node)
    {
        List<Node> successorNodes = new List<Node>();
        successorNodes = grid.GetSucc(node);

        //double trueDistanceMin = 0;

        double tempCost;
        double tempG;

        Node nodeMin = null;
        double costMin = double.PositiveInfinity;

        for (int i = 0; i < successorNodes.Count; i++)
        {
            Node successorNode = successorNodes[i];

            if (path.Contains(successorNode))
                continue;

            tempCost = Cost(node, successorNode);
            tempG = G(successorNode);

            if (double.IsPositiveInfinity(tempCost) || double.IsPositiveInfinity(tempG))
                continue;

            tempCost += tempG;

            /*double trueDistanceVal = TrueDistance(successorNode, targetNode) + TrueDistance(node, successorNode);

            if (close(tempCost, costMin))
            {
                if (trueDistanceMin > trueDistanceVal)
                {
                    trueDistanceMin = trueDistanceVal;
                    costMin = tempCost;
                    nodeMin = successorNode;
                }
            }
            else */if (tempCost < costMin)
            {
                //trueDistanceMin = trueDistanceVal;
                costMin = tempCost;
                nodeMin = successorNode;
            }
        }
        successorNodes.Clear();

        return new Pair<Node, double>(nodeMin, costMin);
    }

    public bool replan()
    {
        path.Clear();

        int result = computeShortestPath();
        if (result != 0)
            return false;

        Node current = startNode;
        path.Add(current);

        while (current != null && current.neq(targetNode))
        {
            if ((current == null) || double.IsPositiveInfinity(G(current)))
            {
                return false;
            }

            current = MinSuccessor(current).First;

            path.Add(current);
        }

        return true;
    }

    public void updateKM()
    {
        k_m += Heuristic(lastNode, startNode);
        lastNode = startNode;
    }

    public void update(Node u, double cost)
    {
        if (u.eq(targetNode))
            return;

        makeNewCell(u);

        double costOld = u.cost;
        double costNew = cost;
        u.cost = cost;

        List<Node> nodes = grid.GetPred(u);

        double tempCostOld, tempCostNew;
        double tempRHS, tempG;

        updatedCount++;

        // Update U
        for (int i = 0; i < nodes.Count; i++)
        {
            updatedCount++;

            Node temp = nodes[i];

            u.cost = costOld;
            tempCostOld = Cost(u, temp);
            u.cost = costNew;
            tempCostNew = Cost(u, temp);

            tempRHS = RHS(u);
            tempG = G(temp);

            if (tempCostOld > tempCostNew)
            {
                if (u.neq(targetNode))
                {
                    RHS(u, Math.Min(tempRHS, (tempCostNew + tempG)));
                }
            }
            else if (Math.Equals(tempRHS, (tempCostOld + tempG)))
            {
                if (u.neq(targetNode))
                {
                    RHS(u, MinSuccessor(u).Second);
                }
            }
        }

        updateVertex(u);

        // Update neighbors
        for (int i = 0; i < nodes.Count; i++)
        {
            Node temp = nodes[i];

            u.cost = costOld;
            tempCostOld = Cost(u, temp);
            u.cost = costNew;
            tempCostNew = Cost(u, temp);

            tempRHS = RHS(temp);
            tempG = G(u);

            if (tempCostOld > tempCostNew)
            {
                if (temp.neq(targetNode))
                {
                    RHS(temp, Math.Min(tempRHS, (tempCostNew + tempG)));
                }
            }
            else if (Math.Equals(tempRHS, (tempCostOld + tempG)))
            {
                if (temp.neq(targetNode))
                {
                    RHS(temp, MinSuccessor(temp).Second);
                }
            }

            updateVertex(temp);
        }
    }

    //-------------------------------------------------------

    IEnumerator FindPath(Vector3 startPos, Vector3 targetPos, bool isFollowing)
    {
        Stopwatch sw = new Stopwatch();
        sw.Start();

        bool replanReturn = false;

        if (!isFollowing)
        {
            if (init(startPos, targetPos))
            {
                replanReturn = replan();
            }
        }
        else
        {
            if (updatedNodes.Count > 0 || updatedObservedNodes.Count > 0 || updatedNodesTrigger.Count > 0)
            {
                updateKM();

                if (updatedNodesTrigger.Count > 0)
                {
                    for (int i = 0; i < updatedNodesTrigger.Count; i++)
                    {
                        Node nodeHit = updatedNodesTrigger[i];
                        update(nodeHit, double.PositiveInfinity);
                    }
                    updatedNodesTrigger.Clear();
                }

                if (updatedNodes.Count > 0)
                {
                    for (int i = 0; i < updatedNodes.Count; i++)
                    {
                        Node nodeHit = updatedNodes[i];
                        update(nodeHit, double.PositiveInfinity);
                    }
                    updatedNodes.Clear();
                }

                if (updatedObservedNodes.Count > 0)
                {
                    for (int i = 0; i < updatedObservedNodes.Count; i++)
                    {
                        Node nodeHit = updatedObservedNodes[i];
                        update(nodeHit, nodeHit.movementPenalty);
                    }
                    updatedObservedNodes.Clear();
                }

                replanReturn = replan();
            }
            else
            {
                replanReturn = true;
            }
        }

        yield return null;
        if (replanReturn)
        {
            if (path.Count > 0)
            {
                path.RemoveAt(0);

                nextMove = path[0].worldPosition;
                startNode = path[0];

                //path.RemoveAt(0);
            }
            
            requestManager.FinishProcessingPath(nextMove, replanReturn, (startNode.eq(targetNode)));
        }
    }

    Vector3[] SimplifyPath(List<Node> path)
    {
        List<Vector3> waypoints = new List<Vector3>();
        //Vector2 directionOld = Vector2.zero;

        for (int i = 0; i < path.Count; i++)
        {
            //Vector2 directionNew = new Vector2(path[1 - 1].gridX - path[i].gridX, path[1 - 1].gridY - path[i].gridY);
            //if (directionNew != directionOld)
            //{
                waypoints.Add(path[i].worldPosition);
            //}

            //directionOld = directionNew;
        }

        return waypoints.ToArray();
    }

    //possui a heuristica da diagonal
    /*double GetDistance(Node nodeA, Node nodeB)
    {
        int distX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int distY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        //return distX + distY;

        if (distX > distY)
            return 14 * distY + 10 * (distX - distY);

        return 14 * distX + 10 * (distY - distX);
    }*/

    //Returns the 8-way distance between state a and state b
    double Heuristic(Node a, Node b)
    {
        double temp;
        double min = Math.Abs(a.gridX - b.gridX);
        double max = Math.Abs(a.gridY - b.gridY);

        if (min > max)
        {
            temp = min;
            min = max;
            max = temp;
        }

        return ((M_SQRT2 - 1.0) * min + max);
    }

    double TrueDistance(Node a, Node b)
    {
        double x = a.gridX - b.gridX;
        double y = a.gridY - b.gridY;

        return Math.Sqrt(x * x + y * y);
    }

    double Cost(Node nodeA, Node nodeB)
    {
        if ( (!nodeA.walkable || double.IsPositiveInfinity(nodeA.cost)) ||
             (!nodeB.walkable || double.IsPositiveInfinity(nodeB.cost)) )
        {
            return double.PositiveInfinity;
        }

        int xd = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int yd = Mathf.Abs(nodeA.gridY - nodeB.gridY);
        double scale = 1.0;

        if (xd + yd > 1)
            scale = M_SQRT2;

        return scale * ((nodeA.cost + nodeB.cost) / 2);
    }
}
