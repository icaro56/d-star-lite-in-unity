using UnityEngine;
using System.Collections;

public class Pair<T, U>
{
    public Pair()
    {
    }

    public Pair(T first, U second)
    {
        this.First = first;
        this.Second = second;
    }

    public T First { get; set; }
    public U Second { get; set; }
};

public class Node : IHeapItem<Node> {

    public bool walkable;
    public Vector3 worldPosition;
    public int gridX;
    public int gridY;

    public int movementPenalty;

    //cell info
    public double rhs;
    //public double g;
    public double cost;

    public Pair<double, double> k = new Pair<double, double>(0, 0);

    public Node parent;
    //public Node bestNextNode;
    int heapIndex;

    public void InitDStarParams(double _rhs, double _g)
    {
        rhs = _rhs;
        //g = _g;
    }

    public void initFirstAndSecond(double f, double s)
    {
        k.First = f;
        k.Second = s;
    }

    public Node(bool _walkable, Vector3 _worldPos, int _gridX, int _gridY, int _penalty)
    {
        walkable = _walkable;
        worldPosition = _worldPos;
        gridX = _gridX;
        gridY = _gridY;
        movementPenalty = _penalty;

        cost = _penalty;
    }

    //construtor usado apenas para o cálculo do kOld em ComputeClosestPath
    public Node(Node other)
    {
        /*this.walkable = other.walkable;
        this.worldPosition = other.worldPosition;
        this.gridX = other.gridX;
        this.gridY = other.gridY;
        this.movementPenalty = other.movementPenalty;*/
        this.k = other.k;
    }

    public int HeapIndex
    {
        get
        {
            return heapIndex;
        }
        set
        {
            heapIndex = value;
        }
    }

    public bool eq(Node n2) {
        return ((this.gridX == n2.gridX) && (this.gridY == n2.gridY));
    }

    public bool neq(Node n2)
    {
        return ((this.gridX != n2.gridX) || (this.gridY != n2.gridY));
    }

    //Greater than
    public bool gt(Node n2)
    {
        if (this.k.First - 0.00001 > n2.k.First)
            return true;
        else if (this.k.First < n2.k.First - 0.00001)
            return false;

        return this.k.Second > n2.k.Second;
    }

    //Less than or equal to
    public bool lte(Node n2)
    {
        if (this.k.First < n2.k.First)
            return true;
        else if (this.k.First > n2.k.First)
            return false;

        return this.k.Second < n2.k.Second + 0.00001;
    }

    //Less than
    public bool lt(Node n2)
    {
        if (this.k.First + 0.000001 < n2.k.First)
            return true;
        else if (this.k.First - 0.000001 > n2.k.First)
            return false;

        return this.k.Second < n2.k.Second;
    }


    public int CompareTo(Node nodeToCompare)
    {
        if (nodeToCompare != null)
        {
            if (this.k.First - 0.00001 > nodeToCompare.k.First)
                return 1;
            else if (this.k.First < nodeToCompare.k.First - 0.00001)
                return -1;
            if (this.k.Second > nodeToCompare.k.Second)
                return 1;
            else if (this.k.Second < nodeToCompare.k.Second)
                return -1;
        }

        return 0;
    }
}
