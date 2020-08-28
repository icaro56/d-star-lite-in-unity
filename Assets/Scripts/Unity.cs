using UnityEngine;
using System.Collections;
using System.Diagnostics;

public class Unity : MonoBehaviour {

    public Transform target;
    float speed = 20;
    Vector3 nextMove;
    int targetIndex;
    bool isFinished = false;
    bool isFollowing = false;

    Stopwatch sw;

    void Awake()
    {
        sw = new Stopwatch();
        sw.Start();
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.tag.Equals("Finish"))
        {
            sw.Stop();

            print("Tempo total: " + sw.ElapsedMilliseconds / 1000 + " s");
            print("Nós visitados: " + Pathfinding.visitedCount);
            print("Nós atualizados: " + Pathfinding.updatedCount);
        }

    }

    void Start()
    {
        isFinished = false;
        isFollowing = false;
        PathRequestManager.RequestPath(transform.position, target.position, isFollowing, OnPathFound);
    }

    public void OnPathFound(Vector3 _nextMove, bool pathSuccessful, bool isFinish)
    {
        isFinished = isFinish;

        if (pathSuccessful)
        {
            nextMove = _nextMove;
            //adicionando offset do pivo
            nextMove.y += 0.84f;
            StopCoroutine("FollowPath");
            StartCoroutine("FollowPath");
        }
    }

    IEnumerator FollowPath()
    {
        Vector3 currentWaypoint = nextMove;

        while(true)
        {
            if (transform.position == currentWaypoint)
            {
                if (!isFinished)
                {
                    isFollowing = true;

                    PathRequestManager.RequestPath(transform.position, target.position, isFollowing, OnPathFound);
                }

                yield break;
            }

            
            transform.position = Vector3.MoveTowards(transform.position, currentWaypoint, speed * Time.deltaTime);
            yield return null;
        }
    }

    /*public void OnDrawGizmos()
    {
        if (path != null)
        {
            for (int i = targetIndex; i < path.Length; i++)
            {
                Gizmos.color = Color.black;
                Gizmos.DrawCube(path[i], Vector3.one);

                if (i == targetIndex)
                {
                    Gizmos.DrawLine(transform.position, path[i]);
                }
                else
                {
                    Gizmos.DrawLine(path[i - 1], path[i]);
                }
            }
        }
    }*/
}
