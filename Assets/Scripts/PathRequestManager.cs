﻿using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;


public class PathRequestManager : MonoBehaviour {

    Queue<PathRequest> pathRequestQueue = new Queue<PathRequest>();
    PathRequest currentPathRequest;

    static PathRequestManager instance;
    Pathfinding pathfinding;

    bool isProcessingPath;

    void Awake()
    {
        instance = this;
        pathfinding = GetComponent<Pathfinding>();
    }

    public static void RequestPath(Vector3 pathStart, Vector3 pathEnd, bool isFollowing, Action<Vector3, bool, bool> callback)
    {
        PathRequest newRequest = new PathRequest(pathStart, pathEnd, isFollowing, callback);
        instance.pathRequestQueue.Enqueue(newRequest);
        instance.TryProcessNext();
    }

    void TryProcessNext()
    {
        if (!isProcessingPath && pathRequestQueue.Count > 0)
        {
            currentPathRequest = pathRequestQueue.Dequeue();
            isProcessingPath = true;
            pathfinding.StartFindPath(currentPathRequest.pathStart, currentPathRequest.pathEnd, currentPathRequest.isFollowing);
        }
    }

    public void FinishProcessingPath(Vector3 path, bool success, bool isFinish)
    {
        currentPathRequest.callback(path, success, isFinish);
        isProcessingPath = false;
        TryProcessNext();
    }

    struct PathRequest
    {
        public Vector3 pathStart;
        public Vector3 pathEnd;
        public Action<Vector3, bool, bool> callback;
        public bool isFollowing;

        public PathRequest(Vector3 _start, Vector3 _end, bool _isFollowing, Action<Vector3, bool, bool> _callback)
        {
            pathStart = _start;
            pathEnd = _end;
            callback = _callback;
            isFollowing = _isFollowing;
        }
    }
}
