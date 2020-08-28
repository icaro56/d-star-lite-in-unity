using UnityEngine;
using System.Collections;

public class AnimatorPatrol : MonoBehaviour {

    public float speed = 0.15f;

	// Use this for initialization
	void Start () {
        GetComponent<Animator>().speed = speed;
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
