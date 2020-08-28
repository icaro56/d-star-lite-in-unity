using UnityEngine;
using System.Collections;

public class PatrolOnlyRotate : MonoBehaviour
{
    private float contador = 0.0f;

    // Use this for initialization
    void Start()
    {
        contador = 0.0f;
    }

    // Update is called once per frame
    void Update()
    {
        contador += Time.deltaTime;

        if (contador >= 0.3f)
        {
            transform.Rotate(Vector3.up * 5);

            contador = 0.0f;
        }
    }
}
