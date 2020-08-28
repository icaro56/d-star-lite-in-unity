using UnityEngine;
using System.Collections;

public class Trigger2 : MonoBehaviour {

    public GameObject dStarLite;
    bool activated = false;

    void OnTriggerEnter(Collider other)
    {
        if (other.tag.Equals("Player") && !activated)
        {
            print("Acionando trigger 2");
            int y = 36;
            for (int i = 68; i < 100; i++)
            {
                Node n = dStarLite.GetComponent<Grid>().NodeFromPos(i, y);
                if (n != null && n.walkable)
                {
                    dStarLite.GetComponent<Pathfinding>().updatedNodesTrigger.Add(n);
                    Instantiate(dStarLite.GetComponent<Pathfinding>().dynamicObstacle, n.worldPosition, Quaternion.identity);
                }
            }


            activated = true;
        }
             
    }
}
