using UnityEngine;
using System.Collections;

public class Trigger1 : MonoBehaviour {

    public GameObject dStarLite;
    bool activated = false;

    void OnTriggerEnter(Collider other)
    {
        if (other.tag.Equals("Player") && !activated)
        {
            print("Acionando trigger 1");
            int y = 36;
            for (int i = 34; i < 52; i++)
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
