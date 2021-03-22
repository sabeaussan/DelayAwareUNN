using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class physic_manager : MonoBehaviour
{
    // script that manage physic such as gravity
    
    private Rigidbody ball;
    public float speed;

    // Start is called before the first frame update
    void Start()
    {
        ball = GetComponent<Rigidbody>();
        float radius = GetComponent<SphereCollider>().radius;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

     void OnCollisionStay(Collision other)
 	{
        Vector3 force = new Vector3(0f, -9.5f, 0f);
        ball.AddForce(force,ForceMode.Force);

 	}


}
