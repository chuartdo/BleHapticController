using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BallMovement : MonoBehaviour {
	public float thrust;
	public Rigidbody rb;

	void Start () {
		rb = GetComponent<Rigidbody>();
	}

	void FixedUpdate()
	{ 
		//rb.AddForce(transform.forward * thrust);
		rb.AddTorque(transform.up * thrust);
	}


	void OnCollisionEnter(Collision collision) {
		if (tag == "Recycle") {
			if (collision.relativeVelocity.magnitude > 2)
				Destroy(this.gameObject,2f);
		}
	}


}
