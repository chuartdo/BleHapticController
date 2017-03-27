using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FanTrigger : MonoBehaviour {
	public int id = 0;
	public GameObject blade;
	float rotation = 1;
	float torque ;
    Rigidbody rb;

	void Start() {
		rb = GetComponentInChildren<Rigidbody>();
	}

	void FixedUpdate() {
		rb.AddTorque(transform.up * torque  );
	}


	void OnTriggerEnter(Collider other) {
		if (other.tag == "Player") {
			BleController.ActivateRelay(id, true);
			torque = 9000;
		}
 	}

	void OnTriggerExit(Collider other) {
		if (other.tag == "Player") {
			BleController.ActivateRelay(id, false);
			torque = 0;
		}

	}

}
