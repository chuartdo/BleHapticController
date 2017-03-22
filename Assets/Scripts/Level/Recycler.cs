using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Recycler : MonoBehaviour {
	public Transform spawnPoint;

	void OnTriggerEnter(Collider other) {
		if (spawnPoint != null && other.tag == "Respawn") 
			other.gameObject.transform.position = spawnPoint.transform.position;
 		else 
			Destroy(other.gameObject,1f);
	}
}
