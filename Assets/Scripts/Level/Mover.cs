using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Mover : MonoBehaviour {

	// Use this for initialization
	void Start () {
		
	}

	float translation = 0;
	float rotation = 0;

	void Update() { 
		
	translation = BleController.x1 * Time.deltaTime * 4f;
	rotation = BleController.x2 * Time.deltaTime * 80f;
		transform.Translate(0, BleController.b1?3:0, translation);
	transform.Rotate(0, rotation, 0);
 }

}
