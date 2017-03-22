using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerMovement : MonoBehaviour {
	BleController plugin;
	public float speed;
	public GameObject bullet = null;
	public bool useRotation = false;

	void Start () {
		
	}
	
	void Update()
	{
		float x1,y1, x2, y2;
		float up;

		float translation = 0;
		float rotation = 0;

		x1 = BleController.x1;  
		y1 = BleController.y1;
		x2 = BleController.x2;
		y2 = BleController.y2;

		up = BleController.b2?0.1f:0;

	

		if (useRotation) {
			float angle = Mathf.Atan2( x2, y2)* 180;
			transform.rotation = Quaternion.Euler(0, angle, 0);
		}else {
			translation = Time.deltaTime * speed;
			//rotation  *= Time.deltaTime * rotationSpeed;
			transform.Translate(x1 * translation , up, y1 * translation);
		}

		if (BleController.b1) {
			if (bullet != null) {
				GameObject ball = Instantiate(bullet,this.transform.position+Vector3.up * 1.3f,this.transform.rotation);
				ball.transform.parent = null;
			}
		}

	}

}
