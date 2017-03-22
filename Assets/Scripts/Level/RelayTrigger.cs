using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RelayTrigger : MonoBehaviour {
	public int id = 0;

	void OnTriggerEnter(Collider other) {
		BleController.ActivateRelay(id, true);
		DebugText.show("Relay " + id + " ON");
	}

	void OnTriggerExit(Collider other) {
		BleController.ActivateRelay(id, false);
		DebugText.show("Relay " + id + " OFF");
	}

}
