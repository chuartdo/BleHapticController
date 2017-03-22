using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DebugText : MonoBehaviour {
	public int id = 0;
    TextMesh textMesh;
    static DebugText _instance;
	static TextMesh[] texts = new TextMesh[5];


    void Awake()
    {
        if (_instance == null)
            _instance = this;
		textMesh = GetComponent<TextMesh>();
    }

    void Start () {
        textMesh = GetComponent<TextMesh>();
		texts[id] = textMesh;
    }

    // Update is called once per frame
	static public   void show (string message, int myid = 0) {
		if (texts[myid] != null) {
 				texts[myid].text = message;
		}
	}

}
