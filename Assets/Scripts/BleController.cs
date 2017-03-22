using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System;
using System.Collections.Generic;

public class BleController : MonoBehaviour
{
    private AndroidJavaObject plugin;
    private bool refresh = false;
 
 
	static BleController _instance = null;

	static public float x1,y1;
	static public float x2,y2;
	static public bool b1,b2,b3,b4;

	/*
    byte[ ] byteArray = {
		0xBD,   0x4c,   0xcc,   0xcD, 0xBD,   0x4c,   0xcc,   0xcD,0xBD,   0x4c,   0xcc,   0xcD,
		  0x3f, 0x49, 0xfb, 0xe7, 128,  59,   0,   0, 
	 };
*/

    void Start()
    {
		if (_instance == null) 
			_instance = this;

#if UNITY_ANDROID
		try {
			Debug.Log("running");
			plugin  = new AndroidJavaClass("com.chuart.util.libs.UnityUARTPlugin").CallStatic<AndroidJavaObject>("getInstance");
			DebugText.show ("Waiting Controller connection ..");
			plugin.Call("connectBLEController", "connect");
 		} catch (Exception e) {
			DebugText.show("Error init Class");
 		}
#endif
		Invoke("ReadControlData", 2f);
    }

    void ReadControlData()
    {
        refresh = true;
    }

    void OnApplicationQuit()
    {
#if UNITY_ANDROID
        if (plugin != null)
        {
            plugin.Call("disconnect");
            plugin = null;
        }
#endif
    }

    void Update()
    {
        if (!refresh)
            return;

#if UNITY_ANDROID
        if (plugin != null)
        {
			byte[] recvBuff =  plugin.Call<byte[]>("getDataBuffer");
			byte[] temp = new byte[20];

			// Convert serial bytes to Joystick axis and buttons controls 

			if (!BitConverter.IsLittleEndian) {  
				// Reverse byte order
				Array.Copy(recvBuff, temp, recvBuff.Length);
				Array.Reverse(temp, 0 ,16);
				y2 = BitConverter.ToSingle(temp,0);
				x2 = BitConverter.ToSingle(temp,4);
				y1 = BitConverter.ToSingle(temp,8);
				x1 = BitConverter.ToSingle(temp,12);

			} else { 
				x1 = BitConverter.ToSingle(recvBuff,0);
				y1 = BitConverter.ToSingle(recvBuff,4);
				x2 = BitConverter.ToSingle(recvBuff,8);
				y2 = BitConverter.ToSingle(recvBuff,12);
 
			}

			byte state = recvBuff[19];
			b1 = ((state & 0x01) ==0);
			b2 = ((state & 0x02) ==0);
			b3 = ((state & 0x04) ==0);
			b4 = ((state & 0x08) ==0);

			DebugText.show (x1+", " +y1+"    "+x2+",   " +y2  + "  " + state,1);

		} else 
#endif
		{   // For game testing in Editor without controller
			b1 = Input.GetButton("Fire1");
			b2 = Input.GetButton("Fire2");
			b3 = Input.GetButton("Fire3");
			b4 = Input.GetButton("Jump");
			x1 = Input.GetAxis("Horizontal");
			y1 = Input.GetAxis("Vertical");
			x2 = Input.GetAxis("Mouse X") ;
			y2 = Input.GetAxis("Mouse Y") ;
		}

    }

	static bool GetButtonDown(int intd) {
		return false;

	}

		
  static bool[] relayState = new bool[8];

  static public void ActivateRelay(int id, Boolean isOn) {
		Debug.Log("Relay "+id + "  " + (isOn?"ON":"OFF"));

		// send state change command only if status has changed.
		if (relayState[id] == isOn)
			return;
		
		relayState[id] = isOn;

		// 3 byte command data packet - command, relay_id, value
		byte[] data = new byte[3];

		data[0] = (byte)(isOn?0x90: 0x80);
		data[1] = (byte) id;
		data[2] = (byte)(isOn? 0xff : 0x0);

#if UNITY_ANDROID

		if (_instance != null && _instance.plugin != null)
		{
 			_instance.plugin.Call("sendData", data);
		}
#endif
	}
		

}