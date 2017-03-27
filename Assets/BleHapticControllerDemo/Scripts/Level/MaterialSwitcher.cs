using UnityEngine;
using System.Collections;

public class MaterialSwitcher : MonoBehaviour {

 
	// Custom serializable class
	[System.Serializable]
	public class Property{
		public string name;
		public float value = 5f;
 	}
	
	public ProceduralPropertyDescription[]  property;
    ProceduralMaterial material;

	void Start () {
			// Randomize material
		material = GetComponentInChildren<Renderer>().sharedMaterial as ProceduralMaterial;
		// ChangeMaterial();
		if (material != null) {
			 property = material.GetProceduralPropertyDescriptions();

		}
	}


	public void ChangeMaterial() {
		if (material != null && property != null) {
			//material.SetProceduralFloat("Input Selection",  Random.Range (0,4));
			for (int i=0; i<property.Length; i++) {
				string name = property[i].label;
				if (name.EndsWith("Color")) {
					material.SetProceduralColor(name,  new Color( Random.Range (0,360f),1f,0.5f));
 				}
                else if (name.StartsWith("Rnd") || material.HasProceduralProperty(name)) {
					material.SetProceduralFloat(name,  Random.Range (0,4));
					
				}
      
            }          
			material.RebuildTextures();
		} 
	}

}
	
	
