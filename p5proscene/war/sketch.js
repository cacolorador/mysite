
function setup() {	

	  createCanvas(600, 400,WEBGL);	 
	  var  scene = new CustomScene(this); 
	   
}
	
function draw() {
		background(200);
	
		  torus(100, 30);		
		
}		



function CustomScene(P5JS){
	
	 
	 
	 this.P5Scene = new P5Scene(P5JS);	 
	
			 
		myCheckIfGrabsInput = function (event) {		
			var key = event.remixlab_bias_event_KeyboardEvent_key;			
			return String.fromCharCode(key) == 'S' || String.fromCharCode(key) == 'A' || String.fromCharCode(key) == 'R' || String.fromCharCode(key)  == '1';					
		};
	 
	this.P5Scene.subscribe("checkIfGrabsInput",myCheckIfGrabsInput);
  	 		
	
	 window.P5Scene = this.P5Scene ;
	
		myPerformInteraction = function (event) {

			var key = event.remixlab_bias_event_KeyboardEvent_key;			
			var eye = window.P5Scene.eye();

			if(String.fromCharCode(key) == "S")			
				eye.interpolateToFitScene();
			if(String.fromCharCode(key) == "A")			
				eye.addKeyFrameToPath(1);
			if(String.fromCharCode(key) == "R")			
				eye.deletePath(1);
			if(String.fromCharCode(key) == "1")			
				eye.playPath(1);									

		};
	 
	this.P5Scene.subscribe("performInteraction",myPerformInteraction);
			
}



