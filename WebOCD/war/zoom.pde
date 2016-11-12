/*import damkjer.ocd.*;

Camera camera1;*/


var lastCalledTime;
var fps;

void requestAnimFrame() {

  if(!lastCalledTime) {
     lastCalledTime = Date.now();
     fps = 0;
     return;
  }
  delta = (Date.now() - lastCalledTime)/1000;
  lastCalledTime = Date.now();
  fps = 1/delta;
} 



var camera1;

void setup() {
  size(100, 100, P3D);

  camera1 = new Camera(this, 100, -125, 150);
}

void draw() {
  background(204);
  lights();
  camera1.feed();

  rotateY(PI/3);
  box(50);
  
  
  			 // show fps
		requestAnimFrame();
		println(fps , 15, 30); 
		//fill(0, 102, 153);	
}

void mouseMoved() {
  camera1.zoom(radians(mouseY - pmouseY) / 2.0);
}