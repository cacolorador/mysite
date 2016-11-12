// pendulums.pde
//
//  8 Dec 2010: Sample copied from 
//              http://processing.org/discourse/yabb2/YaBB.pl?num=1267698314
//              and ported to the Traer Processingjs library -mrn
// 10 Dec 2010: Added a run state indicator -mrn
// 12 Dec 2010: Moved Buttons and Traer physics into separate pde files, 
//              added repulsion slider -mrn
// 17 Dec 2010: Added fps reading and a button to set frameRate(60) or frameRate(9999) -mrn
// Last updated: 17 Dec 2010 10:01, Mike Niemi

// NOTE: To run this in the Processing IDE, uncomment the first line of code in gimeMillis() and comment out the next.
//       To run this in Processing.js, comment out the first line of code in gimeMillis() and uncomment the 2nd.

int            n = 10,  
               kVal = -5;
//ParticleSystem physics;
var physics;

Particle[]     p = new Particle[n], 
               a = new Particle[n];
Particle       perturbation = null;               
Spring[]       s = new Spring[n];
int[][]        f = new int[n][n]; // The original code saved the Attraction objects in this array.
                                  // Removing them appeared to work in the Processing IDE, but not
                                  // in Processingjs. Changing to using the index seems to work.
int[]          pf = new int[n]; // Attractions between the perturbation and the free particles
int            SOMEdelay = 300, i0 = 0, pi = -1;
boolean        justStarting = true, IDE = false;
StateButton    runStateButton;
boolean        running = true;
HScrollbar     hs1;
long           startMillis = 0, totalCycles = 0, totalMillis = 0;
PushButton     limitFrameRateButton;
int            limitingFrameRateState = 0;
                
long gimeMillis()
{
  //IDE=true; return System.currentTimeMillis(); // <-- Uncomment this line when running in the Processing IDE (Java) 
                                               //     AND comment out the next line of code.
                                               
  IDE = false; return gimeMillis2(); // <-- Uncomment this line when running in Processing.js
                                     //     AND comment out the previous line of code.
                                     // The supporting code in the HTML looks like this:
                                     //   <script type="application/javascript">  
                                     //     var gimeMillis2 = function () 
                                     //     {  
                                     //       var d = new Date();
                                     //       return d.getTime();
                                     //     };  
                                     //   </script>   
}

void setup() 
{ 
  size(200,200);
  smooth();
  ellipseMode( CENTER );

  physics = new TraerPhysics.ParticleSystem( 1, 0.05 );
  resetModel();
  gimeMillis();
  
  runStateButton       = new StateButton(-5, -5, 20, 255, 153, 0); 
  hs1                  = new HScrollbar(1, 194, 50, 10, 1); 
  limitFrameRateButton = new PushButton(IDE?105:87, IDE?1:3, IDE?92:70, IDE?13:9, "F.R. Undefined", "F.R. Limited", "F.R. Unlimited");
}

void resetModel()
{ 
  for (int i=0; i<n; i++) 
    {
      p[i] = physics.makeParticle( 1.0, width/2, 25, 0 );
      a[i] = physics.makeParticle( 1.0, random(10,width-10), height/2-2, 0 );

      a[i].makeFixed();
      s[i] = physics.makeSpring( p[i], a[i], 0.5, 0.1, 75 ); // Connects anchor to particle
    }
  
  // Create an attraction between all free particles
  for (int i=0; i<n; i++) 
    for (int j=i+1; j<n; j++) 
        f[i][j] = physics.makeAttraction2( p[i], p[j], kVal, 1 ); // save the index for calcAttractions()
}  

void calcAttractions(float f1)
{ 
  kVal = (int)(-5 - f1 * 500.0); // want to go from -5 to -500
  var att;
  for (int i=0; i<n; i++) 
    for (int j=i+1; j<n; j++) 
        physics.replaceAttraction( f[i][j], new TraerPhysics.Attraction(p[i], p[j], kVal, 1) ); // replace that Attraction
}

void calcPerturbation()
{ if ( perturbation!=null ) 
    {
      if (i0 < 50)
        {
          int kVal0 = -50 + (int)((float)kVal * (float)(50-i0) / 10.0);
          for (int i=0; i<n; i++)
              physics.replaceAttraction( pf[i], new TraerPhysics.Attraction(perturbation, p[i], kVal0, 1) ); 
        }  
      else
        {
          physics.removeParticle( pi ); 
          perturbation = null;
          pi = -1;
          for (int i=n-1; i>=0; i--)
              physics.removeAttraction( pf[i] );
        }     
    }   
}

void draw() 
{ 
  i0++;
  totalCycles++;
  if (startMillis == 0)
     startMillis = gimeMillis(); 
     
  if (i0 > SOMEdelay)
    { // After a while, put our view to sleep to avoid consuming CPU cycles.
      //println("draw(): going to sleep now"); 
      running = false;
      noLoop();
    }   

  background(0);
  runStateButton.display( running?runStateButton.STATEon:runStateButton.STATEoff, // Sets the red indicator
                          SOMEdelay-i0 ); // Tempered by how many cycles until we go to sleep
  limitFrameRateButton.display(limitingFrameRateState);               
                                  
  if (hs1.update(SOMEdelay-i0)) // Sometimes the slider does not quiesce automatically when the mouse leaves it.
    {                           // The arg causes the slider to turn off its color in sync with the runStateButton.
      calcAttractions(hs1.getValue()); 
      i0 = 0; 
    } 
  if (perturbation != null)
     calcPerturbation();  
      
  physics.tick();
  
  hs1.display();
  fill(128,0,0);
  text("Repulsion", 4, 198);  
    
  if ( justStarting && i0>50 ) // Display this text after the 50th cycle when justStarting
    {   
      float f = 255;
      if (i0 < 100)
         f = ((float)i0-50.0)/50.0*255; // 0.0-255.0 .. fade-in the text over 50 cycles
         
      fill(f);
      text("Click on upper half to restart",  25, 40);
      text("Click on lower half to perturb",  25, 55);
      text("Move slider to change repulsion", 25, 70);  
    }   
      
  fill(255);
  
  long currMillis = gimeMillis();
  float f = ((float)totalCycles) *1000.0 / (float)(currMillis - startMillis);
  f = ((float)((int)(f*1000.0)))/1000.0; // Limit to 3 decimal places 
  text(f,     25, 12);
  text("fps", IDE?77:67, 12);
  
  stroke(255, 200);
  
  for (int i=0; i<n; i++) 
    {
      line( p[i].position().x(), p[i].position().y(), 
            a[i].position().x(), a[i].position().y() );
      ellipse( a[i].position().x(), a[i].position().y(), 5, 5 );
      ellipse( p[i].position().x(), p[i].position().y(), 20, 20 );
    }
}

void mousePressed()
{ 
  i0 = 0;
  justStarting = false;

  if (!running) // We have just been reawakened
     startMillis = totalCycles = 0;
  
  if (hs1.over())
    { 
      running = true;
      loop();
      return;
    }  
     
  if (runStateButton.over())
    {
      running = !running;
      if (running)
         loop(); // Wake up the view
      else
          i0 = SOMEdelay; // let draw() put us to sleep
      return;  
    }    
    
  if (limitFrameRateButton.over()) // Undefined > Limited > Unlimited > Limited > ...
    {
      switch(limitingFrameRateState)
        { 
          case 0: limitingFrameRateState=1; frameRate(60); break;
          case 1: limitingFrameRateState=2; frameRate(9999); break;
          case 2: limitingFrameRateState=1; frameRate(60); break;
          default: limitingFrameRateState=2; frameRate(9999);
        }
      
      running = true;
      loop();
      return;
    }
    
  if (mouseY < height/2) 
    { // The mouse is in the upper half - do a reset
      physics.clear();
      if (perturbation != null)
        { // Cleanup
          physics.removeParticle(pi); 
          pi = -1;
          for (int i=0; i<n; i++)
              physics.removeAttraction( pf[i] );
          perturbation = null;
        }  
      resetModel();
    }  
  else
    { // Otherwise, the mouse is in the lower half - create a new perturbation
      boolean replace = false;
      if (perturbation != null)
        {
          physics.removeParticle(pi); // Remove the existing perturbation particle
          replace = true;             // Make a note to replace existing attractions below
        }  
      pi = physics.makeParticle2( 1.0, mouseX, mouseY, 0.0 ); // Make a new perturbation particle, saving its index
      perturbation = physics.getParticle( pi );
      for (int i=0; i<n; i++)
          if (replace)
             physics.replaceAttraction( pf[i], new TraerPhysics.Attraction(perturbation, p[i], kVal, 1) );
          else   
             pf[i] = physics.makeAttraction2( perturbation, p[i], kVal, 1 ); // Returns an index to use later
    }  
  running = true;
  loop(); // Wake up the view (if not already awake)
}
