//import traer.physics.*;

		var physics;

		var p;
		var anchor;
		var s;		

void setup()
{
  size( 400, 400 );
  smooth();
  fill( 0 );
  ellipseMode( CENTER );
        
  physics = new TraerPhysics.ParticleSystem( 1, 0.05 );
        
  p = physics.makeParticle( 1.0, width/2, height/2, 0 );
  anchor = physics.makeParticle( 1.0, width/2, height/2, 0 );
  anchor.makeFixed(); 
  s = physics.makeSpring( p, anchor, 0.5, 0.1, 75 );
}

void mousePressed()
{
   p.makeFixed(); 
   p.position().set( mouseX, mouseY, 0 );
}

void mouseDragged()
{
  p.position().set( mouseX, mouseY, 0 );
}

void mouseReleased()
{
   p.makeFree(); 
}

void draw()
{
  physics.tick();
    
  background( 255 );
  
  line( p.position().x(), p.position().y(), anchor.position().x(), anchor.position().y() );
  ellipse( anchor.position().x(), anchor.position().y(), 5, 5 );
  ellipse( p.position().x(), p.position().y(), 20, 20 );
}