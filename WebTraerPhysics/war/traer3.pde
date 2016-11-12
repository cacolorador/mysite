// Traer Physics 3.0
// Terms from Traer's download page, http://traer.cc/mainsite/physics/
//   LICENSE - Use this code for whatever you want, just send me a link jeff@traer.cc
//
// traer3.pde
//   From traer.physics
//     Attraction              Particle                     
//     EulerIntegrator         ParticleSystem  
//     Force                   RungeKuttaIntegrator         
//     Integrator              Spring
//     ModifiedEulerIntegrator Vector3D          
//
//   From traer.animator    
//     Smoother                                       
//     Smoother3D                  
//     Tickable                           
//
// 13 Dec 2010: Copied 3.0 src from http://traer.cc/mainsite/physics/ and ported to Processingjs,
//              added makeParticle2(), makeAttraction2(), replaceAttraction(), and removeParticle(int) -mrn
// Last updated: 16 Dec 2010 15:40, Mike Niemi

//===========================================================================================
//                                      Attraction
//===========================================================================================
// attract positive repel negative
//package traer.physics;
public class Attraction implements Force
{
  Particle a;
  Particle b;
  float k;
  boolean on;
  float distanceMin;
  float distanceMinSquared;
	
  public Attraction( Particle a, Particle b, float k, float distanceMin )
  {
    this.a = a;
    this.b = b;
    this.k = k;
    on = true;
    this.distanceMin = distanceMin;
    this.distanceMinSquared = distanceMin*distanceMin;
  }

  protected void        setA( Particle p )            { a = p; }
  protected void        setB( Particle p )            { b = p; }
  public final float    getMinimumDistance()          { return distanceMin; }
  public final void     setMinimumDistance( float d ) { distanceMin = d; distanceMinSquared = d*d; }
  public final void     turnOff()                     { on = false; }
  public final void     turnOn()	              { on = true;  }
  public final void     setStrength( float k )        { this.k = k; }
  public final Particle getOneEnd()                   { return a; }
  public final Particle getTheOtherEnd()              { return b; }
  
  public void apply() 
  { if ( on && ( a.isFree() || b.isFree() ) )
      {
	float a2bX = a.position().x() - b.position().x();
	float a2bY = a.position().y() - b.position().y();
	float a2bZ = a.position().z() - b.position().z();

	float a2bDistanceSquared = a2bX*a2bX + a2bY*a2bY + a2bZ*a2bZ;

	if ( a2bDistanceSquared < distanceMinSquared )
	   a2bDistanceSquared = distanceMinSquared;

	float force = k * a.mass0 * b.mass0 / a2bDistanceSquared;

	float length = (float)Math.sqrt( a2bDistanceSquared );
			
	// make unit vector
	a2bX /= length;
	a2bY /= length;
	a2bZ /= length;
			
	// multiply by force 
	a2bX *= force;
	a2bY *= force;
	a2bZ *= force;

	// apply
	if ( a.isFree() )
	   a.force().add( -a2bX, -a2bY, -a2bZ );
	if ( b.isFree() )
	   b.force().add( a2bX, a2bY, a2bZ );
      }
  }

  public final float   getStrength() { return k; }
  public final boolean isOn()        { return on; }
  public final boolean isOff()       { return !on; }
} // Attraction

//===========================================================================================
//                                      EulerIntegrator
//===========================================================================================
//package traer.physics;
public class EulerIntegrator implements Integrator
{
  ParticleSystem s;
	
  public EulerIntegrator( ParticleSystem s ) { this.s = s; }
  public void step( float t )
  {
    s.clearForces();
    s.applyForces();
		
    for ( int i = 0; i < s.numberOfParticles(); i++ )
      {
	Particle p = (Particle)s.getParticle( i );
	if ( p.isFree() )
          {
	    p.velocity().add( p.force().x()/(p.mass()*t), p.force().y()/(p.mass()*t), p.force().z()/(p.mass()*t) );
	    p.position().add( p.velocity().x()/t, p.velocity().y()/t, p.velocity().z()/t );
	  }
      }
  }
} // EulerIntegrator

//===========================================================================================
//                                          Force
//===========================================================================================
// May 29, 2005
//package traer.physics;
// @author jeffrey traer bernstein
public interface Force
{
  public void    turnOn();
  public void    turnOff();
  public boolean isOn();
  public boolean isOff();
  public void    apply();
} // Force

//===========================================================================================
//                                      EulerIntegrator
//===========================================================================================
//package traer.physics;
public interface Integrator 
{
  public void step( float t );
} // Integrator

//===========================================================================================
//                                    ModifiedEulerIntegrator
//===========================================================================================
//package traer.physics;
public class ModifiedEulerIntegrator implements Integrator
{
  ParticleSystem s;
  public ModifiedEulerIntegrator( ParticleSystem s ) { this.s = s; }
  public void step( float t )
  {
    s.clearForces();
    s.applyForces();
		
    float halftt = 0.5f*t*t;
		
    for ( int i = 0; i < s.numberOfParticles(); i++ )
      {
	Particle p = s.getParticle( i );
	if ( p.isFree() )
	  {
	    float ax = p.force().x()/p.mass();
	    float ay = p.force().y()/p.mass();
	    float az = p.force().z()/p.mass();
				
	    p.position().add( p.velocity().x()/t, p.velocity().y()/t, p.velocity().z()/t );
	    p.position().add( ax*halftt, ay*halftt, az*halftt );
	    p.velocity().add( ax/t, ay/t, az/t );
	  }
      }
  }
} // ModifiedEulerIntegrator

//===========================================================================================
//                                         Particle
//===========================================================================================
//package traer.physics;
public class Particle
{
  protected Vector3D position0;
  protected Vector3D velocity0;
  protected Vector3D force0;
  protected float    mass0;
  protected float    age0;
  protected boolean  dead0;
  boolean            fixed0;
	
  public Particle( float m )
  {
    position0 = new Vector3D();
    velocity0 = new Vector3D();
    force0 = new Vector3D();
    mass0 = m;
    fixed0 = false;
    age0 = 0;
    dead0 = false;
  }
  
  // @see traer.physics.AbstractParticle#distanceTo(traer.physics.Particle)
  public final float distanceTo( Particle p ) { return this.position().distanceTo( p.position() ); }
  
  // @see traer.physics.AbstractParticle#makeFixed()
  public final void makeFixed() { fixed0 = true; velocity0.clear(); }
  
  // @see traer.physics.AbstractParticle#isFixed()
  public final boolean isFixed() { return fixed0; }
  
  // @see traer.physics.AbstractParticle#isFree()
  public final boolean isFree() { return !fixed0; }
  
  // @see traer.physics.AbstractParticle#makeFree()
  public final void makeFree() { fixed0 = false; }
  
  // @see traer.physics.AbstractParticle#position()
  public final Vector3D position() { return position0; }
  public final Vector3D velocity() { return velocity0; }
  
  // @see traer.physics.AbstractParticle#mass()
  public final float mass() { return mass0; }
  
  // @see traer.physics.AbstractParticle#setMass(float)
  public final void setMass( float m ) { mass0 = m; }
  
  // @see traer.physics.AbstractParticle#force()
  public final Vector3D force() { return force0; }
  
  // @see traer.physics.AbstractParticle#age()
  public final float age() { return age0; }
  
  protected void reset()
  {
    age0 = 0;
    dead0 = false;
    position0.clear();
    velocity0.clear();
    force0.clear();
    mass0 = 1f;
  }
} // Particle

//===========================================================================================
//                                      ParticleSystem
//===========================================================================================
// May 29, 2005
//package traer.physics;
//import java.util.*;
public class ParticleSystem
{
  public static final int RUNGE_KUTTA = 0;
  public static final int MODIFIED_EULER = 1;
  protected static final float DEFAULT_GRAVITY = 0;
  protected static final float DEFAULT_DRAG = 0.001f;	
  ArrayList  particles;
  ArrayList  springs;
  ArrayList  attractions;
  ArrayList  customForces = new ArrayList();
  Integrator integrator;
  Vector3D   gravity;
  float      drag;
  boolean    hasDeadParticles = false;
  
  public final void setIntegrator( int integrator )
  {
    switch ( integrator )
    {
      case RUNGE_KUTTA:
	this.integrator = new RungeKuttaIntegrator( this );
	break;
      case MODIFIED_EULER:
	this.integrator = new ModifiedEulerIntegrator( this );
	break;
    }
  }
  
  public final void setGravity( float x, float y, float z ) { gravity.set( x, y, z ); }

  // default down gravity
  public final void     setGravity( float g ) { gravity.set( 0, g, 0 ); }
  public final void     setDrag( float d )    { drag = d; }
  public final void     tick()                { tick( 1 ); }
  public final void     tick( float t )       { integrator.step( t ); }
  
  public final Particle makeParticle( float mass, float x, float y, float z )
  {
    Particle p = new Particle( mass );
    p.position().set( x, y, z );
    particles.add( p );
    return p;
  }
  
  public final int makeParticle2( float mass, float x, float y, float z )
  { // mrn
    Particle p = new Particle( mass );
    p.position().set( x, y, z );
    particles.add( p );
    return particles.size()-1;
  }
  
  public final Particle makeParticle() { return makeParticle( 1.0f, 0f, 0f, 0f ); }
  
  public final Spring   makeSpring( Particle a, Particle b, float ks, float d, float r )
  {
    Spring s = new Spring( a, b, ks, d, r );
    springs.add( s );
    return s;
  }
  
  public final Attraction makeAttraction( Particle a, Particle b, float k, float minDistance )
  {
    Attraction m = new Attraction( a, b, k, minDistance );
    attractions.add( m );
    return m;
  }
  
  public final int makeAttraction2( Particle a, Particle b, float k, float minDistance )
  { // mrn
    Attraction m = new Attraction( a, b, k, minDistance );
    attractions.add( m );
    return attractions.size()-1; // return the index 
  }

  public final void replaceAttraction( int i, Attraction m )
  { // mrn
    attractions.set( i, m );
  }  

  public final void clear()
  {
    particles.clear();
    springs.clear();
    attractions.clear();
  }
  
  public ParticleSystem( float g, float somedrag )
  {
    integrator = new RungeKuttaIntegrator( this );
    particles = new ArrayList();
    springs = new ArrayList();
    attractions = new ArrayList();
    gravity = new Vector3D( 0, g, 0 );
    drag = somedrag;
  }
  
  public ParticleSystem( float gx, float gy, float gz, float somedrag )
  {
    integrator = new RungeKuttaIntegrator( this );
    particles = new ArrayList();
    springs = new ArrayList();
    attractions = new ArrayList();
    gravity = new Vector3D( gx, gy, gz );
    drag = somedrag;
  }
  
  public ParticleSystem()
  {
    integrator = new RungeKuttaIntegrator( this );
    particles = new ArrayList();
    springs = new ArrayList();
    attractions = new ArrayList();
    gravity = new Vector3D( 0, ParticleSystem.DEFAULT_GRAVITY, 0 );
    drag = ParticleSystem.DEFAULT_DRAG;
  }
  
  protected final void applyForces()
  {
    if ( !gravity.isZero() )
      {
        for ( int i = 0; i < particles.size(); ++i )
	  {
            Particle p = (Particle)particles.get( i );
            p.force0.add( gravity );
	  }
      }
    for ( int i = 0; i < particles.size(); ++i )
      {
        Particle p = (Particle)particles.get( i );
        p.force0.add( p.velocity0.x() * -drag, p.velocity0.y() * -drag, p.velocity0.z() * -drag );
      }
    for ( int i = 0; i < springs.size(); i++ )
      {
        Spring f = (Spring)springs.get( i );
        f.apply();
      }
    for ( int i = 0; i < attractions.size(); i++ )
      {
        Attraction f = (Attraction)attractions.get( i );
        f.apply();
      }
    for ( int i = 0; i < customForces.size(); i++ )
      {
        Force f = (Force)customForces.get( i );
        f.apply();
      }
  }
  
  protected final void clearForces()
  {
    Iterator i = particles.iterator();
    while ( i.hasNext() )
      {
        Particle p = (Particle)i.next();
        p.force0.clear();
      }
  }
  
  public final int        numberOfParticles()              { return particles.size(); }
  public final int        numberOfSprings()                { return springs.size(); }
  public final int        numberOfAttractions()            { return attractions.size(); }
  public final Particle   getParticle( int i )             { return (Particle)particles.get( i ); }
  public final Spring     getSpring( int i )               { return (Spring)springs.get( i ); }
  public final Attraction getAttraction( int i )           { return (Attraction)attractions.get( i ); }
  public final void       addCustomForce( Force f )        { customForces.add( f ); }
  public final int        numberOfCustomForces()           { return customForces.size(); }
  public final Force      getCustomForce( int i )          { return (Force)customForces.get( i ); }
  public final Force      removeCustomForce( int i )       { return (Force)customForces.remove( i ); }
  public final void       removeParticle( int i )          { particles.remove( i ); } //mrn
  public final void       removeParticle( Particle p )     { particles.remove( p ); }
  public final Spring     removeSpring( int i )            { return (Spring)springs.remove( i ); }
  public final Attraction removeAttraction( int i )        { return (Attraction)attractions.remove( i ); }
  public final void       removeAttraction( Attraction s ) { attractions.remove( s ); }
  public final void       removeSpring( Spring a )         { springs.remove( a ); }
  public final void       removeCustomForce( Force f )     { customForces.remove( f ); }
} // ParticleSystem

//===========================================================================================
//                                      RungeKuttaIntegrator
//===========================================================================================
//package traer.physics;
//import java.util.*;
public class RungeKuttaIntegrator implements Integrator
{	
  ArrayList originalPositions;
  ArrayList originalVelocities;
  ArrayList k1Forces;
  ArrayList k1Velocities;
  ArrayList k2Forces;
  ArrayList k2Velocities;
  ArrayList k3Forces;
  ArrayList k3Velocities;
  ArrayList k4Forces;
  ArrayList k4Velocities;
  ParticleSystem s;

  public RungeKuttaIntegrator( ParticleSystem s )
  {
    this.s = s;
		
    originalPositions = new ArrayList();
    originalVelocities = new ArrayList();
    k1Forces = new ArrayList();
    k1Velocities = new ArrayList();
    k2Forces = new ArrayList();
    k2Velocities = new ArrayList();
    k3Forces = new ArrayList();
    k3Velocities = new ArrayList();
    k4Forces = new ArrayList();
    k4Velocities = new ArrayList();
  }
  final void allocateParticles()
  {
    while ( s.particles.size() > originalPositions.size() )
      {
	originalPositions.add( new Vector3D() );
	originalVelocities.add( new Vector3D() );
	k1Forces.add( new Vector3D() );
	k1Velocities.add( new Vector3D() );
	k2Forces.add( new Vector3D() );
	k2Velocities.add( new Vector3D() );
	k3Forces.add( new Vector3D() );
	k3Velocities.add( new Vector3D() );
	k4Forces.add( new Vector3D() );
	k4Velocities.add( new Vector3D() );
      }
  }
  public final void step( float deltaT )
  {	
    allocateParticles();

    /////////////////////////////////////////////////////////
    // save original position and velocities
    for ( int i = 0; i < s.particles.size(); ++i )
      {
	Particle p = (Particle)s.particles.get( i );
	if ( p.isFree() )
	  {		
	    ((Vector3D)originalPositions.get( i )).set( p.position0 );
	    ((Vector3D)originalVelocities.get( i )).set( p.velocity0 );
	  }
	p.force0.clear();	// and clear the forces
      }
		
    ////////////////////////////////////////////////////////
    // get all the k1 values
    s.applyForces();
		
    // save the intermediate forces
    for ( int i = 0; i < s.particles.size(); ++i )
      {
	Particle p = (Particle)s.particles.get( i );
	if ( p.isFree() )
	  {
	    ((Vector3D)k1Forces.get( i )).set( p.force0 );
	    ((Vector3D)k1Velocities.get( i )).set( p.velocity0 );
	  }
	p.force0.clear();
      }
		
    ////////////////////////////////////////////////////////////////
    // get k2 values
    for ( int i = 0; i < s.particles.size(); ++i )
      {
	Particle p = (Particle)s.particles.get( i );
	if ( p.isFree() )
	  {
	    Vector3D originalPosition = (Vector3D)originalPositions.get( i );
	    Vector3D k1Velocity = (Vector3D)k1Velocities.get( i );
				
	    p.position0.x0 = originalPosition.x0 + k1Velocity.x0 * 0.5f * deltaT;
	    p.position0.y0 = originalPosition.y0 + k1Velocity.y0 * 0.5f * deltaT;
	    p.position0.z0 = originalPosition.z0 + k1Velocity.z0 * 0.5f * deltaT;
				
	    Vector3D originalVelocity = (Vector3D)originalVelocities.get( i );
	    Vector3D k1Force = (Vector3D)k1Forces.get( i );
				
	    p.velocity0.x0 = originalVelocity.x0 + k1Force.x0 * 0.5f * deltaT / p.mass0;
	    p.velocity0.y0 = originalVelocity.y0 + k1Force.y0 * 0.5f * deltaT / p.mass0;
	    p.velocity0.z0 = originalVelocity.z0 + k1Force.z0 * 0.5f * deltaT / p.mass0;
          }
       }
    s.applyForces();

    // save the intermediate forces
    for ( int i = 0; i < s.particles.size(); ++i )
      {
	Particle p = (Particle)s.particles.get( i );
	if ( p.isFree() )
	  {
	    ((Vector3D)k2Forces.get( i )).set( p.force0 );
	    ((Vector3D)k2Velocities.get( i )).set( p.velocity0 );
	  }
	p.force0.clear();	// and clear the forces now that we are done with them
      }
			
    /////////////////////////////////////////////////////
    // get k3 values
    for ( int i = 0; i < s.particles.size(); ++i )
      {
	Particle p = (Particle)s.particles.get( i );
	if ( p.isFree() )
	  {
	    Vector3D originalPosition = (Vector3D)originalPositions.get( i );
	    Vector3D k2Velocity = (Vector3D)k2Velocities.get( i );
				
	    p.position0.x0 = originalPosition.x0 + k2Velocity.x0 * 0.5f * deltaT;
	    p.position0.y0 = originalPosition.y0 + k2Velocity.y0 * 0.5f * deltaT;
	    p.position0.z0 = originalPosition.z0 + k2Velocity.z0 * 0.5f * deltaT;
				
	    Vector3D originalVelocity = (Vector3D)originalVelocities.get( i );
	    Vector3D k2Force = (Vector3D)k2Forces.get( i );
				
	    p.velocity0.x0 = originalVelocity.x0 + k2Force.x0 * 0.5f * deltaT / p.mass0;
	    p.velocity0.y0 = originalVelocity.y0 + k2Force.y0 * 0.5f * deltaT / p.mass0;
	    p.velocity0.z0 = originalVelocity.z0 + k2Force.z0 * 0.5f * deltaT / p.mass0;
	  }
      }
    s.applyForces();
		
    // save the intermediate forces
    for ( int i = 0; i < s.particles.size(); ++i )
      {
	Particle p = (Particle)s.particles.get( i );
	if ( p.isFree() )
	  {
	    ((Vector3D)k3Forces.get( i )).set( p.force0 );
	    ((Vector3D)k3Velocities.get( i )).set( p.velocity0 );
	  }
	p.force0.clear();	// and clear the forces now that we are done with them
      }
		
    //////////////////////////////////////////////////
    // get k4 values
    for ( int i = 0; i < s.particles.size(); ++i )
      {
	Particle p = (Particle)s.particles.get( i );
	if ( p.isFree() )
	  {
	    Vector3D originalPosition = (Vector3D)originalPositions.get( i );
	    Vector3D k3Velocity = (Vector3D)k3Velocities.get( i );
				
	    p.position0.x0 = originalPosition.x0 + k3Velocity.x0 * deltaT;
	    p.position0.y0 = originalPosition.y0 + k3Velocity.y0 * deltaT;
	    p.position0.z0 = originalPosition.z0 + k3Velocity.z0 * deltaT;
				
	    Vector3D originalVelocity = (Vector3D)originalVelocities.get( i );
	    Vector3D k3Force = (Vector3D)k3Forces.get( i );
				
	    p.velocity0.x0 = originalVelocity.x0 + k3Force.x0 * deltaT / p.mass0;
	    p.velocity0.y0 = originalVelocity.y0 + k3Force.y0 * deltaT / p.mass0;
	    p.velocity0.z0 = originalVelocity.z0 + k3Force.z0 * deltaT / p.mass0;
	  }
	}
    s.applyForces();
		
    // save the intermediate forces
    for ( int i = 0; i < s.particles.size(); ++i )
      {
	Particle p = (Particle)s.particles.get( i );
	if ( p.isFree() )
	  {
	    ((Vector3D)k4Forces.get( i )).set( p.force0 );
	    ((Vector3D)k4Velocities.get( i )).set( p.velocity0 );
	  }
      }
		
    /////////////////////////////////////////////////////////////
    // put them all together and what do you get?
    for ( int i = 0; i < s.particles.size(); ++i )
      {
	Particle p = (Particle)s.particles.get( i );
	p.age0 += deltaT;
	if ( p.isFree() )
	  {
	    // update position
	    Vector3D originalPosition = (Vector3D)originalPositions.get( i );
	    Vector3D k1Velocity = (Vector3D)k1Velocities.get( i );
	    Vector3D k2Velocity = (Vector3D)k2Velocities.get( i );
	    Vector3D k3Velocity = (Vector3D)k3Velocities.get( i );
            Vector3D k4Velocity = (Vector3D)k4Velocities.get( i );
			
	    p.position0.x0 = originalPosition.x0 + deltaT / 6.0f * ( k1Velocity.x0 + 2.0f*k2Velocity.x0 + 2.0f*k3Velocity.x0 + k4Velocity.x0 );
	    p.position0.y0 = originalPosition.y0 + deltaT / 6.0f * ( k1Velocity.y0 + 2.0f*k2Velocity.y0 + 2.0f*k3Velocity.y0 + k4Velocity.y0 );
	    p.position0.z0 = originalPosition.z0 + deltaT / 6.0f * ( k1Velocity.z0 + 2.0f*k2Velocity.z0 + 2.0f*k3Velocity.z0 + k4Velocity.z0 );
				
	    // update velocity
	    Vector3D originalVelocity = (Vector3D)originalVelocities.get( i );
	    Vector3D k1Force = (Vector3D)k1Forces.get( i );
	    Vector3D k2Force = (Vector3D)k2Forces.get( i );
	    Vector3D k3Force = (Vector3D)k3Forces.get( i );
	    Vector3D k4Force = (Vector3D)k4Forces.get( i );
				
	    p.velocity0.x0 = originalVelocity.x0 + deltaT / ( 6.0f * p.mass0 ) * ( k1Force.x0 + 2.0f*k2Force.x0 + 2.0f*k3Force.x0 + k4Force.x0 );
	    p.velocity0.y0 = originalVelocity.y0 + deltaT / ( 6.0f * p.mass0 ) * ( k1Force.y0 + 2.0f*k2Force.y0 + 2.0f*k3Force.y0 + k4Force.y0 );
	    p.velocity0.z0 = originalVelocity.z0 + deltaT / ( 6.0f * p.mass0 ) * ( k1Force.z0 + 2.0f*k2Force.z0 + 2.0f*k3Force.z0 + k4Force.z0 );
	  }
	}
  }
} // RungeKuttaIntegrator

//===========================================================================================
//                                         Spring
//===========================================================================================
// May 29, 2005
//package traer.physics;
// @author jeffrey traer bernstein
public class Spring implements Force
{
  float springConstant0;
  float damping0;
  float restLength0;
  Particle a, b;
  boolean on;
    
  public Spring( Particle A, Particle B, float ks, float d, float r )
  {
    springConstant0 = ks;
    damping0 = d;
    restLength0 = r;
    a = A;
    b = B;
    on = true;
  }
  
  public final void     turnOff()                { on = false; }
  public final void     turnOn()                 { on = true; }
  public final boolean  isOn()                   { return on; }
  public final boolean  isOff()                  { return !on; }
  public final Particle getOneEnd()              { return a; }
  public final Particle getTheOtherEnd()         { return b; }
  public final float    currentLength()          { return a.position().distanceTo( b.position() ); }
  public final float    restLength()             { return restLength0; }
  public final float    strength()               { return springConstant0; }
  public final void     setStrength( float ks )  { springConstant0 = ks; }
  public final float    damping()                { return damping0; }
  public final void     setDamping( float d )    { damping0 = d; }
  public final void     setRestLength( float l ) { restLength0 = l; }
  public final void apply()
  {	
    if ( on && ( a.isFree() || b.isFree() ) )
      {
	float a2bX = a.position().x0 - b.position().x0;
	float a2bY = a.position().y0 - b.position().y0;
	float a2bZ = a.position().z0 - b.position().z0;
		
	float a2bDistance = (float)Math.sqrt( a2bX*a2bX + a2bY*a2bY + a2bZ*a2bZ );
		
	if ( a2bDistance == 0 )
	  {
	    a2bX = 0;
	    a2bY = 0;
	    a2bZ = 0;
	   }
	else
	   {
	     a2bX /= a2bDistance;
	     a2bY /= a2bDistance;
	     a2bZ /= a2bDistance;
	   }
	
	// spring force is proportional to how much it stretched 
	float springForce = -( a2bDistance - restLength0 ) * springConstant0; 
		
	// want velocity along line b/w a & b, damping force is proportional to this
	float Va2bX = a.velocity().x0 - b.velocity().x0;
	float Va2bY = a.velocity().y0 - b.velocity().y0;
	float Va2bZ = a.velocity().z0 - b.velocity().z0;
		               		
	float dampingForce = -damping0 * ( a2bX*Va2bX + a2bY*Va2bY + a2bZ*Va2bZ );
		
	// forceB is same as forceA in opposite direction
	float r = springForce + dampingForce;
		
	a2bX *= r;
	a2bY *= r;
	a2bZ *= r;
	    
	if ( a.isFree() )
	   a.force().add( a2bX, a2bY, a2bZ );
	if ( b.isFree() )
	   b.force().add( -a2bX, -a2bY, -a2bZ );
      }
  }
  protected void setA( Particle p ) { a = p; }
  protected void setB( Particle p ) { b = p; }
} // Spring

//===========================================================================================
//                                        Vector3D
//===========================================================================================
//package traer.physics;
public class Vector3D
{
  float x0, y0, z0;

  public Vector3D( float X, float Y, float Z )	              { x0 = X;    y0 = Y;    z0 = Z; }
  public Vector3D()                      		      { x0 = 0;    y0 = 0;    z0 = 0; }
  public Vector3D( Vector3D p )				      { x0 = p.x0; y0 = p.y0; z0 = p.z0; }
  public final float    z()				      { return z0; }
  public final float    y()                   		      { return y0; }
  public final float    x()                   		      { return x0; }
  public final void     setX( float X )                       { x0 = X; }
  public final void     setY( float Y )           	      { y0 = Y; }
  public final void     setZ( float Z )			      { z0 = Z; }
  public final void     set( float X, float Y, float Z )      { x0 = X;     y0 = Y;     z0 = Z; }
  public final void     set( Vector3D p )		      { x0 =  p.x0; y0 = p.y0;  z0 = p.z0; }
  public final void     add( Vector3D p )          	      { x0 += p.x0; y0 += p.y0; z0 += p.z0; }
  public final void     subtract( Vector3D p )		      { x0 -= p.x0; y0 -= p.y0; z0 -= p.z0; }
  public final void     add( float a, float b, float c )      { x0 += a;    y0 += b;    z0 += c; } 
  public final void     subtract( float a, float b, float c ) { x0 -= a;    y0 -= b;    z0 -= c; } 
  public final Vector3D multiplyBy( float f )		      { x0 *= f;    y0 *= f;    z0 *= f; return this; }
  public final float    distanceTo( Vector3D p )  	      { return (float)Math.sqrt( distanceSquaredTo( p ) ); }
  public final float    distanceSquaredTo( Vector3D p )	      { float dx = x0-p.x0; float dy = y0-p.y0; float dz = z0-p.z0; return dx*dx + dy*dy + dz*dz; }
  public final float    distanceTo( float x, float y, float z )
  {
    float dx = this.x0 - x;
    float dy = this.y0 - y;
    float dz = this.z0 - z;
    return (float)Math.sqrt( dx*dx + dy*dy + dz*dz );
  }
  public final float    dot( Vector3D p )         	      { return x0*p.x0 + y0*p.y0 + z0*p.z0; }
  public final float    length()                 	      { return (float)Math.sqrt( x0*x0 + y0*y0 + z0*z0 ); }
  public final float    lengthSquared()			      { return x0*x0 + y0*y0 + z0*z0; }
  public final void     clear()                   	      { x0 = 0; y0 = 0; z0 = 0; }
  public final String   toString()              	      { return new String( "(" + x0 + ", " + y0 + ", " + z0 + ")" ); }
  public final Vector3D cross( Vector3D p )
  {
    return new Vector3D( this.y0*p.z0 - this.z0*p.y0, 
  			 this.x0*p.z0 - this.z0*p.x0,
			 this.x0*p.y0 - this.y0*p.x0 );
  }
  public boolean        isZero()                              { return x0 == 0 && y0 == 0 && z0 == 0; }
} // Vector3D

//===========================================================================================
//                                       Smoother
//===========================================================================================
//package traer.animator;
public class Smoother implements Tickable
{
  public Smoother(float smoothness)                     { setSmoothness(smoothness);  setValue(0.0F); }
  public Smoother(float smoothness, float start)        { setSmoothness(smoothness); setValue(start); }
  public final void     setSmoothness(float smoothness) { a = -smoothness; gain = 1.0F + a; }
  public final void     setTarget(float target)         { input = target; }
  public void           setValue(float x)               { input = x; lastOutput = x; }
  public final float    getTarget()                     { return input; }
  public final void     tick()                          { lastOutput = gain * input - a * lastOutput; }
  public final float    getValue()                      { return lastOutput; }
  public float a, gain, lastOutput, input;
} // Smoother

//===========================================================================================
//                                      Smoother3D
//===========================================================================================
//package traer.animator;
public class Smoother3D implements Tickable
{
  public Smoother3D(float smoothness)
  {
    x0 = new Smoother(smoothness);
    y0 = new Smoother(smoothness);
    z0 = new Smoother(smoothness);
  }
  public Smoother3D(float initialX, float initialY, float initialZ, float smoothness)
  {
    x0 = new Smoother(smoothness, initialX);
    y0 = new Smoother(smoothness, initialY);
    z0 = new Smoother(smoothness, initialZ);
  }
  public final void setXTarget(float X) { x0.setTarget(X); }
  public final void setYTarget(float X) { y0.setTarget(X); }
  public final void setZTarget(float X) { z0.setTarget(X); }
  public final float getXTarget()       { return x0.getTarget(); }
  public final float getYTarget()       { return y0.getTarget(); }
  public final float getZTarget()       { return z0.getTarget(); }
  public final void setTarget(float X, float Y, float Z)
  {
    x0.setTarget(X);
    y0.setTarget(Y);
    z0.setTarget(Z);
  }
  public final void setValue(float X, float Y, float Z)
  {
    x0.setValue(X);
    y0.setValue(Y);
    z0.setValue(Z);
  }
  public final void setX(float X)  { x0.setValue(X); }
  public final void setY(float Y)  { y0.setValue(Y); }
  public final void setZ(float Z)  { z0.setValue(Z); }
  public final void setSmoothness(float smoothness)
  {
    x0.setSmoothness(smoothness);
    y0.setSmoothness(smoothness);
    z0.setSmoothness(smoothness);
  }
  public final void tick()         { x0.tick(); y0.tick(); z0.tick(); }
  public final float x()           { return x0.getValue(); }
  public final float y()           { return y0.getValue(); }
  public final float z()           { return z0.getValue(); }
  public Smoother x0, y0, z0;
} // Smoother3D

//===========================================================================================
//                                      Tickable
//===========================================================================================
//package traer.animator;
public interface Tickable
{
  public abstract void tick();
  public abstract void setSmoothness(float f);
} // Tickable
