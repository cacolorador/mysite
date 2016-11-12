// widgets.pde
//   Button       
//     PushButton
//     StateButton  
//     StartPauseStopButton
//     RestartButton
//   HScrollbar
// 
// Last updated: 17 Dec 2010 09:56, Mike Niemi

class Button
{
  int       x, y, buttonSize, ibackgroundcolor, ihighlightcolor, ibasecolor;
  color     currentcolor;
  boolean   over0    = false,
            pressed0 = false,
            locked0  = false;   

  void update() 
  {
    if (over()) 
       currentcolor = color(ihighlightcolor);
    else 
       currentcolor = color(ibasecolor);
  }

  boolean pressed() 
  {
    if (over0) 
      {
        locked0 = true;
        return true;
      } 
    else 
      {
        locked0 = false;
        return false;
      }     
  }

  boolean over() 
  { return true; }

  boolean overRect(int x, int y, int width, int height) 
  {
    if (mouseX >= x && mouseX <= x+width && mouseY >= y && mouseY <= y+height) 
       return true;
    else 
       return false;
  }

  boolean overCircle(int x, int y, int diameter) 
  {
    float disX = x - mouseX;
    float disY = y - mouseY;
    if (sqrt(sq(disX) + sq(disY)) < diameter/2 ) 
       return true;
    else 
       return false;
  }

  boolean overArc(int x, int y, int width, int height) 
  {
    if (mouseX >= x && mouseX <= x+width && 
        mouseY >= (y-height/2) && mouseY <= (y-height/2)+height) 
       return true;
    else 
       return false;
  }
  
} // Button

class PushButton extends Button
{ 
  int buttonWidth, buttonHeight;
  String s[] = new String[3];
  
  PushButton(int ix, int iy, int iWidth, int iHeight, String text0, String text1, String text2) 
  {
    x = ix; // x and y are the upper left corner (like ellipseMode(CORNER))
    y = iy; 
    buttonHeight = iHeight;
    buttonWidth = iWidth;
    s[0] = text0;
    s[1] = text1;
    s[2] = text2;
  }
 
  boolean over() 
  {
    if ( overRect(x, y, buttonWidth, buttonHeight) ) 
      {
        over0 = true;
        return true;
      } 
    else 
      {
        over0 = false;
        return false;
      }
  }

  void display(int state) // e.g. Undefined, Limited, Unlimited
  { 
    switch(state)
    { 
      case(0): fill(100); break;
      case(1): fill(100,0,0); break;
      default: fill(0,100,0);
    }  
    rect(x, y, buttonWidth, buttonHeight);
    
    fill(255);
    text(s[state], x+1, y+buttonHeight-1);
  }  
}

class StateButton extends Button
{ 
  final static int STATEoff = 0, STATEon = 1;
  int cx, cy;
  
  StateButton(int ix, int iy, int isize, int icolor, int ihighlight, int ibackground) 
  {
    x = ix; // x and y are the upper left corner (like ellipseMode(CORNER))
    y = iy; 
    buttonSize = isize;
    cx = x + buttonSize/2; // cx and cy will be the center
    cy = y + buttonSize/2;
    ibasecolor = icolor;
    ihighlightcolor = ihighlight;
    currentcolor = color(ibasecolor);
    ibackgroundcolor = ibackground;
  }

  boolean over() 
  {
    if ( overCircle(cx, cy, buttonSize) ) 
      {
        over0 = true;
        return true;
      } 
    else 
      {
        over0 = false;
        return false;
      }
  }

  void display(int state, int numCyclesLeft) 
  {
    if (state == STATEon)
      { 
        stroke(128);
        
        if (numCyclesLeft < 20) // fade in the idled color
          { 
            float f0 = 1.0 - ((float)numCyclesLeft)/20.0; // 0.0 - 1.0
            float f1 = ibasecolor - f0 * (float)(ibasecolor-ibackgroundcolor);
            if (ibackgroundcolor == 0)
               fill(f1,0,0);
            else
              {   
                float f2 = f0 * 255.0;
                fill(f1,f2,f2);
              }  
          }    
        else
          fill(ibasecolor,0,0); 
          
        ellipse(cx, cy, buttonSize, buttonSize);
      }  
    else
      { 
        stroke(128);
        fill(color(ibackgroundcolor));
        ellipse(cx, cy, buttonSize, buttonSize);
      }  
  }
} // StateButton

class StartPauseStopButton extends Button
{ 
  final static int SHOWrun = 0, SHOWpause = 1, SHOWstop = 2; 
  
  StartPauseStopButton(int ix, int iy, int isize, 
                       int icolor, int ihighlight, int ibackground) 
  {
    x = ix; // x and y are the upper left corner
    y = iy;
    buttonSize = isize;
    ibasecolor = icolor;
    ihighlightcolor = ihighlight;
    currentcolor = color(ibasecolor);
    ibackgroundcolor = ibackground;
  }

  boolean over() 
  {
    if ( overRect(x, y, buttonSize, buttonSize) ) 
      {
        over0 = true;
        return true;
      } 
    else 
      {
        over0 = false;
        return false;
      }
  }

  void display(int whatToShow) 
  {
    if (whatToShow == SHOWstop)
      { 
        stroke(255, 80, 80);
        fill(255,0,0);
        rect(x, y, buttonSize, buttonSize);
      }  
    else
    if (whatToShow == SHOWpause)
      { 
        stroke(128);
        fill(0);
        rect(x+2,                y, buttonSize/4, buttonSize);
        rect(x+2*buttonSize/3-2, y, buttonSize/4, buttonSize);
      }  
    else
    if (whatToShow == SHOWrun)
      { 
        stroke(80, 255, 80);
        fill(0,255,0);
        triangle(x,y, x+buttonSize,y+buttonSize/2, x,y+buttonSize);
      }  
  }
} // StartPauseStopButton

class RestartButton extends Button
{ 
  int cx, cy;
  
  RestartButton(int ix, int iy, int isize, 
                int icolor, int ihighlight, int ibackground) 
  {
    x = ix;                // x and y are the upper left corner
    y = iy;
    buttonSize = isize;
    cx = x + buttonSize/2; // cx and cy will be the center
    cy = y + buttonSize/2;
    ibasecolor = icolor;
    ihighlightcolor = ihighlight;
    currentcolor = color(ibasecolor);
    ibackgroundcolor = ibackground;
  }

  boolean over() 
  {
    if ( overCircle(cx, cy, buttonSize) ) 
      {
        over0 = true;
        return true;
      } 
    else 
      {
        over0 = false;
        return false;
      }
  }

  void display() 
  {
    stroke(0);
    noFill();
    arc(cx, cy, buttonSize, buttonSize, PI/2, 2*PI);
    arc(cx, cy, buttonSize-10, buttonSize-10, PI/2, 2*PI);
    line(x+buttonSize-3, cy+5, x+buttonSize+4,  cy-3);
    line(x+buttonSize-3, cy+5, x+buttonSize-10, cy-3);
    line(cx, y+buttonSize-4, cx, y+buttonSize);
  }
} // RestartButton

class HScrollbar
{
  int     swidth, sheight;  // width and height of bar
  int     xpos, ypos;       // x and y position of bar
  float   spos, newspos;    // x position of slider
  int     sposMin, sposMax; // max and min values of slider
  int     loose;            // how loose/heavy
  boolean over0;            // is the mouse over the slider?
  boolean locked0;
  float   ratio;
  int     cyclesLeft = 9999;

  HScrollbar (int xp, int yp, int sw, int sh, int l) 
  {
    swidth = sw;
    sheight = sh;
    int widthtoheight = sw - sh;
    ratio = (float)sw / (float)widthtoheight;
    xpos = xp;
    ypos = yp-sheight/2;
    //spos = xpos + swidth/2 - sheight/2;
    spos = xpos;
    newspos = spos;
    sposMin = xpos;
    sposMax = xpos + swidth - sheight;
    loose = l;
  }

  boolean update(int i) 
  {
    float oldNewspos = newspos;
    
    cyclesLeft = i;
      
    if (over()) 
       over0 = true;
    else 
       over0 = false;
        
    if (mousePressed && over0) 
       locked0 = true;
    
    if (!mousePressed) 
       locked0 = false;
    
    if (locked0) 
       newspos = constrain(mouseX-sheight/2, sposMin, sposMax);
    
    if (abs(newspos - spos) > 1) 
       spos = spos + (newspos-spos)/loose;
       
    return ( oldNewspos != newspos );
  }

  int constrain (int val, int minv, int maxv) 
  { return min(max(val, minv), maxv); }

  boolean over() 
  {
    over0 = (mouseX > xpos && mouseX < xpos+swidth && 
             mouseY > ypos && mouseY < ypos+sheight);
    return over0;
  }
  
  boolean locked()
  { 
    locked0 = over0 && mousePressed;
    return locked0; 
  }

  void display() 
  { 
    fill(255);
    rect(xpos, ypos, swidth, sheight);
    
    boolean quiescing = false;
    if ( cyclesLeft < 50 )
      {
        if (over0 || locked0)
          {
            float f = ((float)cyclesLeft)/50.0; // 1.0-0.0 .. fade-out over 50 cycles
            f = 102 + f*(200-102);
            fill(f,102,0);
            quiescing = true;
          }
      }
      
    if (!quiescing)  
      { if (over0 || locked0) 
           fill(200, 102, 0);
        else 
           fill(102, 102, 102);
      }     
    rect(spos, ypos, sheight, sheight);
  }

  float getPos() 
  { return spos * ratio; } // 0 - total width of the scrollbar
  
  float getValue()
  { return (newspos-xpos)/(swidth-sheight); } // 0.0 - 1.0
} // HScrollbar
