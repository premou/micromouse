import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;

HScrollbar hs1;

int MAX_ITERATION = 8000;

Float distance = 0.0f;
Float heading = 0.0f;
Float xspeed_target = 0.0f;
Float xspeed_actual = 0.0f;
Float wspeed_target = 0.0f;
Float wspeed_actual = 0.0f;
Float wall_pid = 0.0f;
Float wall_actual = 0.0f;

Float distance_zoom = 1000.0f;
Float heading_zoom = 1.0f;
Float xspeed_target_zoom = 250.0f;
Float xspeed_actual_zoom = 250.0f;
Float wspeed_target_zoom = 0.33f;
Float wspeed_actual_zoom = 0.33f;
Float wall_pid_zoom = 20.0f;
Float wall_actual_zoom = 1.0f;

int iteration  = 0;
Float[] distance_history = new Float[8000];
Float[] heading_history = new Float[8000];
Float[] xspeed_target_history = new Float[8000];
Float[] xspeed_actual_history = new Float[8000];
Float[] wspeed_target_history = new Float[8000];
Float[] wspeed_actual_history = new Float[8000];
Float[] wall_pid_history = new Float[8000];
Float[] wall_actual_history = new Float[8000];

Float zoom = 1.0f;
Float distance_offset = 200.0f;
Float heading_offset = 200.0f;
Float xspeed_target_offset = 500.0f;
Float xspeed_actual_offset = 500.0f;
Float wspeed_target_offset = 800.0f;
Float wspeed_actual_offset = 800.0f;
Float wall_pid_offset = 650.0f;
Float wall_actual_offset = 800.0f;

PFont font;

// Serial port state.
Serial       port;

void setup()
{
  size(1600, 1000, OPENGL);
  frameRate(30);
  font = createFont("ArialMT", 48, true);
  // Open port.
  port = new Serial(this, "COM3", 115200);
  port.bufferUntil('\n');
  // scrollbar  
  hs1 = new HScrollbar(0, height-16, width, 16, 16);
}
 
void draw()
{
  background(255);
  stroke(0,0,0);
  strokeWeight(1);
  line (0, 200, 0, 1600, 200, 0);
  line (0, 500, 0, 1600, 500, 0);
  line (0, 800, 0, 1600, 800, 0);
  //if(iteration>=1600)
  {
    float index = hs1.getPos(); 
    int xpos = 0;
    for (int i = (int)((float)iteration*index/1600.0); i < (int)((float)iteration*index/1600.0)+1600; i++) 
    {
      if(i<iteration-1)
      {
        // distance
        if((i%20)==0)
        {  
          stroke(255,0,0);
          strokeWeight(3);
          point(xpos,distance_offset-distance_history[i]*distance_zoom,0);
          String dist =  (int)(distance_history[i]*1000.0)+" ";
          stroke(255,0,0);
          fill(0,0,0);
          textFont(font, 9);
          text(dist, xpos, distance_offset-distance_history[i]*distance_zoom-10);
        }
        else
        {
          stroke(0,0,0);
          strokeWeight(2);
          point(xpos,distance_offset-distance_history[i]*distance_zoom,0);
        }
        
        // heading
        stroke(0,0,0);
        strokeWeight(2);
        point(xpos,heading_offset-heading_history[i]*heading_zoom,0); 
  
        // xspeed
        stroke(0,0,0);
        strokeWeight(2);
        point(xpos,xspeed_target_offset-xspeed_target_history[i]*xspeed_target_zoom,0); 
        // xspeed
        stroke(0,0,255);
        strokeWeight(3);
        point(xpos,xspeed_actual_offset-xspeed_actual_history[i]*xspeed_actual_zoom,0); 
        
        // wspeed
        stroke(0,0,0);
        strokeWeight(2);
        point(xpos,wspeed_target_offset-wspeed_target_history[i]*wspeed_target_zoom,0); 
        // wspeed
        stroke(0,0,255);
        strokeWeight(3);
        point(xpos,wspeed_actual_offset-wspeed_actual_history[i]*wspeed_actual_zoom,0); 
        
        // wall pid
        stroke(0,0,0);
        strokeWeight(2);
        point(xpos,wall_pid_offset-wall_pid_history[i]*wall_pid_zoom,0); 
        // wall position
        stroke(255,0,0);
        strokeWeight(2);
        point(xpos,wall_actual_offset-wall_actual_history[i]*wall_actual_zoom,0); 
      }
      ++xpos;
    }
  }
  
  hs1.update();
  hs1.display();
}

void serialEvent(Serial p) 
{
  String incoming = p.readString();
    println(incoming);
  
  if ((incoming.length() > 8))
  {
    String[] list = split(incoming, " ");
      if ( list.length >= 20  ) 
      {
        iteration = int(list[1]);
        distance = float(list[3])/1000.0f;
        heading = float(list[4])/1.0f;
        xspeed_target = float(list[5])/1000.0f;
        xspeed_actual = float(list[6])/1000.0f;
        wspeed_target = float(list[7])/1.0f;
        wspeed_actual = float(list[8])/1.0f;
        wall_pid = float(list[9])/1.0f;
        wall_actual = float(list[10])/1000.0f;

        distance_history[iteration] = distance;
        heading_history[iteration] = heading;
        xspeed_target_history[iteration] = xspeed_target;
        xspeed_actual_history[iteration] = xspeed_actual;
        wspeed_target_history[iteration] = wspeed_target;
        wspeed_actual_history[iteration] = wspeed_actual;
        wall_pid_history[iteration] = wall_pid;
        wall_actual_history[iteration] = wall_actual;

       //++iteration;
    }
  }
}

class HScrollbar {
  int swidth, sheight;    // width and height of bar
  float xpos, ypos;       // x and y position of bar
  float spos, newspos;    // x position of slider
  float sposMin, sposMax; // max and min values of slider
  int loose;              // how loose/heavy
  boolean over;           // is the mouse over the slider?
  boolean locked;
  float ratio;

  HScrollbar (float xp, float yp, int sw, int sh, int l) {
    swidth = sw;
    sheight = sh;
    int widthtoheight = sw - sh;
    ratio = (float)sw / (float)widthtoheight;
    xpos = xp;
    ypos = yp-sheight/2;
    spos = xpos;// + swidth/2 - sheight/2;
    newspos = spos;
    sposMin = xpos;
    sposMax = xpos + swidth - sheight;
    loose = l;
  }

  void update() {
    if (overEvent()) {
      over = true;
    } else {
      over = false;
    }
    if (mousePressed && over) {
      locked = true;
    }
    if (!mousePressed) {
      locked = false;
    }
    if (locked) {
      newspos = constrain(mouseX-sheight/2, sposMin, sposMax);
    }
    if (abs(newspos - spos) > 1) {
      spos = spos + (newspos-spos)/loose;
    }
  }

  float constrain(float val, float minv, float maxv) {
    return min(max(val, minv), maxv);
  }

  boolean overEvent() {
    if (mouseX > xpos && mouseX < xpos+swidth &&
       mouseY > ypos && mouseY < ypos+sheight) {
      return true;
    } else {
      return false;
    }
  }

  void display() {
    noStroke();
    fill(204);
    rect(xpos, ypos, swidth, sheight);
    if (over || locked) {
      fill(0, 0, 0);
    } else {
      fill(102, 102, 102);
    }
    rect(spos, ypos, sheight, sheight);
  }

  float getPos() {
    // Convert spos to be values between
    // 0 and the total width of the scrollbar
    return spos * ratio;
  }
}