import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
//import saito.objloader.*;
//import g4p_controls.*;

HScrollbar hs1;

int MAX_ITERATION = 8000;

Float distance = 0.0f;
Float heading = 0.0f;
Float xspeed_target = 0.0f;
Float xspeed_actual = 0.0f;
Float wspeed_target = 0.0f;
Float wspeed_actual = 0.0f;
Float front_wall_pid = 0.0f;
Float wall_pid = 0.0f;
Float wall_actual = 0.0f;
Float dl = 0.0f;
Float fl = 0.0f;
Float fr = 0.0f;
Float dr = 0.0f;
Float x_front_wall_target = 0.0f;
Float x_front_wall_actual = 0.0f;
Float w_front_wall_target = 0.0f;
Float w_front_wall_actual = 0.0f;
Float front_wall_detect = 0.0f;
Float left_wall_detect = 0.0f;
Float right_wall_detect = 0.0f;

int iteration  = 0;
Float[] distance_history = new Float[8000];
Float[] heading_history = new Float[8000];
Float[] xspeed_target_history = new Float[8000];
Float[] xspeed_actual_history = new Float[8000];
Float[] wspeed_target_history = new Float[8000];
Float[] wspeed_actual_history = new Float[8000];
Float[] front_wall_pid_history = new Float[8000];
Float[] wall_pid_history = new Float[8000];
Float[] wall_actual_history = new Float[8000];
Float[] dl_history = new Float[8000];
Float[] fl_history = new Float[8000];
Float[] fr_history = new Float[8000];
Float[] dr_history = new Float[8000];
Float[] x_front_wall_target_history = new Float[8000];
Float[] x_front_wall_actual_history = new Float[8000];
Float[] w_front_wall_target_history = new Float[8000];
Float[] w_front_wall_actual_history = new Float[8000];
Float[] front_wall_detect_history = new Float[8000];
Float[] left_wall_detect_history = new Float[8000];
Float[] right_wall_detect_history = new Float[8000];

Float distance_zoom = 1000.0f; // OK => 0..180mm >> 180pixels 
Float heading_zoom = 0.4f;
Float xspeed_zoom = 100.0f; // 0..1m/s ==> 0..100pixels
Float wspeed_zoom = 0.33f;
Float wall_pid_zoom = 20.0f;
Float wall_actual_zoom = 3.0f;
Float dx_zoom = 1.0f;
Float fx_zoom = 0.5f;
Float x_front_wall_zoom = 2.0f;
Float w_front_wall_zoom = 4.0f;

Float zoom = 1.0f;

Float distance_offset = 200.0f;

Float xspeed_target_offset = 300.0f;
Float xspeed_actual_offset = 300.0f;
Float x_front_wall_offset = 300.0f;

Float heading_offset = 500.0f; 
Float wspeed_target_offset = 500.0f;
Float wspeed_actual_offset = 500.0f;
Float wall_pid_offset = 500.0f;
Float wall_actual_offset = 500.0f;
Float w_front_wall_offset = 500.0f;

Float dx_offset = 800.0f;
Float fx_offset = 1020.0f;

PFont font;

// Serial port state.
Serial       port;

void setup()
{
  size(1900, 1060, OPENGL);
  frameRate(30);
  font = createFont("ArialMT", 48, true);
  // Open port.
  port = new Serial(this, "COM4", 115200);
  //port = new Serial(this, "COM3", 115200);
  port.bufferUntil('\n');
  // scrollbar  
  hs1 = new HScrollbar(0, height-8, width, 16, 16);
}
 
void draw()
{
  background(255);
  {
    float index = hs1.getPos(); 
    int xpos = 0;
    for (int i = (int)((float)iteration*index/1900.0); i < (int)((float)iteration*index/1900.0)+1900; i++) 
    {
      if(i<iteration-1)
      {
        // distance
        stroke(0,0,0);
        strokeWeight(1);
        line (0, distance_offset, 0, 1900, distance_offset, 0);
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
        strokeWeight(1);
        line (0, heading_offset, 0, 1900, heading_offset, 0);
        if((i%20)==0)
        {  
          stroke(255,0,0);
          strokeWeight(3);
          point(xpos,heading_offset-heading_history[i]*heading_zoom,0); 
          String dist =  (int)(heading_history[i]*1.0)+" ";
          stroke(255,0,0);
          fill(0,0,0);
          textFont(font, 9);
          text(dist, xpos, heading_offset-heading_history[i]*heading_zoom-10);
        }
        else
        {  
          stroke(0,0,0);
          strokeWeight(2);
          point(xpos,heading_offset-heading_history[i]*heading_zoom,0); 
        }        

        // xspeedor front wall x position
        stroke(0,0,0);
        strokeWeight(1);
        line (0, xspeed_target_offset, 0, 1900, xspeed_target_offset, 0);
        if(front_wall_pid_history[i]==0)
        {
          stroke(0,0,0);
          strokeWeight(2);
          point(xpos,xspeed_target_offset-xspeed_target_history[i]*xspeed_zoom,0); 
          stroke(0,0,255);
          strokeWeight(3);
          point(xpos,xspeed_actual_offset-xspeed_actual_history[i]*xspeed_zoom,0); 
        }
        else
        {
          stroke(0,0,0);
          strokeWeight(2);
          point(xpos,x_front_wall_offset-x_front_wall_target_history[i]*x_front_wall_zoom,0); 
          stroke(0,255,0);
          strokeWeight(3);
          point(xpos,x_front_wall_offset-x_front_wall_actual_history[i]*x_front_wall_zoom,0); 
        }

         // wspeed or wall_following pid or front wall w position
        stroke(0,0,0);
        strokeWeight(1);
        line (0, wspeed_target_offset, 0, 1900, wspeed_target_offset, 0);
        if(front_wall_pid_history[i]==0)
        {
          if(wall_pid_history[i]==0)
          {
            // wspeed
            stroke(0,0,0);
            strokeWeight(2);
            point(xpos,wspeed_target_offset-wspeed_target_history[i]*wspeed_zoom,0); 
            // wspeed
            stroke(0,0,255);
            strokeWeight(3);
            point(xpos,wspeed_actual_offset-wspeed_actual_history[i]*wspeed_zoom,0); 
          }
          else
          {
            // wall following
            stroke(255,0,0);
            strokeWeight(3);
            point(xpos,wall_actual_offset-wall_actual_history[i]*wall_actual_zoom,0);
          }
        }
        else
        {
          stroke(0,0,0);
          strokeWeight(2);
          point(xpos,w_front_wall_offset-w_front_wall_target_history[i]*w_front_wall_zoom,0); 
          stroke(0,255,0);
          strokeWeight(3);
          point(xpos,w_front_wall_offset-w_front_wall_actual_history[i]*w_front_wall_zoom,0);
        }
        
        // dx
        
        // refence 0
        stroke(0,0,0);
        strokeWeight(1);
        line (0, dx_offset, 0, 1900, dx_offset, 0);
        // side wall distance detection
        stroke(200,200,200);
        strokeWeight(1);
        point(xpos,dx_offset-(100.0)*dx_zoom,0);  // SIDE_WALL_DISTANCE
        point(xpos,dx_offset-(140.0)*dx_zoom,0); // wall to no wall 
        stroke(200,0,0);
        strokeWeight(1);
        point(xpos,dx_offset-(40.0)*dx_zoom,0); // RIGHT_WALL_DISTANCE_NO_SIDE_ERROR
        stroke(0,200,0);
        strokeWeight(1);
        point(xpos,dx_offset-(40.0)*dx_zoom,0); // LEFT_WALL_DISTANCE_NO_SIDE_ERROR
        // plot
        if((i%20)==0)
        { 
            stroke(0,0,0);
            strokeWeight(3);
            point(xpos,dx_offset-dl_history[i]*dx_zoom,0); 
            stroke(0,0,0);
            strokeWeight(3);
            point(xpos,dx_offset-dr_history[i]*dx_zoom,0);
            // dl
            String dist_dl =  (int)(dl_history[i]+1.0)+" ";
            stroke(0,200,0);
            fill(0,200,0);
            textFont(font, 10);
            text(dist_dl, xpos, dx_offset+9);
            // dr
            String dist_dr =  (int)(dr_history[i]+1.0)+" ";
            stroke(200,0,0);
            fill(200,0,0);
            textFont(font, 10);
            text(dist_dr, xpos, dx_offset+19);        
        }
        else
        {
          stroke(0,255,0);
          if( left_wall_detect_history[i] == 1 )
            strokeWeight(3);
          else
            strokeWeight(1);
          point(xpos,dx_offset-dl_history[i]*dx_zoom,0); 
          stroke(255,0,0);
          if( right_wall_detect_history[i] == 1 )
            strokeWeight(3);
          else
            strokeWeight(1);
          point(xpos,dx_offset-dr_history[i]*dx_zoom,0);
        }
        
        // fx
        
        // refence 0
        stroke(0,0,0);
        strokeWeight(1);
        line (0, fx_offset, 0, 1900, fx_offset, 0);
        // front wall distance destection
        stroke(200,200,200);
        strokeWeight(1);
        point(xpos,fx_offset-(350.0)/2.0*fx_zoom,0); // FRONT_WALL_DISTANCE
        point(xpos,fx_offset-(240.0)/2.0*fx_zoom,0); // WALL_FRONT_ANGLE_TURNING_mm
        point(xpos,fx_offset-(26.0)*fx_zoom,0); // WALL_FRONT_DISTANCE_mm
        // plot
        stroke(0,255,0);
        strokeWeight(1);
        point(xpos,fx_offset-fl_history[i]*fx_zoom,0); 
        stroke(255,0,0);
        strokeWeight(1);
        point(xpos,fx_offset-fr_history[i]*fx_zoom,0);
        // values
          if((i%20)==0)
          { 
            // mean fr+fl
            stroke(255,0,0);
            strokeWeight(3);
            point(xpos,fx_offset-(fr_history[i]+fl_history[i])/2.0*fx_zoom,0); 
            // fl
            String dist_fl =  (int)(fl_history[i]+1.0)+" ";
            stroke(0,200,0);
            fill(0,200,0);
            textFont(font, 10);
            text(dist_fl, xpos, fx_offset+9);
            //fr
            String dist_fr =  (int)(fr_history[i]+1.0)+" ";
            stroke(200,0,0);
            fill(200,0,0);
            textFont(font, 10);
            text(dist_fr, xpos, fx_offset+19);
            // sum
            String dist =  (int)(fr_history[i]+fl_history[i])+" ";
            stroke(200,200,200);
            fill(0,0,0);
            textFont(font, 10);
            text(dist, xpos, fx_offset-(fr_history[i]+fl_history[i])/2.0*fx_zoom-10);
            
          }
          else
          {
            // mean fr+fl
            stroke(0,0,0);
            if( front_wall_detect_history[i] == 1 )
              strokeWeight(3);
            else
              strokeWeight(1);
            point(xpos,fx_offset-(fr_history[i]+fl_history[i])/2.0*fx_zoom,0); 
          }            
      }
      ++xpos;
    }
    // legend
    String legend_d =  "Distance(cm)";
    stroke(0,0,0);
    fill(0,0,0);
    textFont(font, 15);
    text(legend_d, 5, distance_offset+15);    
    // legend
    String legend_x =  "xPID(B) FrontWall(G):";
    stroke(0,0,0);
    fill(0,0,0);
    textFont(font, 15);
    text(legend_x, 5, xspeed_actual_offset+15);    
    // legend
    String legend_w =  "wPID(B) SideWall(R) FrontWall(G)";
    stroke(0,0,0);
    fill(0,0,0);
    textFont(font, 15);
    text(legend_w, 5, wspeed_actual_offset+20);    
    // legend
    String legend_dx =  "DL(green) DR(red)";
    stroke(0,0,0);
    fill(0,0,0);
    textFont(font, 15);
    text(legend_dx, 5, dx_offset-10);    
    // legend
    String legend_fx =  "FL(G) FR(R) Sum(B)";
    stroke(0,0,0);
    fill(0,0,0);
    textFont(font, 15);
    text(legend_fx, 5, fx_offset-10);    
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
      if ( list.length >= 19  ) 
      {
        iteration = int(list[1]);
        distance = float(list[3])/1000.0f;
        heading = float(list[4])/1.0f;
        xspeed_target = float(list[5])/1000.0f;
        xspeed_actual = float(list[6])/1000.0f;
        wspeed_target = float(list[7])/1.0f;
        wspeed_actual = float(list[8])/1.0f;
        wall_actual = float(list[9])/1.0f;
        x_front_wall_target = float(list[10])/1.0f;
        x_front_wall_actual = float(list[11])/1.0f;
        w_front_wall_target = float(list[12])/1.0f;
        w_front_wall_actual = float(list[13])/1.0f;
        dl = float(list[14])/1.0f;
        fl = float(list[15])/1.0f;
        fr = float(list[16])/1.0f;
        dr = float(list[17])/1.0f;
        left_wall_detect = float((int(list[18])>>0)&0x01);
        front_wall_detect = float((int(list[18])>>1)&0x01);
        right_wall_detect = float((int(list[18])>>2)&0x01);
        wall_pid = float((int(list[18])>>3)&0x01);
        front_wall_pid = float((int(list[18])>>4)&0x01);

        distance_history[iteration] = distance;
        heading_history[iteration] = (heading % 360);
        xspeed_target_history[iteration] = xspeed_target;
        xspeed_actual_history[iteration] = xspeed_actual;
        wspeed_target_history[iteration] = wspeed_target;
        wspeed_actual_history[iteration] = wspeed_actual;
        wall_pid_history[iteration] = wall_pid;
        wall_actual_history[iteration] = wall_actual;
        front_wall_pid_history[iteration] = front_wall_pid;
        dl_history[iteration] = dl;
        fl_history[iteration] = fl;
        fr_history[iteration] = fr;
        dr_history[iteration] = dr;
        x_front_wall_target_history[iteration] = x_front_wall_target;
        x_front_wall_actual_history[iteration] = x_front_wall_actual;
        w_front_wall_target_history[iteration] = w_front_wall_target;
        w_front_wall_actual_history[iteration] = w_front_wall_actual;
        left_wall_detect_history[iteration] = left_wall_detect;
        front_wall_detect_history[iteration] = front_wall_detect;
        right_wall_detect_history[iteration] = right_wall_detect;

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
