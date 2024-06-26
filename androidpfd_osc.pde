import org.firmata.*;
import cc.arduino.*;

import oscP5.*;

OscP5 oscP5;


//variables
float expand = 1.8; // Reno5a-1.8

//Thanks to Adrian Fernandez for the beta-version
//Modified and updated as per latest IDE by Aritro Mukherjee (April,2016)
//Check the detailed tutorial @ www.hackster.io/Aritro
// Sensor used while demonstration (MPU-6050,6DOF)

int W=2400; //My Laptop's screen width 
int H=1080;  //My Laptop's screen height 
float Pitch; 
float Bank; 
float Azimuth; 
float ArtificialHoizonMagnificationFactor=0.7; 
float CompassMagnificationFactor=0.85; 
float SpanAngle=120; 
int NumberOfScaleMajorDivisions; 
int NumberOfScaleMinorDivisions; 
PVector v1, v2; 


float phi;    //Dimensional axis
float theta;
float psi;

void setup() 
{ 
  oscP5 = new OscP5(this, 55555);
  fullScreen(); 
  rectMode(CENTER); 
  smooth(); 
  strokeCap(SQUARE);//Optional 
  orientation(LANDSCAPE);
}
void draw() 
{ 
  background(0); 
  translate(W/4, H/2);
  scale(expand);
  MakeAnglesDependentOnMPU6050(); 
  Horizon(); 
  rotate(radians(-Bank)); 
  PitchScale(); 
  Axis(); 
  rotate(radians(Bank)); 
  Borders(); 
  Plane(); 
  ShowAngles(); 
  Compass(); 
  ShowAzimuth();
}
void oscEvent(OscMessage theOscMessage) {
  String[] values = split(theOscMessage.get(2).stringValue(), ",");
  if(values.length == 3)
  {
    phi = float(values[0]);
    theta = float(values[1]); 
    psi = float(values[2]);
  }
}

void MakeAnglesDependentOnMPU6050() 
{ 
  Bank =-phi; 
  Pitch=theta; 
  Azimuth=psi;
}
void Horizon() 
{ 
  scale(ArtificialHoizonMagnificationFactor); 
  //noStroke(); 
  stroke(255,255,255);
  strokeWeight(5);
  fill(0, 180, 255); 
  rect(0, -100, 900, 1000); 
  fill(95, 55, 40); 
  rotate(radians(-Bank)); 
  rect(0, 400+Pitch*10, 900, 800); 
  rotate(radians(Bank)); 
  rotate(-PI-PI/6); 
  SpanAngle=120; 
  NumberOfScaleMajorDivisions=12; 
  NumberOfScaleMinorDivisions=24;  
  CircularScale(); 
  rotate(PI+PI/6); 
  rotate(-PI/6);  
  CircularScale(); 
  rotate(PI/6); 
}
void ShowAzimuth() 
{ 
  fill(50); 
  noStroke(); 
  rect(20, 470, 440, 50); 
  int Azimuth1=round(Azimuth); 
  textAlign(CORNER); 
  textSize(35); 
  fill(255); 
  text("Azimuth:  "+Azimuth1+" Deg", 80, 477, 500, 60); 
  textSize(40);
  fill(25,25,150);
}
void Compass() 
{ 
  translate(1*W/3, 0); 
  scale(CompassMagnificationFactor); 
  noFill(); 
  stroke(100); 
  strokeWeight(80); 
  ellipse(0, 0, 750, 750); 
  strokeWeight(50); 
  stroke(50); 
  fill(0, 0, 40); 
  ellipse(0, 0, 610, 610); 
  for (int k=255;k>0;k=k-5) 
  { 
    noStroke(); 
    fill(0, 0, 255-k); 
    ellipse(0, 0, 2*k, 2*k); 
  } 
  strokeWeight(20); 
  NumberOfScaleMajorDivisions=18; 
  NumberOfScaleMinorDivisions=36;  
  SpanAngle=180; 
  CircularScale(); 
  rotate(PI); 
  SpanAngle=180; 
  CircularScale(); 
  rotate(-PI); 
  fill(255); 
  textSize(60); 
  textAlign(CENTER); 
  text("L", -375, 0, 100, 80); 
  text("R", 370, 0, 100, 80); 
  text("16", 0, -365, 100, 80); 
  text("34", 0, 375, 100, 80); 
  textSize(30); 
  text("COMPASS", 0, -130, 500, 80); 
  rotate(PI/4); 
  textSize(40); 
  text("NW", -370, 0, 100, 50); 
  text("SE", 365, 0, 100, 50); 
  text("NE", 0, -355, 100, 50); 
  text("SW", 0, 365, 100, 50); 
  rotate(-PI/4); 
  CompassPointer(); 
}
void CompassPointer() 
{ 
  rotate(PI+radians(Azimuth)-radians(160));  
  stroke(0); 
  strokeWeight(4); 
  fill(100, 255, 100); 
  triangle(-20, -210, 20, -210, 0, 270); 
  triangle(-15, 210, 15, 210, 0, 270); 
  ellipse(0, 0, 45, 45);   
  fill(0, 0, 50); 
  noStroke(); 
  ellipse(0, 0, 10, 10); 
  triangle(-20, -213, 20, -213, 0, -190); 
  triangle(-15, -215, 15, -215, 0, -200); 
  rotate(-PI-radians(Azimuth)+radians(160)); 
}
void Plane() 
{ 
  fill(0); 
  strokeWeight(1); 
  stroke(0, 255, 0); 
  triangle(-20, 0, 20, 0, 0, 25); 
  rect(110, 0, 140, 20); 
  rect(-110, 0, 140, 20); 
}
void CircularScale() 
{ 
  float GaugeWidth=800;  
  textSize(GaugeWidth/30); 
  float StrokeWidth=1; 
  float an; 
  float DivxPhasorCloser; 
  float DivxPhasorDistal; 
  float DivyPhasorCloser; 
  float DivyPhasorDistal; 
  strokeWeight(2*StrokeWidth); 
  stroke(255);
  float DivCloserPhasorLenght=GaugeWidth/2-GaugeWidth/9-StrokeWidth; 
  float DivDistalPhasorLenght=GaugeWidth/2-GaugeWidth/7.5-StrokeWidth;
  for (int Division=0;Division<NumberOfScaleMinorDivisions+1;Division++) 
  { 
    an=SpanAngle/2+Division*SpanAngle/NumberOfScaleMinorDivisions;  
    DivxPhasorCloser=DivCloserPhasorLenght*cos(radians(an)); 
    DivxPhasorDistal=DivDistalPhasorLenght*cos(radians(an)); 
    DivyPhasorCloser=DivCloserPhasorLenght*sin(radians(an)); 
    DivyPhasorDistal=DivDistalPhasorLenght*sin(radians(an));   
    line(DivxPhasorCloser, DivyPhasorCloser, DivxPhasorDistal, DivyPhasorDistal); 
  }
  DivCloserPhasorLenght=GaugeWidth/2-GaugeWidth/10-StrokeWidth; 
  DivDistalPhasorLenght=GaugeWidth/2-GaugeWidth/7.4-StrokeWidth;
  for (int Division=0;Division<NumberOfScaleMajorDivisions+1;Division++) 
  { 
    an=SpanAngle/2+Division*SpanAngle/NumberOfScaleMajorDivisions;  
    DivxPhasorCloser=DivCloserPhasorLenght*cos(radians(an)); 
    DivxPhasorDistal=DivDistalPhasorLenght*cos(radians(an)); 
    DivyPhasorCloser=DivCloserPhasorLenght*sin(radians(an)); 
    DivyPhasorDistal=DivDistalPhasorLenght*sin(radians(an)); 
    if (Division==NumberOfScaleMajorDivisions/4|Division==NumberOfScaleMajorDivisions*3/4) //bank angle green line
    { 
      strokeWeight(15); 
      stroke(0); 
      line(DivxPhasorCloser, DivyPhasorCloser, DivxPhasorDistal, DivyPhasorDistal); 
      strokeWeight(8); 
      stroke(100, 255, 100); 
      line(DivxPhasorCloser, DivyPhasorCloser, DivxPhasorDistal, DivyPhasorDistal); 
    } 
    else if (Division ==0|Division ==NumberOfScaleMajorDivisions)
    {
      strokeWeight(15); 
      stroke(0); 
      line(DivxPhasorCloser, DivyPhasorCloser, DivxPhasorDistal, DivyPhasorDistal); 
      strokeWeight(8); 
      stroke(255, 0, 0); 
      line(DivxPhasorCloser, DivyPhasorCloser, DivxPhasorDistal, DivyPhasorDistal); 
    }
    else if (Division == NumberOfScaleMajorDivisions/2)
    {
      strokeWeight(15); 
      stroke(0); 
      line(DivxPhasorCloser, DivyPhasorCloser, DivxPhasorDistal, DivyPhasorDistal); 
      strokeWeight(8); 
      stroke(255); 
      line(DivxPhasorCloser, DivyPhasorCloser, DivxPhasorDistal, DivyPhasorDistal); 
    }
    { 
      strokeWeight(3); 
      stroke(255); 
      line(DivxPhasorCloser, DivyPhasorCloser, DivxPhasorDistal, DivyPhasorDistal); 
    } 
  } 
}
void Axis() 
{ 
  stroke(255, 0, 0); 
  strokeWeight(3); 
  line(-115, 0, 115, 0); 
  line(0, 280, 0, -280); 
  fill(100, 255, 100); 
  stroke(0); 
  triangle(0, -285, -10, -255, 10, -255); 
  triangle(0, 285, -10, 255, 10, 255); 
}
void ShowAngles() 
{ 
  textSize(30); 
  fill(50); 
  noStroke(); 
  rect(-150, 400, 280, 40); 
  rect(150, 400, 280, 40); 
  fill(255);
  float Pitch1 = round(Pitch*10);
  float Bank1 = round(Bank*10);
  
  Pitch1 /= 10;
  Bank1 /= 10;
  
  text("Pitch:  "+Pitch1+" Deg", -20, 411, 500, 60); 
  text("Bank:  "+Bank1+" Deg", 280, 411, 500, 60); 
}
void Borders() 
{ 
  noFill(); 
  stroke(0); 
  strokeWeight(400); 
  rect(0, 0, 1100, 1100); 
  strokeWeight(200); 
  ellipse(0, 0, 1000, 1000); 
  fill(0); 
  noStroke(); 
  rect(4*W/5, 0, W, 2*H); 
  rect(-4*W/5, 0, W, 2*H); 
}
void PitchScale() 
{  
  stroke(255); 
  fill(255); 
  strokeWeight(3); 
  textSize(24); 
  textAlign(CENTER); 
  for (int i=-2;i<3;i++) 
  {  
    if ((i==0)==false) 
    { 
      line(110, 100*i, -110, 100*i); 
    }  
    text(""+i*10, 140, 100*i, 100, 30); 
    text(""+i*10, -140, 100*i, 100, 30); 
  } 
  textAlign(CORNER); 
  strokeWeight(2); 
  for (int i=-3;i<4;i++) 
  {  
    if ((i==0)==false) 
    {    
      line(25, 50*i, -25, 50*i); 
    } 
  } 
}
