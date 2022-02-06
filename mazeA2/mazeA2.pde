/* library imports *****************************************************************************************************///<>// //<>// //<>// //<>//
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *******************
 
/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/


/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition **************************************************************************/


/* graphical elements */

PShape pGraph, joint, endEffector;

float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* virtual wall parameter  */
float             kWall                               = 450;
PVector           fWall                               = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
PVector           posWall                             = new PVector(0.01, 0.10);



/* pantagraph link parameters in meters */
float             l                                   = 0.07; 
float             L                                   = 0.09;

/* end effector radius in meters */
float             rEE                                 = 0.003;

int unit = 20;//calculate num balls
int count;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
final int         worldPixelWidth                     = 250;
final int         worldPixelHeight                    = 250;

/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/

//line detection
int lArraySize = 119; //determines the size of the array
//line co-ordinates
float[] lx1 = new float[lArraySize];
float[] lx2 = new float[lArraySize];
float[] ly1 = new float[lArraySize];
float[] ly2 = new float[lArraySize];

//background colour
color mazeCol = color(0, 0, 0);

//Module[] mods;

void setup() {
  size(250, 250);
  background(255, 155, 0);

  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard` = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[2], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();

  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);

  widgetOne.device_set_parameters();

  //endeffector set up
  background(0);
  deviceOrigin.add(worldPixelWidth/2, 0);

  /* create pantagraph graphics */
  create_pantagraph();


  /* setup framerate speed */
  //frameRate(baseFrameRate);


  /* setup simulation thread to run at 1kHz */

  /* setup framerate speed */
  frameRate(baseFrameRate);


  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}

void draw() {

  if (renderingForce == false) {
    background(150, 150, 150); 

    update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);
  }
}



void create_pantagraph() {
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  float rEEAni = pixelsPerMeter * rEE;

  endEffector = createShape(ELLIPSE, deviceOrigin.x - 120, deviceOrigin.y - 80, rEEAni/2, rEEAni/2);
  endEffector.setFill(color(255, 0, 50));
  strokeWeight(5);
}

void update_animation(float th1, float th2, float xE, float yE) {
  background(255);


  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;

  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;

  th1 = 3.14 - th1;
  th2 = 3.14 - th2;



  // maze();
  for (int n = 0; n < lArraySize; ++n) {
    strokeWeight(6);
    stroke(mazeCol);
    line(lx1[n], ly1[n], lx2[n], ly2[n]);
  }

  translate(xE, yE);
  shape(endEffector);
}


class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

    renderingForce = true;

    if (haplyBoard.data_available()) {
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();

      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(device_to_graphics(posEE));
    }

    fWall.set(0, 0);
    maze(); 

    fEE = (fWall.copy()).mult(-1);
    fEE.set(graphics_to_device(fEE));
    /* end haptic wall force calculation */

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();


    renderingForce = false;
  }
}

void maze() { 
  int n = 0;
  int mC = 50; 

  if (n < lArraySize) { 
    n = 0;
  }

  //permiter walls
  lx1[n] = 0; 
  ly1[n] = 0; 
  lx2[n] = width; 
  ly2[n] = 0; 
  n = n +1;



  lx1[n] = 0; 
  ly1[n] = 0; 
  lx2[n] = 0; 
  ly2[n] = height; 
  n = n +1;
  penWall.set(((lx1[n] + 0.01) - (posEE.x + rEE)), 0);

  if (penWall.x > 0) {
    fWall = fWall.add(penWall.mult(-kWall));
  }

  lx1[n] = 0; 
  ly1[n] = height; 
  lx2[n] = width-mC*2; 
  ly2[n] = height;
  n = n +1;

  lx1[n] = width; 
  ly1[n] = 0; 
  lx2[n] = width; 
  ly2[n] = height; 
  n = n +1;
  penWall.set(((( (lx1[n]) + 80)/pixelsPerMeter) - (posEE.x + rEE)), 0);

  if (penWall.x < 0) {
    fWall = fWall.add(penWall.mult(-kWall));
  }



  //vertical line 1 
  lx1[n] = mC*3; 
  ly1[n] = mC*0; 
  lx2[n] = mC*3; 
  ly2[n] = mC*2;
  n = n +1;
  penWall.set((((lx1[n] - 10)/pixelsPerMeter) - (posEE.x + rEE)), (posEE.y + rEE));

  if ((penWall.x < -0.041 && penWall.x > -0.046 )&& penWall.y <= 0.049) {
    fWall = fWall.add(penWall.mult(-kWall));
  }



  //y starts on 1
  lx1[n] = mC*0; 
  ly1[n] = mC*1; 
  lx2[n] = mC*2; 
  ly2[n] = mC*1;
  n = n +1;

  penWall.set((posEE.x + rEE), (((ly1[n])/pixelsPerMeter) - (posEE.y + rEE)));
  println("4-" + penWall.x);
  println("5-" + penWall.y);
  if ((penWall.y < -0.024 && penWall.y > -0.027) && penWall.x <= 0.027) {
    fWall = fWall.add(penWall.mult(-kWall));  
    println("addes");
  }

  lx1[n] = mC*1; 
  ly1[n] = mC*2; 
  lx2[n] = mC*1; 
  ly2[n] = mC*3;
  n = n +1;

  lx1[n] = mC*1; 
  ly1[n] = mC*3; 
  lx2[n] = mC*4; 
  ly2[n] = mC*3;
  n = n +1;
}

PVector device_to_graphics(PVector deviceFrame) {
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame) {
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}
/* end simulation section **********************************************************************************************/
