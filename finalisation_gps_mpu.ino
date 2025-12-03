#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h> 

#define SDA 21
#define SCL 22
#define RX 16
#define TX 17
HardwareSerial GPS(2);
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
// params
const int FS_HZ = 50;
const int N = 25; // parametre du fenêtrage

// Seuil ,ils sont fixeen se basant sur le test avec notre capteur ici MPU6050 donc relative a la precision de chaque capteur
const float ALIM_TH = 1.8; // g // normalized unit
const float ANGLE_DELTA_TH = 60.0;
const float ANGLE_ROLL_TH = 100.0; 
const float ANGLE_PITCH_TH = 50.0;// degrees
const float ANGLE_BETA_TH=100;
const float MODULE_TH=10;

// buffers (circular)
float axBuf[N], ayBuf[N], azBuf[N];
float gxBuf[N], gyBuf[N], gzBuf[N];

// initialisation de la gravite
float gravX=0, gravY=0, gravZ=0;
const float alphaLPF = 0.98; // coefficient du filtre complémentaire 

unsigned long lastMillis = 0;
int oldest = 0;
int newest = N-1;

void setup(){
  Serial.begin(115200);
  GPS.begin(9600, SERIAL_8N1,  RX, TX);
  Wire.begin(SDA,SCL);
  while (!mpu.begin()) {
      delay(10);
    }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
  lastMillis = millis();
}

sensors_event_t a, g, temp;
float ax,ay,az,gx,gy,gz,lax,lay,laz,pitch,roll,alim,mag1,mag2,dot,angle_deg;
double lat,lng;
int votes;
bool f_roll,f_pitch,fa_angle,f_alim,fg_angle,f_module;

void loop(){
  // run at ~FS_HZ
  if(millis() - lastMillis < (1000/FS_HZ)) return;
  lastMillis = millis();

  
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x ; ay = a.acceleration.y ; az = a.acceleration.z ;
  gx = g.gyro.x ; gy = g.gyro.y ; gz = g.gyro.z ;

  // accel linéaire 
  lax = ax - gravX; lay = ay - gravY; laz = az - gravZ;
  
// LPF gravity estimate on accelerometer (for Alim)
  gravX = alphaLPF *ax ;gravY = alphaLPF *ay ; gravZ = alphaLPF *az ;
  
  // store in buffers
  for(int i=N-1;i>0;i--)
  {
     axBuf[i] = axBuf[i-1] ; ayBuf[i] = ayBuf[i-1] ;azBuf[i] = azBuf[i-1] ;
  gxBuf[i] = gxBuf[i-1]; gyBuf[i] = gyBuf[i-1]; gzBuf[i] = gzBuf[i-1];
  }
  axBuf[0] = ax; ayBuf[0] = ay; azBuf[0] = az;
  gxBuf[0] = gx; gyBuf[0] = gy; gzBuf[0] = gz;

  // pitch : inclinaison avant/arriere
  pitch = atan2(-ax,sqrt(ay * ay + az * az)) * 180.0 / PI;
  f_pitch = (fabs(pitch) > ANGLE_PITCH_TH);

  // roll : inclinaison gauche/droite
  roll  = atan2(ay,az) * 180.0 / PI;
  f_roll = (fabs(roll) > ANGLE_ROLL_TH);

  // Alim 
  alim = sqrt(lax*lax + lay*lay + laz*laz);
  f_alim = (alim > ALIM_TH);

// Angle delta ∠α
// angle entre vecteurs v1 and v2:
  dot = (axBuf[newest]*axBuf[oldest] + ayBuf[newest]*ayBuf[oldest] + azBuf[newest]*azBuf[oldest]);
  mag1 = sqrt(axBuf[newest]*axBuf[newest] + ayBuf[newest]*ayBuf[newest] + azBuf[newest]*azBuf[newest]);
  mag2 = sqrt(axBuf[oldest]*axBuf[oldest] + ayBuf[oldest]*ayBuf[oldest] + azBuf[oldest]*azBuf[oldest]);
  angle_deg = 0;
  if(mag1*mag2 > 0.0001) {
    float c = dot / (mag1*mag2);
    if(c>1) c=1; if(c<-1) c=-1;
    angle_deg = acos(c) * 180.0 / M_PI;
  }
  fa_angle = (angle_deg > ANGLE_DELTA_TH);

//angle beta
  float dot2 = (gxBuf[newest]*gxBuf[oldest] + gyBuf[newest]*gyBuf[oldest] + azBuf[newest]*azBuf[oldest]);
  float mag3 = sqrt(gxBuf[newest]*gxBuf[newest] + gyBuf[newest]*gyBuf[newest] + azBuf[newest]*azBuf[newest]);
  float module = sqrt(gxBuf[oldest]*gxBuf[oldest] + gyBuf[oldest]*gyBuf[oldest] + azBuf[oldest]*azBuf[oldest]);
  float g_angle = 0;
  if(mag3*module > 0.0001) {
    float c1 = dot2 / (mag3*module);
    if(c1>1) c1=1; if(c1<-1) c1=-1;
    g_angle = acos(c1) * 180.0 / M_PI;
  }
 fg_angle = (g_angle > ANGLE_BETA_TH);
 f_module=(module<MODULE_TH);

  // Decision
  votes = (f_pitch?1:0) +(f_roll?1:0)+ (f_alim?1:0) + (fa_angle?1:0)+(fg_angle?1:0) +(f_module?1:0) ;
  if(votes >= 4) {
    Serial.println("FALL_CANDIDATE detected, running post-check...");
    // Alarme
    Post_Confirmation()?Serial.println("FALL CONFIRMED!"):Serial.println("FALL REJECTED!");

  }
  else {Serial.println("Candidate in NORMAL_STATE ");}

  //GPS
  while (GPS.available()) {
    char c=GPS.read();
    gps.encode(c);
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lng = gps.location.lng();
  }
  }
  sendAlert(pitch, roll,alim, angle_deg, g_angle, module, votes, lat, lng );
  delay(4000);
}

bool Post_Confirmation()
{
  int cpt=0;
  while(votes >= 3 && cpt<8)
  {
  mpu.getEvent(&a, &g, &temp);
  
  ax = a.acceleration.x ;ay = a.acceleration.y ;az = a.acceleration.z ;
  gx = g.gyro.x ;gy = g.gyro.y ;gz = g.gyro.z ;
  
   for(int i=N-1;i>0;i--)
  {
     axBuf[i] = axBuf[i-1] ; ayBuf[i] = ayBuf[i-1] ;azBuf[i] = azBuf[i-1] ;
  gxBuf[i] = gxBuf[i-1]; gyBuf[i] = gyBuf[i-1]; gzBuf[i] = gzBuf[i-1];
  }
  axBuf[0] = ax; ayBuf[0] = ay; azBuf[0] = az;
  gxBuf[0] = gx; gyBuf[0] = gy; gzBuf[0] = gz;

  pitch = atan2(-ax,sqrt(ay * ay + az * az)) * 180.0 / PI;f_pitch = (fabs(pitch) > ANGLE_PITCH_TH);
  roll  = atan2(ay,az) * 180.0 / PI;f_roll = (fabs(roll) > ANGLE_ROLL_TH);
  alim = sqrt(lax*lax + lay*lay + laz*laz); f_alim = (alim > ALIM_TH);

  dot = (axBuf[newest]*axBuf[oldest] + ayBuf[newest]*ayBuf[oldest] + azBuf[newest]*azBuf[oldest]);
  mag1 = sqrt(axBuf[newest]*axBuf[newest] + ayBuf[newest]*ayBuf[newest] + azBuf[newest]*azBuf[newest]);
  mag2 = sqrt(axBuf[oldest]*axBuf[oldest] + ayBuf[oldest]*ayBuf[oldest] + azBuf[oldest]*azBuf[oldest]);
  angle_deg = 0;
  if(mag1*mag2 > 0.0001) {
    float c = dot / (mag1*mag2);
    if(c>1) c=1; if(c<-1) c=-1;
    angle_deg = acos(c) * 180.0 / M_PI;
  }
  fa_angle = (angle_deg > ANGLE_DELTA_TH);
  
  float dot2 = (gxBuf[newest]*gxBuf[oldest] + gyBuf[newest]*gyBuf[oldest] + azBuf[newest]*azBuf[oldest]);
  float mag3 = sqrt(gxBuf[newest]*gxBuf[newest] + gyBuf[newest]*gyBuf[newest] + azBuf[newest]*azBuf[newest]);
  float module = sqrt(gxBuf[oldest]*gxBuf[oldest] + gyBuf[oldest]*gyBuf[oldest] + azBuf[oldest]*azBuf[oldest]);
  float g_angle = 0;
  if(mag3*module > 0.0001) {
    float c1 = dot2 / (mag3*module);
    if(c1>1) c1=1; if(c1<-1) c1=-1;
    g_angle = acos(c1) * 180.0 / M_PI;
  }
 fg_angle = (g_angle > ANGLE_BETA_TH);
 f_module=(module<MODULE_TH);

  votes = (f_pitch?1:0) +(f_roll?1:0)+ (f_alim?1:0) + (fa_angle?1:0)+(fg_angle?1:0) +(f_module?1:0) ;
  cpt++;
  delay(1000);
  }
  bool status=cpt>6?1:0;
return status;
}

void sendAlert(float pitch, float roll,float alim,float angle_deg, float g_angle,float module, int votes,double lat, double lng) 
{
    StaticJsonDocument<200> doc;

    doc["Pitch"] = pitch;
    doc["Roll"] = roll;
    doc["Alim"] =alim;
    doc["ACC_Angle"] = angle_deg;
    doc["GYRO_Angle"] = g_angle;
    doc["module_acc"] = module;
    doc["votes"] = votes;
    

    String link = "https://www.google.com/maps?q=" + String(lat, 6) + "," + String(lng, 6);
    doc["google_maps"] = link;

    String output;
    serializeJsonPretty(doc, output);
    Serial.println(output);
}