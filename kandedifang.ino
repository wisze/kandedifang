/**
 * Kandedifang viewer and logger
 * june 2015      custom screens (jun20a)
 * july 2015      new pin layout to connect sd card module (jul14a)
 * september 2015 version for handheld with new pin layout
 *                for next and select button (sep05a)
 * oktober 2015   switched to different SD library (oct7a)
 * oktober 2015   started on waypoints, gpx file conversion
 * januari 2016   added waypoint struct and distance computation, 
 *                added waypoint screen, sorted by distance to current location
 * may 2016       single simple satellite screen
 * june 2016      rewrite to native SD methods (TBD)
 *
 * Nokia LCD uses pins:
 * GPIO 3   LED
 * GPIO 10  SCE
 * GPIO 16  DC
 * GPIO 12  SCLK (was 30)
 * GPIO 13  SDIN data in (was 28)
 * GPIO 5   RES
 * V33 Power
 * GND
 *
 * Logger uses pins:
 * GPIO 14  ?
 * GPIO 22  SD
 * GPIO 28  SD
 * GPIO 29  SD
 * GPIO 30  SD
 * GPIO 31  SD
 *
 * V5  Loading battery from USB on board
 * V33 Output power to board
 * GND
 *
 * Buttons:
 * GPIO 4 Select button (red)
 * GPIO 6 Next button (black)
 *
 */

#include "Nokia_5110.h" // PCD8544 Controller
// #include "sti_gnss_lib.h" // GNSS
// #include "GNSS.h" // GNSS
#include "math.h" // fabs
// #include "FatFileSystem.h"  // for SD card
#include "Timer.h"
#include "SD.h"

#define _CHECK_MARK 0xf0  // Check mark

/**
 * At most one of Beidou or Glonass can be selected.
 * This is a precompiler command because you have to
 * use different compiler settings for either
 * Beidou or Glonass
 * GPS status is always shown on the LCD. If you unselect
 * GPS it only means it is not logged in the
 * constellation file
 **/
#define USEGPS     1
#define USEBEIDOU  1
#define USEGLONASS 0
#define USEGALILEO 0 // If only...
const boolean loggps = false;

// Constant values
const int satLed      = 0;
const int blackButton = 2;
const int redButton   = 6;
const int delayLoop   = 5000;
const int distList    = 6;   // Number of locations on the distance list
// const long pressTime  = 200;

/**
 * Location and state variables, updated after each GNSS refresh,
 * previous location, number of points, mean speed, etc.
 **/
double lat, lon, lat0, lon0, alt, alt0;
float pdop, hdop, vdop;
float spd, crs, dist, mnspd;
uint16_t npts, h, m, s, yr, mon, day;
uint16_t nptsbd2, nptsglo, nptsbd23d, nptsglo3d;
double ds, spdsum;

/**
 * Satellite counts, how many do we actually use?
 */
uint16_t gps_view = 0, gps_used = 0, gps_show = 0;
uint16_t bd2_view = 0, bd2_used = 0, bd2_show = 0;
uint16_t glo_view = 0, glo_used = 0, glo_show = 0;

// Display and button state variables:
int blackButtonState     = 0;
int lastBlackButtonState = 0;
int redButtonState       = 0;
int lastRedButtonState   = 0;
int screenMode;
boolean screenLight;
int maxScreens = 3;

/**
 * Folder and file names
 **/
// FAT::FileSystem fatFsAgent;
// FAT::File loc_file, cst_file, wp_file;
SDClass SD;
// File loc_file, cst_file, wp_file;

// FatFileSystem fatFsAgent;
char folderDate[10]  = {0};
char fileTime[10]    = {0};
char currentFolder[64] = {0};
const int sSize = 1024;
char fileLocName[64] = {0};
char fileCstName[64] = {0};
boolean newFile = true;;

// Print buffers
char bufLoc[sSize] = {0}; // Location buffer
char bufCst[sSize] = {0}; // Constellation buffer
char bufWp[sSize]  = {0}; // Waypoint read buffer
UINT len = 0;

// GPS stuff to log
uint16_t gpsPRN[STGNSS_GPS_NCHAN] = {0};
uint16_t gpscnr[STGNSS_GPS_NCHAN] = {0};
uint16_t gpsazi[STGNSS_GPS_NCHAN] = {0};
uint16_t gpsele[STGNSS_GPS_NCHAN] = {0};
// GPS methods
uint16_t GPSStatusLCD(char* str) {
  return sprintf(str, "GPS  %02dV %02dU", gps_view, gps_used);
}
uint16_t GPSPRNLCD(uint16_t sn, char* str) {
  return sprintf(str, "%02d %02d %02d %03d",
                 gpsPRN[sn], gpscnr[sn], gpsele[sn], gpsazi[sn]);
}

#if (USEBEIDOU==1)
// Beidou stuff to log
uint16_t bd2PRN[STGNSS_BD2_NCHAN] = {0};
uint16_t bd2cnr[STGNSS_BD2_NCHAN] = {0};
uint16_t bd2azi[STGNSS_BD2_NCHAN] = {0};
uint16_t bd2ele[STGNSS_BD2_NCHAN] = {0};
// Beidou methods
uint16_t BeidouStatusLCD(char* str) {
  return sprintf(str, "BD2  %02dV %02dU", bd2_view, bd2_used);
}
uint16_t BeidouPRNLCD(uint16_t sn, char* str) {
  return sprintf(str, "%02d %02d %02d %03d",
                 bd2PRN[sn], bd2cnr[sn], bd2ele[sn], bd2azi[sn]);
}
uint16_t BeidouStat(char* str) {
  int ip2d = 0, ip3d = 0;
  if (npts > 0) {
    ip2d = round(100*nptsbd2/npts);
    ip3d = round(100*nptsbd23d/npts);
  } 
  return sprintf(str, "%3d%% 3D %3d%%",ip2d,ip3d);
}
#endif

#if (USEGLONASS==1)
// Glonass stuff to log
uint16_t gloPRN[STGNSS_GLONASS_NCHAN] = {0};
uint16_t glocnr[STGNSS_GLONASS_NCHAN] = {0};
uint16_t gloazi[STGNSS_GLONASS_NCHAN] = {0};
uint16_t gloele[STGNSS_GLONASS_NCHAN] = {0};
// Glonass display methods
uint16_t GlonassStatusLCD(char* str) {
  return sprintf(str, "GLO  %02dV %02dU", glo_view, glo_used);
}
uint16_t GlonassPRNLCD(uint16_t sn, char* str) {
  return sprintf(str, "%02d %02d %02d %03d",
                 gloPRN[sn], glocnr[sn], gloele[sn], gloazi[sn]);
}
uint16_t GlonassStat(char* str) {
  int ip2d = 0, ip3d = 0;
  if (npts > 0) {
    ip2d = round(100*nptsglo/npts);
    ip3d = round(100*nptsglo3d/npts);
  } 
  return sprintf(str, "%3d%% 3D %3d%%",ip2d,ip3d);
  }
}
#endif

/**
 * Waypoint structure and arrau
 * We will use up to 100 waypoints
 */
typedef struct {
  double lat, lon, alt;
  char name[8];
  // Distance to a reference point (subject to constant change)
  double dist, course;
} Waypoint;
const int maxwp = 10; // max number of waypoints
int nwp = 0; // number of actual waypoints
Waypoint wp[maxwp];
int inear[maxwp] = {0,1,2,3,4,5,6,7,8,9}; // Distance index

/** 
 * Format strings to print latitude and longitude to the screen.
 **/
uint16_t LocationLatitudeLCD(char* str) {
  char N_S = 'N';
  double absLatitude = fabs(lat);
  if (lat < 0) {
    N_S = 'S';
  }
  return sprintf(str, "%10.6f %c", absLatitude, N_S);
}
uint16_t LocationLongitudeLCD(char* str) {
  char E_W = 'E';
  double absLongitude = fabs(lon);
  if (lon < 0) {
    E_W = 'W';
  }
  return sprintf(str, "%10.6f %c", absLongitude, E_W);
}

// Check if GPS is in fixed mode
uint8_t CheckMark(void) {
  if (GnssInfo.fixMode() >= 3) {
    return _CHECK_MARK;  // In fixed mode (check mark)
  } else {
    return '?';  // Not in fixed mode
  }
}
// Format String for HDOP status
uint16_t HDOPLCD(char* str) {
  return sprintf(str, "HDOP   %5.2f", hdop);
}
// Format String for HDOP status
uint16_t VDOPLCD(char* str) {
  return sprintf(str, "VDOP   %5.2f", vdop);
}
// Format String for HDOP status
uint16_t PDOPLCD(char* str) {
  return sprintf(str, "PDOP   %5.2f", pdop);
}
// Format string for speed
uint16_t SpeedLCD(char* str) {
  return sprintf(str, "Speed %6.2f", spd);
}
uint16_t MeanSpeedLCD(char* str) {
  return sprintf(str, "Mn Sp %6.2f", mnspd);
}
uint16_t DistanceLCD(char* str) {
  if (dist < 9999.0) {
    return sprintf(str,"Dist %5d m", int(dist+0.5));
  } else {
    return sprintf(str, "%9.2f km", dist/1000);
  }
}
// Format string for speed
uint16_t AltitudeLCD(char* str) {
  return sprintf(str, "Alt %8.2f", alt);
}
// Format string for course
uint16_t CourseLCD(char* str) {
  return sprintf(str, "Course %5.1f", crs);
}

// Format string for time and fixed mode verification
uint16_t TimeLCD(char *str) {
  return sprintf(str, "%02d:%02d:%02d.%01.0f %c", h, m, s, ds, CheckMark());
}
uint16_t TimeShort(char *str) {
  return sprintf(str, "%02d%02d%02d", h, m, s);
}
// Format string for date
uint16_t DateLCD(char *str) {
  return sprintf(str, "%04d-%02d-%02d  ", yr, mon, day);
}
// Format string of 8 characters for file and dir names
uint16_t DateShort(char *str) {
  return sprintf(str, "%04d%02d%02d", yr, mon, day);
}
// Format string for waypoint and distance
uint16_t WaypointDistance(char *str, int i) {
  char c = cardinal(wp[i].course);
  if (wp[i].dist < 10000.0) { 
    return sprintf(str, "%-6.6s%5.2f%c", wp[i].name, wp[i].dist/1000.0,c);
  } else {
    return sprintf(str, "%-6.6s%5d%c", wp[i].name, int(wp[i].dist/1000.0+0.5),c);
  }
}
// Empty line of 12 chars
uint16_t LCDemptyLine(char* str) {
  return sprintf(str, "            ");
}

/**
 * Returns cardinal direction (NESW) given a course.
 */
char cardinal(double a) {
  char c = 'N';
  if (a> 45.0) {c='E';}
  if (a>135.0) {c='S';}
  if (a>225.0) {c='W';}
  if (a>315.0) {c='N';}
  return c;
}

/**
 * Go over the whole waypoint list and get 
 * the nearest waypoint from a given distance.
 */
int nearestBeyond(double d) {
  int minwp = 0; 
  double dmin = 20000000.0;
  for (int ip=0; ip<nwp; ip++) {
    if (wp[ip].dist > d && wp[ip].dist < dmin) {
       dmin = wp[ip].dist;
       minwp = ip;
    }
  }
  return minwp;
}

/**
 * TODO Read a waypoints gpx file.
 * 
 * <wpt lat="35.004120" lon="135.777568">
 *   <ele>54.115540</ele>
 *   <time>2014-09-24T07:07:33Z</time>
 *   <name>087</name>
 *   <sym>Lodging</sym>
 * </wpt>
 */
uint16_t read_waypoints(char* fn) { 
  // if (FAT::File::exists(fn)) {
  //   wp_file.open(fn, FA_READ | FA_OPEN_EXISTING);
  if (SD.exist(fn)) {
    File wp_file = SD.open(fn, FILE_READ);
    len = sSize;
    do {
      len = wp_file.read(bufWp, len);
      bufWp[len] = 0;  //Add string null terminator
      /** 
       * Now find all <wpt > tags in the buffer with lat and lon parameters, 
       * and then <name>, <ele> and <time> inside
       **/
    } while(len == sSize);
    wp_file.close();
  }
}

/**
 * TODO Convert a csv file into a gps location file
 */
uint16_t csv2gpx() {
  // Read directory names
  // Is there no matching gps for the current date?
  // Read the directory filenames
  // Read all location csv files and write single gpx for the date
}

/**
 * TODO Convert a csv file into kml
 */
// uint16_t csv2kml() {}

/** 
 * Interrupt functions for black and red buttons.
 **/
/**
  void switchScreen() {
  unsigned long timePressed = pulseIn(blackButton,HIGH,pressTime);
  if (timePressed = pressTime) {
    screenMode = screenMode + 1;
    if (screenMode > maxScreens) {screenMode = 0;}
  }
  return;
}
void toggleLight() {
  unsigned long timePressed = pulseIn(redButton,HIGH,pressTime);
  if (timePressed = pressTime) {
    digitalWrite(PCD8544_LED_PIN, !screenLight);
    screenLight = !screenLight;
  }
  return;
}
**/

/*****************************************************
 * Setup for receiver, nokia screen, sd card interface
 **/
void setup() {
  GnssConf.setNavMode(STGNSS_NAV_MODE_AUTO);
  GnssConf.setUpdateRate(STGNSS_POSITION_UPDATE_RATE_1HZ);
  GnssConf.setDopMaskMode(STGNSS_DOP_MASK_DISABLE);
  GnssConf.setDopMaskMode(STGNSS_DOP_MASK_AUTO);
  GnssConf.setPdopMask(30.0);
  GnssConf.setHdopMask(30.0);
  GnssConf.setGdopMask(30.0);
  GnssConf.init(); // Initialization for GNSS

  // Finally a couple of start values
  screenMode = 0;
  screenLight = TRUE;

  // Switch baudrate to slower speed (115200 is default)
  Serial.begin(9600);

  // Configure GPIO pins.
  pinMode(PCD8544_LED_PIN, screenLight);
  pinMode(PCD8544_SCE_PIN, OUTPUT);
  pinMode(PCD8544_DC_PIN, OUTPUT);
  pinMode(PCD8544_SCLK_PIN, OUTPUT);
  pinMode(PCD8544_SDIN_PIN, OUTPUT);
  pinMode(PCD8544_RES_PIN, OUTPUT);
  pinMode(satLed, OUTPUT);
  pinMode(blackButton, INPUT);
  pinMode(redButton, INPUT);

  digitalWrite(PCD8544_LED_PIN, HIGH);
  digitalWrite(PCD8544_SCE_PIN, HIGH);
  digitalWrite(PCD8544_DC_PIN, HIGH);
  digitalWrite(PCD8544_SCLK_PIN, LOW);
  digitalWrite(PCD8544_SDIN_PIN, LOW);
  digitalWrite(PCD8544_RES_PIN, HIGH);

  // attachInterrupt(blackButton,switchScreen,RISING);
  // attachInterrupt(redButton,toggleLight,RISING);
  // Timer0.every(25000,switchScreen);
  
  // Initialize the LCD controller
  pcd8544Init();

  /**
   * Initialise SD card stuff
   **/

  spiMaster.config(0, 10000000, false, false); // mode 0, 5MHz, CS0 and CS1
  spiMaster.begin();
  spiMaster.slaveSelect(0); // use GPIO28

  fatFsAgent.initialize();
  // sd.begin();

  /** 
   * Read waypoints from the waypoint gpx file 
   * For now just hardcode a few waypoints into the array
   * waypoint wp[0] is by default the current startpoint
   **/
  wp[0].lat = 0.0;
  wp[0].lon = 0.0;
  wp[0].alt = 0.0;
  wp[0].dist = 1000.0;
  strcpy(wp[0].name,"Start");
  wp[1].lat = 52.80075;
  wp[1].lon = 6.05467;
  wp[1].alt = -9.97;
  wp[1].dist = 1000.0;
  strcpy(wp[1].name,"Wold");
  wp[2].lat = 53.20466;
  wp[2].lon = 5.76804;
  wp[2].alt = -1.92;
  wp[2].dist = 1000.0;
  strcpy(wp[2].name,"Wetter");
  wp[3].lat = 52.36793;
  wp[3].lon = 4.89844;
  wp[3].alt = -5.13;
  wp[3].dist = 10.0;
  strcpy(wp[3].name,"Gracht");
  wp[4].lat = 45.91502;
  wp[4].lon = 4.04042;
  wp[4].alt = 412.09;
  wp[4].dist = 10000.0;
  strcpy(wp[4].name,"Dance");
  wp[5].lat = 32.02656;
  wp[5].lon = 118.78004;
  wp[5].alt = 51.80;
  wp[5].dist = 100000.0;
  strcpy(wp[5].name,"Nanjing");
  nwp = 6;
  
  // No trackpoints yet, no distance yet
  npts = 0;
  nptsbd2 = 0;
  nptsglo = 0;
  nptsbd23d = 0;
  nptsglo3d = 0;
  dist = 0.0;
}

/********************************************
 * Main loop
 *
 * Handles button presses on every cycles and
 * every now and the refreshes the display
 */
void loop() {
  int  len[6];
  char buf[6][64];
  static uint16_t delay = 0;

  /**
   * Check for button press
   * read the state of the pushbutton value:
   * Black button switches between screens
   * Red button switches LED backlighting on and off
   **/
  blackButtonState = digitalRead(blackButton);
  if (blackButtonState == HIGH && lastBlackButtonState == LOW) {
    screenMode = screenMode + 1;
    if (screenMode > maxScreens) {
      screenMode = 0;
    }
  }
  lastBlackButtonState = blackButtonState;

  redButtonState   = digitalRead(redButton);
  if (redButtonState == HIGH && lastRedButtonState == LOW) {
    digitalWrite(PCD8544_LED_PIN, !screenLight);
    screenLight = !screenLight;
  }
  lastRedButtonState   = redButtonState;

  /**
   * Actual display loop
   */
  if ((delay % delayLoop) == 0) {
    
    // Status LED on navspark lights up when GNSS is updated
    if (GnssInfo.isUpdated() == true) {
      digitalWrite(0, HIGH);
    } else {
      digitalWrite(0, LOW);
    }

    // Show the satellite status page
    switch (screenMode) {
      case 0:
        len[0] = GPSStatusLCD(buf[0]);
        #if (USEBEIDOU==1) 
          len[1] = BeidouStatusLCD(buf[1]);
          len[2] = BeidouStat(buf[2]);
        #endif
        #if (USEGLONASS==1) 
          len[1] = GlonassStatusLCD(buf[1]);
          len[2] = GlonassStat(buf[2]);
        #endif
        len[3] = PDOPLCD(buf[3]);
        len[4] = HDOPLCD(buf[4]);
        len[5] = VDOPLCD(buf[5]);
        break;
      case 1:
        len[0] = DateLCD(buf[0]);
        len[1] = TimeLCD(buf[1]);
        len[2] = LocationLatitudeLCD(buf[2]);
        len[3] = LocationLongitudeLCD(buf[3]);
        len[4] = AltitudeLCD(buf[4]);
        len[5] = WaypointDistance(buf[5], 0);
        break;
      case 2:
        len[0] = TimeLCD(buf[0]);
        len[1] = DistanceLCD(buf[1]);
        len[2] = SpeedLCD(buf[2]);
        len[3] = MeanSpeedLCD(buf[3]);
        len[4] = CourseLCD(buf[4]);
        len[5] = LCDemptyLine(buf[5]);
        break;
      case 3:
        for (int il=0; il<min(6,nwp); il++) {
          len[il] = WaypointDistance(buf[il], inear[il]);
        }
        break;
    }

    for (int line = 0; line < 6; line++) {
      pcd8544SetCursor(line, 0);
      for (int i = 0; i < len[line]; i++) {
        printOnLCD(buf[line][i]);
      }
    }
  }
  delay++;
}

/**************************
 * Called after each update
 *
 * Store all values in chache variables
 * Write the location and constellation info to the print buffers.
 * If a buffer is 80% full then write it to disk.
 */
void task_called_after_GNSS_update(void) {

  int len;
  char buf[1024];

  static uint16_t delay = 0;

  GnssInfo.update();

  /**
   * Remember current location information
   * to the location and status.
   **/
  lat = GnssInfo.location.latitude();
  lon = GnssInfo.location.longitude();
  alt = GnssInfo.altitude.meters();
  // DOP values
  pdop = GnssInfo.dop.pdop();
  hdop = GnssInfo.dop.hdop();
  vdop = GnssInfo.dop.vdop();
  // Speed and course
  spd = GnssInfo.speed.kph();
  crs = GnssInfo.course.deg();
  // Date and time
  yr  = GnssInfo.date.year();
  mon = GnssInfo.date.month();
  day = GnssInfo.date.day();
  h  = GnssInfo.time.hour();
  m  = GnssInfo.time.minute();
  s  = GnssInfo.time.second();
  ds = GnssInfo.time.decisecond();
  
  /**
   * Open a new directory when current date
   * does not match current directory name and
   * we have a fix (2d is ok)
   **/
  len = DateShort(folderDate);
  if (GnssInfo.fixMode() > 0) {  
    /**
     * Open a new direcotry if current directory name
     * does not match the current date
     **/
    if (strcmp(currentFolder, folderDate) != 0) {
      fatFsAgent.make_dir(folderDate);
      // sd.mkdir(folderDate);
      strcpy(currentFolder, folderDate);
      newFile = true;
    }

    /**
     * New location and constellation file name
     * in the new directory, we create it only
     * when first writing to it in the gnss_update loop
     **/
    if (newFile) {
      len = TimeShort(fileTime);
      strcpy(fileLocName, currentFolder);
      strcat(fileLocName, "/");
      strcat(fileLocName, fileTime);
      strcat(fileLocName, "loc.csv");
      strcpy(fileCstName, currentFolder);
      strcat(fileCstName, "/");
      strcat(fileCstName, fileTime);
      strcat(fileCstName, "sat.csv");
      newFile = false;
    }
  }

  /**
   * Compute new distances to the 6 nearest (or less) waypoints every locations refresh.
   **/
  for (int iwp=0; iwp<distList; iwp++) {
    int in = inear[iwp];
    wp[in].dist   = GnssInfo.distanceBetween(lat, lon, wp[in].lat, wp[in].lon);
    wp[in].course = GnssInfo.courseTo(lat, lon, wp[in].lat, wp[in].lon);
  }
  
  /**   
   * Compute distances to all waypoints every [delayDist] locations and sort them.
   * Skip wp[0] in the distance list
   **/
  if (GnssInfo.fixMode() >= 2 && s == 0) { 
    for (int iwp=0; iwp<nwp; iwp++) {
      wp[iwp].dist   = GnssInfo.distanceBetween(lat, lon, wp[iwp].lat, wp[iwp].lon);
      wp[iwp].course = GnssInfo.courseTo(lat, lon, wp[iwp].lat, wp[iwp].lon);
    }
    double dmin = 0.0;
    for (int iwp=0; iwp<distList; iwp++) {
      int in = nearestBeyond(dmin);
      // if (in==0) {in = nearestBeyond(wp[in].dist);} // Skip to next waypoint if default start waypoint wp[0]
      dmin = wp[in].dist;
      inear[iwp] = in;
    }
  }
    
  /**
   * then just list satellites used, otherwise
   * list all sats in view
   **/
  gps_used = GnssInfo.satellites.numGPSInUse(gpsPRN);
  if (gps_used > 0) {
    gps_view = GnssInfo.satellites.numGPSInView(NULL);
    gps_show = gps_used;
  } else {
    gps_view = GnssInfo.satellites.numGPSInView(gpsPRN);
    gps_show = gps_view;
  }
  for (uint16_t satno = 0; satno < gps_show; satno++) {
    gpscnr[satno] = GnssInfo.satellites.CNR(CONSTELLATION_GPS, gpsPRN[satno]);
    gpsele[satno] = GnssInfo.satellites.elevation(CONSTELLATION_GPS, gpsPRN[satno]);
    gpsazi[satno] = GnssInfo.satellites.azimuth(CONSTELLATION_GPS, gpsPRN[satno]);
  }
  if (loggps) {
    /**
     * Write each used satellite location to the
     * constellation buffer separately
     **/
    for (uint16_t satno = 0; satno < gps_used; satno++) {
      sprintf(buf, "%04d-%02d-%02d,%02d:%02d:%02d.%01d,",
              yr, mon, day, h, m, s, ds);
      strcat(bufCst, buf);
      sprintf(buf, "GPS,%02d,%02d,%02d,%03d\n",
              gpsPRN[satno], gpscnr[satno],
              gpsele[satno], gpsazi[satno]);
      strcat(bufCst, buf);
    }
  }

#if (USEBEIDOU==1)
  bd2_used = GnssInfo.satellites.numBD2InUse(bd2PRN);
  if (bd2_used > 0) {
    bd2_view = GnssInfo.satellites.numBD2InView(NULL);
    bd2_show = bd2_used;
  } else {
    bd2_view = GnssInfo.satellites.numBD2InView(bd2PRN);
    bd2_show = bd2_view;
  }
  for (uint16_t satno = 0; satno < bd2_show; satno++) {
    bd2cnr[satno] = GnssInfo.satellites.CNR(CONSTELLATION_BD2, bd2PRN[satno]);
    bd2ele[satno] = GnssInfo.satellites.elevation(CONSTELLATION_BD2, bd2PRN[satno]);
    bd2azi[satno] = GnssInfo.satellites.azimuth(CONSTELLATION_BD2, bd2PRN[satno]);
  }
  // Write each used satellite location to the constellation buffer separately
  for (uint16_t satno = 0; satno < bd2_used; satno++) {
    sprintf(buf, "%04d-%02d-%02d,%02d:%02d:%02d.%01d,",
            yr, mon, day, h, m, s, ds);
    strcat(bufCst, buf);
    sprintf(buf, "BD2,%02d,%02d,%02d,%03d\n",
            bd2PRN[satno], bd2cnr[satno], bd2ele[satno], bd2azi[satno]);
    strcat(bufCst, buf);
  }
#endif

#if (USEGLONASS==1)
  glo_used = GnssInfo.satellites.numGLNInUse(gloPRN);
  if (glo_used > 0) {
    glo_view = GnssInfo.satellites.numGLNInView(NULL);
    glo_show = glo_used;
  } else {
    glo_view = GnssInfo.satellites.numGLNInView(gloPRN);
    glo_show = glo_view;
  }
  for (uint16_t satno = 0; satno < glo_show; satno++) {
    glocnr[satno] = GnssInfo.satellites.CNR(CONSTELLATION_GLONASS, gloPRN[satno]);
    gloele[satno] = GnssInfo.satellites.elevation(CONSTELLATION_GLONASS, gloPRN[satno]);
    gloazi[satno] = GnssInfo.satellites.azimuth(CONSTELLATION_GLONASS, gloPRN[satno]);
  }
  // Write each used satellite location to the constellation buffer separately
  for (uint16_t satno = 0; satno < glo_used; satno++) {
    sprintf(buf, "%04d-%02d-%02d,%02d:%02d:%02d.%01d,",
            yr, mon, day, h, m, s, ds);
    strcat(bufCst, buf);
    sprintf(buf, "GLO,%02d,%02d,%02d,%03d\n",
            gloPRN[satno], glocnr[satno], gloele[satno], gloazi[satno]);
    strcat(bufCst, buf);
  }
#endif

 
  if (GnssInfo.fixMode() >= 3) { 
    /**
     * Update covered distance and mean speed with the current values
     */
    npts += 1;
    if (npts > 1) {
      double d = GnssInfo.distanceBetween(lat0,lon0,lat,lon);
      dist = dist + d;
      spdsum = spdsum + spd;
      mnspd = spdsum / npts;
    } else {
      wp[0].lat = lat;
      wp[0].lon = lon;
      wp[0].alt = alt;
    }   
    /**
     * Constellation statistics
     */
    if (bd2_used > 0) {nptsbd2 += 1;}
    if (bd2_used > 3) {nptsbd23d += 1;}
    if (glo_used > 0) {nptsglo += 1;}
    if (glo_used > 3) {nptsglo3d += 1;}
    
    /**
     * Remember current location for the next update
     */
    lat0 = lat;
    lon0 = lon;
    alt0 = alt;
    /**
     * Write location stuff to buffer if we have a 3d fix
     */
    sprintf(buf, "%04d-%02d-%02d,%02d:%02d:%02d.%01d,",
            yr, mon, day, h, m, s, ds);
    strcat(bufLoc, buf);
    sprintf(buf, "%12.8f,%12.8f,%10.2f,%5.1f,%5.1f,%5.1f,%5.1f,%5.1f,",
            lat, lon, alt, spd, crs, hdop, vdop, pdop);
    strcat(bufLoc, buf);
    sprintf(buf, "%02d,%02d,%02d,%02d,%02d,%02d\n",
            gps_view, gps_used, bd2_view, bd2_used, glo_view, glo_used);
    strcat(bufLoc, buf);
  }

  /**
   * Write location buffer to file if 80% full
   **/
  if (strlen(bufLoc) > sSize * .8) {
    // if (FAT::File::exists(fileLocName)) {
    if (SD.exist(fileLocName)) {
      // loc_file.open(fileLocName, FA_WRITE | FA_OPEN_EXISTING);
      File loc_file = SD.open(fileLocName, FILE_WRITE);
      // loc_file.lseek(loc_file.size());
      loc_file.seek(loc_file.uint32size());
      // loc_file.write((BYTE*)bufLoc, strlen(bufLoc));
      loc_file.write((char*)bufLoc);
      loc_file.close();
    } else {
      // File does not exist, create a new one
      // loc_file.open(fileLocName, FA_WRITE | FA_CREATE_NEW);
      loc_file = SD.open(fileLocName, FILE_WRITE);
      loc_file.write("date,time,lat,lon,alt,speed,course,hdop,vdop,pdop,");
      loc_file.write("gps_view,gps_used,beidou_view,beidou_used,");
      loc_file.write("glonass_view,glonass_used\n");
      loc_file.write((BYTE*)bufLoc, strlen(bufLoc));
      loc_file.close();
    }
    // Reset the buffer
    bufLoc[0] = (char)0;
  }

  /**
   * Write constellation buffer to file if 80% full
   **/
  if (strlen(bufCst) > sSize * .8) {
    // if (FAT::File::exists(fileCstName)) {
    if (SD.exists(fileCstName)) {
      // cst_file.open(fileCstName, FA_WRITE | FA_OPEN_EXISTING);
      File cst_file = SD.open(fileCstName, FILE_WRITE);
      // cst_file.lseek(cst_file.size());
      cst_file.seek(cst_file.uint32size());
      // cst_file.write((BYTE*)bufCst, strlen(bufCst));
      cst_file.write((char*)bufCst);
      cst_file.close();
    } else {
      // File does not exist, create a new one
      // cst_file.open(fileCstName, FA_WRITE | FA_CREATE_NEW);
      File cst_file = SD.open(fileCstName, FILE_WRITE);
      cst_file.write("date,time,system,prn,cnr,elevation,azimuth\n");
      // cst_file.write((BYTE*)bufCst, strlen(bufCst));
      cst_file.write((char*)bufCst);
      cst_file.close();
    }
    bufCst[0] = (char)0;
  }
}
