// Global copy of slave
#define NUMSLAVES 20
#define COMMANDLENGTH 100
esp_now_peer_info_t slaves[NUMSLAVES] = {};
int SlaveCnt = 0;

typedef struct commandStruct
{
  int speed;
  int angle;
};

commandStruct command;

typedef struct telemetryStruct
{
  float batteryVoltage;
};

telemetryStruct telemetry;