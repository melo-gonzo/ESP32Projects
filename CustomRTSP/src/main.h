#include "platglue-esp32.h"
#include "CStreamer.h"
#include <Arduino.h>

// WiFi stuff
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include "CRtspSession.h"

void initRTSP(void);
void stopRTSP(void);


