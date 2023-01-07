#include <Arduino.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#pragma once

#include "CStreamer.h"
#include "main.h"
// Replace with your network credentials
#include "wifikeys.h"

//       AI-Thinker
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define ENABLE_RTSPSERVER
camera_fb_t *fb = NULL;
WiFiServer server(80);

void setup()
{
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

	Serial.begin(115200);
	Serial.setDebugOutput(true);
	Serial.println();
	camera_config_t config;
	config.ledc_channel = LEDC_CHANNEL_0;
	config.ledc_timer = LEDC_TIMER_0;
	config.pin_d0 = Y2_GPIO_NUM;
	config.pin_d1 = Y3_GPIO_NUM;
	config.pin_d2 = Y4_GPIO_NUM;
	config.pin_d3 = Y5_GPIO_NUM;
	config.pin_d4 = Y6_GPIO_NUM;
	config.pin_d5 = Y7_GPIO_NUM;
	config.pin_d6 = Y8_GPIO_NUM;
	config.pin_d7 = Y9_GPIO_NUM;
	config.pin_xclk = XCLK_GPIO_NUM;
	config.pin_pclk = PCLK_GPIO_NUM;
	config.pin_vsync = VSYNC_GPIO_NUM;
	config.pin_href = HREF_GPIO_NUM;
	config.pin_sscb_sda = SIOD_GPIO_NUM;
	config.pin_sscb_scl = SIOC_GPIO_NUM;
	config.pin_pwdn = PWDN_GPIO_NUM;
	config.pin_reset = RESET_GPIO_NUM;
	config.xclk_freq_hz = 20000000;
	config.pixel_format = PIXFORMAT_JPEG;
	config.frame_size = FRAMESIZE_SVGA;
	config.jpeg_quality = 10; // 0-63 lower number means higher quality
	config.fb_count = 2;
	// FRAMESIZE_QVGA (320 x 240)
	// FRAMESIZE_CIF (352 x 288)
	// FRAMESIZE_VGA (640 x 480)
	// FRAMESIZE_SVGA (800 x 600)
	// FRAMESIZE_XGA (1024 x 768)
	// FRAMESIZE_SXGA (1280 x 1024)
	// FRAMESIZE_UXGA (1600 x 1200)

	// camera init
	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK)
	{
		Serial.printf("Camera init failed with error 0x%x", err);
		delay(1000);
		ESP.restart();
	}

	WiFi.mode(WIFI_AP_STA);
	WiFi.begin(ssid, password);

	delay(1000);

	long int StartTime = millis();
	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		if ((StartTime + 10000) < millis())
			break;
	}

	if (WiFi.status() == WL_CONNECTED)
	{
		Serial.print("Stream Link: rtsp://");
		Serial.println(WiFi.localIP());
		Serial.println(":8554/mjpeg/1\n");
	}
	server.begin();
	initRTSP();
}

void loop()
{
	delay(1);
}

// Use this URL to connect the RTSP stream, replace the IP address with the address of your device
// rtsp://192.168.0.109:8554/mjpeg/1

/** Forward dedclaration of the task handling RTSP */
void rtspTask(void *pvParameters);

/** Task handle of the RTSP task */
TaskHandle_t rtspTaskHandler;

/** WiFi server for RTSP */
WiFiServer rtspServer(8554);

/** Stream for the camera video */
CStreamer *streamer = NULL;
/** Session to handle the RTSP communication */
CRtspSession *session = NULL;
/** Client to handle the RTSP connection */
WiFiClient rtspClient;
/** Flag from main loop to stop the RTSP server */
boolean stopRTSPtask = false;

/**
 * Starts the task that handles RTSP streaming
 */
void initRTSP(void)
{
	// Create the task for the RTSP server
	xTaskCreate(rtspTask, "RTSP", 4096, NULL, 1, &rtspTaskHandler);

	// Check the results
	if (rtspTaskHandler == NULL)
	{
		Serial.println("Create RTSP task failed");
	}
	else
	{
		Serial.println("RTSP task up and running");
	}
}

/**
 * Called to stop the RTSP task, needed for OTA
 * to avoid OTA timeout error
 */
void stopRTSP(void)
{
	stopRTSPtask = true;
}

/**
 * The task that handles RTSP connections
 * Starts the RTSP server
 * Handles requests in an endless loop
 * until a stop request is received because OTA
 * starts
 */

#include <Arduino.h>
#include <pgmspace.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_camera.h"

extern camera_config_t esp32cam_config, esp32cam_aithinker_config, esp32cam_ttgo_t_config;

class ESPCAM
{
public:
	ESPCAM()
	{
		fb = NULL;
	};
	~ESPCAM(){};
	esp_err_t init(camera_config_t config);
	void run(void);
	size_t getSize(void);
	uint8_t *getfb(void);
	int getWidth(void);
	int getHeight(void);
	framesize_t getFrameSize(void);
	pixformat_t getPixelFormat(void);

	void setFrameSize(framesize_t size);
	void setPixelFormat(pixformat_t format);

private:
	void runIfNeeded(); // grab a frame if we don't already have one

	// camera_framesize_t _frame_size;
	// camera_pixelformat_t _pixel_format;
	camera_config_t _cam_config;

	camera_fb_t *fb;
};

void ESPCAM::run(void)
{
	if (fb)
		// return the frame buffer back to the driver for reuse
		esp_camera_fb_return(fb);

	fb = esp_camera_fb_get();
}

void ESPCAM::runIfNeeded(void)
{
	if (!fb)
		run();
}

int ESPCAM::getWidth(void)
{
	runIfNeeded();
	return fb->width;
}

int ESPCAM::getHeight(void)
{
	runIfNeeded();
	return fb->height;
}

size_t ESPCAM::getSize(void)
{
	runIfNeeded();
	return fb->len;
}

uint8_t *ESPCAM::getfb(void)
{
	runIfNeeded();
	return fb->buf;
}

framesize_t ESPCAM::getFrameSize(void)
{
	return _cam_config.frame_size;
}

void ESPCAM::setFrameSize(framesize_t size)
{
	_cam_config.frame_size = size;
}

pixformat_t ESPCAM::getPixelFormat(void)
{
	return _cam_config.pixel_format;
}

void ESPCAM::setPixelFormat(pixformat_t format)
{
	switch (format)
	{
	case PIXFORMAT_RGB565:
	case PIXFORMAT_YUV422:
	case PIXFORMAT_GRAYSCALE:
	case PIXFORMAT_JPEG:
		_cam_config.pixel_format = format;
		break;
	default:
		_cam_config.pixel_format = PIXFORMAT_GRAYSCALE;
		break;
	}
}

class CamStreamer : public CStreamer
{
	bool m_showBig;
	ESPCAM &m_cam;

public:
	CamStreamer(SOCKET aClient, ESPCAM &cam);

	virtual void streamImage(uint32_t curMsec);
};

ESPCAM cam;
// fb = esp_camera_fb_get();

CamStreamer::CamStreamer(SOCKET aClient, ESPCAM &cam) : CStreamer(aClient, cam.getWidth(), cam.getHeight()), m_cam(cam)
{
	printf("Created streamer width=%d, height=%d\n", cam.getWidth(), cam.getHeight());
}

void CamStreamer::streamImage(uint32_t curMsec)
{
	m_cam.run(); // queue up a read for next time

	BufPtr bytes = m_cam.getfb();
	streamFrame(bytes, m_cam.getSize(), curMsec);
}

void rtspTask(void *pvParameters)
{
	uint32_t msecPerFrame = 1;
	static uint32_t lastimage = millis();

	// rtspServer.setNoDelay(true);
	rtspServer.setTimeout(1);
	rtspServer.begin();

	while (1)
	{
		// If we have an active client connection, just service that until gone
		if (session)
		{
			session->handleRequests(0); // we don't use a timeout here,
			// instead we send only if we have new enough frames

			uint32_t now = millis();
			if (now > lastimage + msecPerFrame || now < lastimage)
			{ // handle clock rollover
				session->broadcastCurrentFrame(now);
				lastimage = now;
			}

			// Handle disconnection from RTSP client
			if (session->m_stopped)
			{
				Serial.println("RTSP client closed connection");
				delete session;
				delete streamer;
				session = NULL;
				streamer = NULL;
			}
		}
		else
		{
			rtspClient = rtspServer.accept();
			// Handle connection request from RTSP client
			if (rtspClient)
			{
				Serial.println("RTSP client started connection");
				streamer = new CamStreamer(&rtspClient, cam); // our streamer for UDP/TCP based RTP transport

				session = new CRtspSession(&rtspClient, streamer); // our threads RTSP session and state
				delay(100);
			}
		}
		if (stopRTSPtask)
		{
			// User requested RTSP server stop
			if (rtspClient)
			{
				Serial.println("Shut down RTSP server because OTA starts");
				delete session;
				delete streamer;
				session = NULL;
				streamer = NULL;
			}
			// Delete this task
			vTaskDelete(NULL);
		}
		delay(1);
	}
}
