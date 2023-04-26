#include "OV2640.h"

#define TAG "OV2640"

#include <iostream>

void OV2640::run(void)
{
    if (fb)
        //return the frame buffer back to the driver for reuse
        esp_camera_fb_return(fb);

    fb = esp_camera_fb_get();
}

void OV2640::runIfNeeded(void)
{
    if (!fb)
        run();
}

int OV2640::getWidth(void)
{
    runIfNeeded();
    return fb->width;
}

int OV2640::getHeight(void)
{
    runIfNeeded();
    return fb->height;
}

size_t OV2640::getSize(void)
{
    runIfNeeded();
    if (!fb)
        return 0; // FIXME - this shouldn't be possible but apparently the new cam board returns null sometimes?
    return fb->len;
}

uint8_t *OV2640::getfb(void)
{
    runIfNeeded();
    if (!fb)
        return NULL; // FIXME - this shouldn't be possible but apparently the new cam board returns null sometimes?

    return fb->buf;
}

framesize_t OV2640::getFrameSize(void)
{
    return _cam_config.frame_size;
}

void OV2640::setFrameSize(framesize_t size)
{
    _cam_config.frame_size = size;
}

pixformat_t OV2640::getPixelFormat(void)
{
    return _cam_config.pixel_format;
}

void OV2640::setPixelFormat(pixformat_t format)
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

esp_err_t OV2640::init(camera_config_t config)
{
    memset(&_cam_config, 0, sizeof(_cam_config));
    memcpy(&_cam_config, &config, sizeof(config));
    std::cout<<&_cam_config<<std::endl;

    esp_err_t err = esp_camera_init(&_cam_config);
    if (err != ESP_OK)
    {
        printf("Camera probe failed with error 0x%x", err);
        return err;
    }
    // ESP_ERROR_CHECK(gpio_install_isr_service(0));

    return ESP_OK;
}
