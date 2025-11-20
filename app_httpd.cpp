// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"
// JSON parsing
#include "ArduinoJson-v6.11.1.h"





typedef struct
{
    size_t size;  //number of values used for filtering
    size_t index; //current value index
    size_t count; //value count
    int sum;
    int *values; //array to be filled with values
} ra_filter_t;

typedef struct
{
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;

static const char *_STREAM_BOUNDARY = "\r\n";
static const char *_STREAM_PART = "Len: %u\r\n";

static const char *_STREAM_BOUNDARY_test = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART_test = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size)
{
    memset(filter, 0, sizeof(ra_filter_t));
    filter->values = (int *)malloc(sample_size * sizeof(int));
    if (!filter->values)
    {
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}

static int ra_filter_run(ra_filter_t *filter, int value)
{
    if (!filter->values)
    {
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size)
    {
        filter->count++;
    }
    return filter->sum / filter->count;
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (!index)
    {
        j->len = 0;
    }
    if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK)
    {
        return 0;
    }
    j->len += len;
    return len;
}
//图片帧捕获（图片）
static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_JPEG)
    {
        fb_len = fb->len;
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    }
    else
    {
        jpg_chunking_t jchunk = {req, 0};
        res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
        httpd_resp_send_chunk(req, NULL, 0);
        fb_len = jchunk.len;
    }
    esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
    return res;
}
//图片帧流（实时视频）AAP
static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[64];
    int64_t fr_start = 0;

    static int64_t last_frame = 0;
    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    while (true)
    {
        fb = esp_camera_fb_get(); //获取一帧图像
        if (!fb)
        {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        }
        else
        {
            fr_start = esp_timer_get_time();
            if (fb->format != PIXFORMAT_JPEG)
            {
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if (!jpeg_converted)
                {
                    Serial.println("JPEG compression failed");
                    res = ESP_FAIL;
                }
            }
            else
            {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }
        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART_test, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len); //原始发送
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY_test, strlen(_STREAM_BOUNDARY_test));
        }

        if (fb)
        {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        }
        else if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if (res != ESP_OK)
        {
            break;
        }
    }
    last_frame = 0;
    return res;
}

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char *buf;
    size_t buf_len;
    char variable[32] = {
        0,
    };
    char value[32] = {
        0,
    };

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1)
    {
        buf = (char *)malloc(buf_len);
        if (!buf)
        {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
        {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK)
            {
            }
            else
            {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        }
        else
        {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    }
    else
    {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int val = atoi(value);
    Serial.println(val);
    Serial.println(variable);
    sensor_t *s = esp_camera_sensor_get();
    int res = 0;

    if (!strcmp(variable, "framesize"))
    {
        if (s->pixformat == PIXFORMAT_JPEG)
            res = s->set_framesize(s, (framesize_t)val);
    }
    else if (!strcmp(variable, "quality"))
        res = s->set_quality(s, val);
    else if (!strcmp(variable, "contrast"))
        res = s->set_contrast(s, val);
    else if (!strcmp(variable, "brightness"))
        res = s->set_brightness(s, val);
    else if (!strcmp(variable, "saturation"))
        res = s->set_saturation(s, val);
    else if (!strcmp(variable, "gainceiling"))
        res = s->set_gainceiling(s, (gainceiling_t)val);
    else if (!strcmp(variable, "colorbar"))
        res = s->set_colorbar(s, val);
    else if (!strcmp(variable, "awb"))
        res = s->set_whitebal(s, val);
    else if (!strcmp(variable, "agc"))
        res = s->set_gain_ctrl(s, val);
    else if (!strcmp(variable, "aec"))
        res = s->set_exposure_ctrl(s, val);
    else if (!strcmp(variable, "hmirror"))
        res = s->set_hmirror(s, val);
    else if (!strcmp(variable, "vflip"))
        res = s->set_vflip(s, val);
    else if (!strcmp(variable, "awb_gain"))
        res = s->set_awb_gain(s, val);
    else if (!strcmp(variable, "agc_gain"))
        res = s->set_agc_gain(s, val);
    else if (!strcmp(variable, "aec_value"))
        res = s->set_aec_value(s, val);
    else if (!strcmp(variable, "aec2"))
        res = s->set_aec2(s, val);
    else if (!strcmp(variable, "dcw"))
        res = s->set_dcw(s, val);
    else if (!strcmp(variable, "bpc"))
        res = s->set_bpc(s, val);
    else if (!strcmp(variable, "wpc"))
        res = s->set_wpc(s, val);
    else if (!strcmp(variable, "raw_gma"))
        res = s->set_raw_gma(s, val);
    else if (!strcmp(variable, "lenc"))
        res = s->set_lenc(s, val);
    else if (!strcmp(variable, "special_effect"))
        res = s->set_special_effect(s, val);
    else if (!strcmp(variable, "wb_mode"))
        res = s->set_wb_mode(s, val);
    else if (!strcmp(variable, "ae_level"))
        res = s->set_ae_level(s, val);
    else
    {
        res = -1;
    }

    if (res)
    {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    static char json_response[1024];

    sensor_t *s = esp_camera_sensor_get();
    char *p = json_response;
    *p++ = '{';

    p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p += sprintf(p, "\"quality\":%u,", s->status.quality);
    p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
    p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p += sprintf(p, "\"awb\":%u,", s->status.awb);
    p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p += sprintf(p, "\"aec\":%u,", s->status.aec);
    p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p += sprintf(p, "\"agc\":%u,", s->status.agc);
    p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p += sprintf(p, "\"vflip\":%u,", s->status.vflip);
    p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID)
    {
        return httpd_resp_send(req, (const char *)index_ov3660_html_gz, index_ov3660_html_gz_len);
    }
    return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}
// //图片帧流（实时视频）Test
// static esp_err_t Test_handler(httpd_req_t *req)
// {
//     camera_fb_t *fb = NULL;
//     esp_err_t res = ESP_OK;
//     size_t _jpg_buf_len = 0;
//     uint8_t *_jpg_buf = NULL;
//     char *part_buf[64];
//     dl_matrix3du_t *image_matrix = NULL;
//     bool detected = false;
//     int face_id = 0;
//     int64_t fr_start = 0;
//     int64_t fr_ready = 0;
//     int64_t fr_face = 0;
//     int64_t fr_recognize = 0;
//     int64_t fr_encode = 0;

//     detection_enabled = 1;
//     static int64_t last_frame = 0;
//     if (!last_frame)
//     {
//         last_frame = esp_timer_get_time();
//     }

//     res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
//     if (res != ESP_OK)
//     {
//         return res;
//     }

//     httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
//     while (true)
//     {
//         detected = false;
//         face_id = 0;
//         fb = esp_camera_fb_get(); //获取一帧图像
//         if (!fb)
//         {
//             Serial.println("Camera capture failed");
//             res = ESP_FAIL;
//         }
//         else
//         {
//             fr_start = esp_timer_get_time();
//             fr_ready = fr_start;
//             fr_face = fr_start;
//             fr_encode = fr_start;
//             fr_recognize = fr_start;
//             if (!detection_enabled || fb->width > 400)
//             {
//                 if (fb->format != PIXFORMAT_JPEG)
//                 {
//                     bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
//                     esp_camera_fb_return(fb);
//                     fb = NULL;
//                     if (!jpeg_converted)
//                     {
//                         Serial.println("JPEG compression failed");
//                         res = ESP_FAIL;
//                     }
//                 }
//                 else
//                 {
//                     _jpg_buf_len = fb->len;
//                     _jpg_buf = fb->buf;
//                 }
//             }
//             else
//             {
//                 image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
//                 if (!image_matrix)
//                 {
//                     Serial.println("dl_matrix3du_alloc failed");
//                     res = ESP_FAIL;
//                 }
//                 else
//                 {
//                     if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item))
//                     {
//                         Serial.println("fmt2rgb888 failed");
//                         res = ESP_FAIL;
//                     }
//                     else
//                     {
//                         fr_ready = esp_timer_get_time();
//                         box_array_t *net_boxes = NULL;
//                         if (detection_enabled)
//                         {
//                             net_boxes = face_detect(image_matrix, &mtmn_config);
//                         }
//                         fr_face = esp_timer_get_time();
//                         fr_recognize = fr_face;
//                         if (net_boxes || fb->format != PIXFORMAT_JPEG)
//                         {
//                             if (net_boxes)
//                             {
//                                 detected = true;
//                                 if (recognition_enabled)
//                                 {
//                                     face_id = run_face_recognition(image_matrix, net_boxes);
//                                 }
//                                 fr_recognize = esp_timer_get_time();
//                                 //rgb_printf(image_matrix, FACE_COLOR_GREEN, "Hello Subject %u", face_id);
//                                 draw_face_boxes(image_matrix, net_boxes, face_id);
//                                 free(net_boxes->score);
//                                 free(net_boxes->box);
//                                 free(net_boxes->landmark);
//                                 free(net_boxes);
//                             }
//                             if (!fmt2jpg(image_matrix->item, fb->width * fb->height * 3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len))
//                             {
//                                 Serial.println("fmt2jpg failed");
//                                 res = ESP_FAIL;
//                             }
//                             esp_camera_fb_return(fb);
//                             fb = NULL;
//                         }
//                         else
//                         {
//                             _jpg_buf = fb->buf;
//                             _jpg_buf_len = fb->len;
//                         }
//                         fr_encode = esp_timer_get_time();
//                     }
//                     dl_matrix3du_free(image_matrix);
//                 }
//             }
//         }
//         if (res == ESP_OK)
//         {
//             size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART_test, _jpg_buf_len);
//             res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
//         }
//         if (res == ESP_OK)
//         {
//             res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len); //原始发送
//         }
//         if (res == ESP_OK)
//         {
//             res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY_test, strlen(_STREAM_BOUNDARY_test));
//         }

//         if (fb)
//         {
//             esp_camera_fb_return(fb);
//             fb = NULL;
//             _jpg_buf = NULL;
//         }
//         else if (_jpg_buf)
//         {
//             free(_jpg_buf);
//             _jpg_buf = NULL;
//         }
//         if (res != ESP_OK)
//         {
//             break;
//         }
//     }
//     last_frame = 0;
//     return res;
// }

static esp_err_t Test1_handler(httpd_req_t *req)
{
    Serial.println("Test1_handler...");
    char *buf;
    size_t buf_len;
    char variable[32] = {
        0,
    };
    char value[32] = {
        0,
    };
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1)
    {
        buf = (char *)malloc(buf_len);
        if (!buf)
        {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
        {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK)
            {
               // Serial2.println(variable);
            }
            else
            {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        }
        else
        {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    }
    else
    {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // httpd_resp_send(req, (const char *)"index", 5);
    return ESP_OK;
}

static esp_err_t Test2_handler(httpd_req_t *req)
{
    Serial.println("Test2_handler...");

    //  httpd_resp_send(req, (const char *)"index", 5);
    httpd_resp_send(req, (const char *)"index", 5);
    return ESP_OK;
}

// Helper: send a frame string over Serial2 and wait for Arduino ack of form {<id>_ok}
static bool sendFrameAndWaitAck(const String &frame, const String &id, uint32_t timeoutMs = 800)
{
    
    // send frame
    Serial2.print(frame);
    Serial.print("sent command for queueing ");
    Serial.println(frame);
    unsigned long start = millis();
    String buf = "";
delay(500);
    while (millis() - start < timeoutMs)
    {
        while (Serial2.available())
        {
            buf = Serial2.readString();
            Serial.print("Received: ");
            Serial.println(buf);
            if(buf.length() > 0){
                return true;
            }
        }
        delay(5);
    }
    Serial.print("timed out");
    return false; // timeout
}
// Helper: send a frame over Serial2 and wait for a JSON object response (returns true + fills outJson)
static bool sendFrameAndWaitJsonResponse(const String &frame, String &outJson, uint32_t timeoutMs = 3000)
{
 
    // send frame
    Serial2.print(frame);
    Serial.print("Sent frame: ");
    Serial.println(frame);
    
    unsigned long start = millis();
    String buf = "";
    delay(500);
    bool ok = Serial2.available();
    Serial.print(ok);
    while (millis() - start < timeoutMs)
    {
        while (Serial2.available())
        {
        outJson = Serial2.readString();
        if(outJson.length()>0){
            return true;
        }
    }
}
// Serial.print("\nTimeout! Received ");
// Serial.print(frame_from_arduino.length());
// Serial.println(" chars total");
// Serial.println(frame_from_arduino);
return false; // timeout
}
// POST /api/path
// Accepts single action {"cmd":"move","d":5.0,"dir":1,"id":"m001"} or {"cmd":"turn","a":90,"id":"t001"}
// or an array of such objects. Converts to Arduino protocol and forwards per-action.
static esp_err_t path_post_handler(httpd_req_t *req)
{
    size_t content_len = req->content_len;
    if (content_len == 0 || content_len > 4096)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }

    char *body = (char *)malloc(content_len + 1);
    if (!body)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    size_t received = 0;
    while (received < content_len)
    {
        int ret = httpd_req_recv(req, body + received, content_len - received);
        if (ret <= 0)
        {
            free(body);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        received += ret;
    }
    body[content_len] = '\0';

    // Parse JSON
    DynamicJsonDocument doc(4096);
    DeserializationError err = deserializeJson(doc, body);
    free(body);
    if (err)
    {
          Serial.print(F("deserializeJson() failed: "));
    Serial.println(err.c_str());
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }

    // Prepare response JSON
    DynamicJsonDocument resp(1024);
    JsonArray acks = resp.createNestedArray("acks");

    // Helper to process a single action object
    auto processAction = [&](JsonObject action) {
        const char *cmd = action["cmd"] | "";
        String id = action["id"] | "";
        String frame;
        if (strcmp(cmd, "move") == 0)
        {
            float meters = action["d"] | 0.0f;
            int dir = action["dir"] | 1;
            uint32_t cm = (uint32_t)round(meters * 100.0f);
            if (id.length() == 0)
            {
                id = String("m") + String(random(1, 10000));
            }
            frame = String("{\"N\":200,\"D1\":") + String(dir) + String(",\"D2\":") + String(cm) + String(",\"H\":\"") + id + String("\"}");
        }
        else if (strcmp(cmd, "turn") == 0)
        {
            int angle = action["a"] | 0;
            if (id.length() == 0)
            {
                id = String("t") + String(random(1, 10000));
            }
            frame = String("{\"N\":201,\"D1\":") + String(angle) + String(",\"H\":\"") + id + String("\"}");
        }
        else
        {
            return; // ignore unknown
        }

        bool ok = sendFrameAndWaitAck(frame, id, 3000);
        if (ok)
            acks.add(id + String("_ok"));
        else
            acks.add(String("{\"id\":\"") + id + String("\",\"status\":\"fail\"}"));
    };

    if (doc.is<JsonArray>())
    {
        for (JsonObject item : doc.as<JsonArray>())
            processAction(item);
    }
    else if (doc.is<JsonObject>())
    {
        processAction(doc.as<JsonObject>());
    }
    else
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad payload");
        return ESP_FAIL;
    }

    String out;
    serializeJson(resp, out);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, out.c_str(), out.length());
    return ESP_OK;
}

// GET /api/pose
// Query the Arduino for current estimated pose via N=300. Returns the Arduino JSON directly.
static esp_err_t pose_get_handler(httpd_req_t *req)
{
    // construct a random id so Arduino will echo it in H
    String id = String("p") + String(random(1000, 9999));
    String frame = String("{\"N\":300,\"H\":\"") + id + String("\"}");
    String respJson;
    Serial.println("\n=== Pose Query Start ===");
    Serial.print("Sending: ");
    Serial.println(frame);
        
    bool ok = sendFrameAndWaitJsonResponse(frame, respJson, 3000);
    Serial.print("recieved ");
    Serial.print(respJson);
    Serial.println("=== Pose Query End ===\n");
    
    if (!ok)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no response");
        return ESP_FAIL;
    }
    // respJson contains the Arduino response JSON (e.g. {"H":"p1234","pose":{"x":...,"v":...}})
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, respJson.c_str(), respJson.length());
    return ESP_OK;
}

// Simple web UI served at /ui — tiny page to send path actions and show pose
static esp_err_t ui_get_handler(httpd_req_t *req)
{
    const char *html = "<!doctype html>\n"
                       "<html><head><meta charset=\"utf-8\"><title>Robot Path UI</title></head>\n"
                       "<body>\n"
                       "<h3>Robot Path UI</h3>\n"
                       "<div>Move (meters): <input id=\"moveMeters\" value=\"0.5\"> dir: <select id=\"dir\"><option value=\"1\">forward</option><option value=\"2\">backward</option></select> <button onclick=\"sendMove()\">Send Move</button></div>\n"
                       "<div>Turn (deg): <input id=\"turnDeg\" value=\"90\"> <button onclick=\"sendTurn()\">Send Turn</button></div>\n"
                       "<div><button onclick=\"sendSamplePath()\">Send Sample Path</button></div>\n"
                       "<pre id=\"log\" style=\"height:200px;overflow:auto;border:1px solid #ccc\"></pre>\n"
                       "<h4>Pose</h4><pre id=\"pose\">-</pre>\n"
                       "<script>\n"
                       "function log(s){document.getElementById('log').textContent += s + '\n';} \n"
                       "function sendMove(){let m=document.getElementById('moveMeters').value;let dir=document.getElementById('dir').value;fetch('/api/path',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'move',d:parseFloat(m),dir:parseInt(dir)})}).then(r=>r.json()).then(j=>log(JSON.stringify(j)));}\n"
                       "function sendTurn(){let a=document.getElementById('turnDeg').value;fetch('/api/path',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'turn',a:parseInt(a)})}).then(r=>r.json()).then(j=>log(JSON.stringify(j)));}\n"
                       "function sendSamplePath(){let arr=[{cmd:'move',d:0.5,dir:1,id:'m1'},{cmd:'turn',a:90,id:'t1'},{cmd:'move',d:0.2,dir:1,id:'m2'}];fetch('/api/path',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(arr)}).then(r=>r.json()).then(j=>log(JSON.stringify(j)));}\n"
                       "function pollPose(){fetch('/api/pose').then(r=>r.json()).then(j=>{document.getElementById('pose').textContent = JSON.stringify(j);}).catch(e=>{});} setInterval(pollPose,500);\n"
                       "</script>\n"
                       "</body></html>";
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}
void startCameraServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL};

    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL};

    httpd_uri_t cmd_uri = {
        .uri = "/control",
        .method = HTTP_GET,
        .handler = cmd_handler,
        .user_ctx = NULL};

    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = NULL};

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL};

    httpd_uri_t Test_uri = {
        .uri = "/Test",
        .method = HTTP_GET,
        //.handler = Test_handler,
        .handler = stream_handler,
        .user_ctx = NULL};

    httpd_uri_t Test1_uri = {
        .uri = "/test1",
        .method = HTTP_GET,
        .handler = Test1_handler,
        .user_ctx = NULL};

    httpd_uri_t Test2_uri = {
        .uri = "/test2",
        .method = HTTP_GET,
        .handler = Test2_handler,
        .user_ctx = NULL};

    httpd_uri_t path_uri = {
        .uri = "/api/path",
        .method = HTTP_POST,
        .handler = path_post_handler,
        .user_ctx = NULL};

    httpd_uri_t pose_uri = {
        .uri = "/api/pose",
        .method = HTTP_GET,
        .handler = pose_get_handler,
        .user_ctx = NULL};

    httpd_uri_t ui_uri = {
        .uri = "/ui",
        .method = HTTP_GET,
        .handler = ui_get_handler,
        .user_ctx = NULL};

    ra_filter_init(&ra_filter, 20);

    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
        httpd_register_uri_handler(camera_httpd, &Test_uri);
        httpd_register_uri_handler(camera_httpd, &Test1_uri);
        httpd_register_uri_handler(camera_httpd, &Test2_uri);
        httpd_register_uri_handler(camera_httpd, &path_uri);
        httpd_register_uri_handler(camera_httpd, &pose_uri);
        httpd_register_uri_handler(camera_httpd, &ui_uri);
    }
    config.server_port += 1; //视频流端口
    config.ctrl_port += 1;
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}
