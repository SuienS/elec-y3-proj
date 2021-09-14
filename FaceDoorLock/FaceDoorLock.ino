/* 
======================================================================
                          FACE DOOR LOCK
______________________________________________________________________
 * Author      : Rammuni Ravidu Suien Silva
 * Student No  : PS/2016/220
 * Group       : B07
 * Course Unit : PHYS 33542
 * Code        : Face Doorlock Backend
 * SoC Module  : ESP32-CAM
______________________________________________________________________
  University of Kelaniya
======================================================================
*/
#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"

// Wifi credentials
const char* ssid = "xxxxxxx";
const char* password = "xxxxxxx";

#define FACE_SAMPLES_COUNT 5
#define FACE_COUNT_INDEX 7


#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

using namespace websockets;
WebsocketsServer socket_server;

camera_fb_t * fb = NULL;

long current_millis;
long last_detected_millis = 0;

#define lock_relay_pin 2 // Defining pin for lock relay
#define locked_indicator 12 // Defining pin for lock relay
unsigned long door_unlocked_period = 0;
long open_hold_time = 5000;           // time period that the lock stays opened

void start_recognition();
void start_web_server();

typedef struct
{
  uint8_t *face_image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} http_img_process_result;

// Detection parameters
/*
 * Below parameters define the sensitivity of the overall detection result of the algorithm
*/
static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

// Defining face list variable
face_id_name_list auth_face_name_list;
static dl_matrix3du_t *aligned_face = NULL;

// Variables for HTTP handles and camera stream
httpd_handle_t camera_httpd = NULL;


// Below ENUMs was used for code clarity
// ENUMs for representing main states of the face detection process
typedef enum
{
  START_FACE_STREAM,
  FACE_DETECT,
  FACE_RECOGNITION_INIT,
  FACE_ENROLL,
} en_fsm_state;
en_fsm_state system_state;

typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value new_face_name;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Started");

  digitalWrite(lock_relay_pin, LOW);
  pinMode(lock_relay_pin, OUTPUT);

  pinMode(locked_indicator, OUTPUT);
  digitalWrite(locked_indicator, HIGH);

  camera_config_t config;

  //pin assignment of the ESP module
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
  
  // Initializing with high values to reserve larger buffer amounts because of performance intensive tasks
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // Camera initialization
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif


  // WIFI configuration
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // HTTP server initialization
  start_web_server();
  start_recognition();
  // Listening to the socket
  socket_server.listen(82);

  Serial.print("Ready! GoTo 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to access the system");
}

// Web - Front End
static esp_err_t web_page_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

httpd_uri_t index_uri = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = web_page_handler,
  .user_ctx  = NULL
};

// HTTP Server initialization
void start_web_server()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
    Serial.println("httpd_start");
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}

// 
void start_recognition()
{
  face_id_name_init(&auth_face_name_list, FACE_COUNT_INDEX, FACE_SAMPLES_COUNT);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&auth_face_name_list);
}

//Enrolling the faces; Training the network with the input face_image
static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  ESP_LOGD(TAG, "START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, new_face_name.enroll_name);
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           new_face_name.enroll_name,
           FACE_SAMPLES_COUNT - left_sample_face);
  return left_sample_face;
}


static esp_err_t update_registered_faces(WebsocketsClient &socket_client)
{
  socket_client.send("delete_faces"); // tell browser to clear all faces
  face_id_node *head = auth_face_name_list.head;
  char new_face[64];
  for (int i = 0; i < auth_face_name_list.count; i++) // loop current faces
  {
    sprintf(new_face, "listface:%s", head->id_name);
    socket_client.send(new_face); //send face to browser
    head = head->next;
  }
}

static esp_err_t reset_face_list(WebsocketsClient &socket_client)
{
  delete_face_all_in_flash_with_name(&auth_face_name_list);
  socket_client.send("delete_faces");
}

// Handling request messages
void web_page_msg_handle(WebsocketsClient &socket_client, WebsocketsMessage web_page_msg)
{
  if (web_page_msg.data() == "stream") {
    system_state = START_FACE_STREAM;
    socket_client.send("STREAMING");
  }
  if (web_page_msg.data() == "detect") {
    system_state = FACE_DETECT;
    socket_client.send("DETECTING");
  }
  if (web_page_msg.data().substring(0, 8) == "capture:") {
    system_state = FACE_ENROLL;
    char person[FACE_COUNT_INDEX * ENROLL_NAME_LEN] = {0,};
    web_page_msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(new_face_name.enroll_name, person, strlen(person) + 1);
    socket_client.send("CAPTURING");
  }
  if (web_page_msg.data() == "recognise") {
    system_state = FACE_RECOGNITION_INIT;
    socket_client.send("RECOGNISING");
  }
  if (web_page_msg.data().substring(0, 7) == "remove:") {
    char person[ENROLL_NAME_LEN * FACE_COUNT_INDEX];
    web_page_msg.data().substring(7).toCharArray(person, sizeof(person));
    delete_face_id_in_flash_with_name(&auth_face_name_list, person);
    update_registered_faces(socket_client); // update face list in the browser
  }
  if (web_page_msg.data() == "delete_all") {
    reset_face_list(socket_client);
  }
}

//Lock opening
void open_door(WebsocketsClient &socket_client) {
  if (digitalRead(lock_relay_pin) == LOW) {
    digitalWrite(lock_relay_pin, HIGH); // Unlocking by setting the pin to HIGH
    digitalWrite(locked_indicator, LOW);  
    Serial.println("Door Unlocked");
    socket_client.send("door_open");
    door_unlocked_period = millis(); // time count starts
  }
}

// Looping program
void loop() {
  auto socket_client = socket_server.accept();
  socket_client.onMessage(web_page_msg_handle);
  dl_matrix3du_t *face_image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
  http_img_process_result out_res = {0};
  out_res.face_image = face_image_matrix->item;

  update_registered_faces(socket_client);
  socket_client.send("STREAMING");

  while (socket_client.available()) {
    socket_client.poll();

    if (millis() - open_hold_time > door_unlocked_period) { // time period after locking > 5 secs
      digitalWrite(lock_relay_pin, LOW); //opening lock relay
      digitalWrite(locked_indicator, HIGH);
    }

    fb = esp_camera_fb_get();

    if (system_state == FACE_DETECT || system_state == FACE_ENROLL || system_state == FACE_RECOGNITION_INIT)
    {
      out_res.net_boxes = NULL;
      out_res.face_id = NULL;

      fmt2rgb888(fb->buf, fb->len, fb->format, out_res.face_image);

      out_res.net_boxes = face_detect(face_image_matrix, &mtmn_config);

      if (out_res.net_boxes)
      {
        if (align_face(out_res.net_boxes, face_image_matrix, aligned_face) == ESP_OK)
        {

          out_res.face_id = get_face_id(aligned_face);
          last_detected_millis = millis();
          if (system_state == FACE_DETECT) {
            socket_client.send("FACE DETECTED");
          }

          if (system_state == FACE_ENROLL)
          {
            int left_sample_face = do_enrollment(&auth_face_name_list, out_res.face_id);
            char enrolling_message[64];
            sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", FACE_SAMPLES_COUNT - left_sample_face, new_face_name.enroll_name);
            socket_client.send(enrolling_message);
            if (left_sample_face == 0)
            {
              ESP_LOGI(TAG, "Enrolled Face ID: %s", auth_face_name_list.tail->id_name);
              system_state = START_FACE_STREAM;
              char captured_message[64];
              sprintf(captured_message, "FACE CAPTURED FOR %s", auth_face_name_list.tail->id_name);
              socket_client.send(captured_message);
              update_registered_faces(socket_client);

            }
          }

          if (system_state == FACE_RECOGNITION_INIT  && (auth_face_name_list.count > 0))
          {
            face_id_node *f = recognize_face_with_name(&auth_face_name_list, out_res.face_id);
            if (f)
            {
              char recognised_message[64];
              sprintf(recognised_message, "DOOR OPEN FOR %s", f->id_name);
              open_door(socket_client);
              socket_client.send(recognised_message);
            }
            else
            {
              socket_client.send("FACE NOT RECOGNISED");
            }
          }
          dl_matrix3d_free(out_res.face_id);
        }

      }
      else
      {
        if (system_state != FACE_DETECT) {
          socket_client.send("NO FACE DETECTED");
        }
      }

      if (system_state == FACE_DETECT && millis() - last_detected_millis > 500) {//Face detection is active but no face detected
        socket_client.send("DETECTING");
      }

    }

    socket_client.sendBinary((const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    fb = NULL;
  }
}
