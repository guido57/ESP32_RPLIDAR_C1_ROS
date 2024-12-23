#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>

#include "credentials.h"
#include "RplidarC1.h"

// Micro-ROS variables
rcl_allocator_t allocator;
rclc_support_t support;
rcl_publisher_t publisher;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
    
#define RCCHECK(fn, msg)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("err=%d %s\r\n",temp_rc,msg);}return temp_rc;}
#define RCSOFTCHECK(fn, msg) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("err=%d %s\r\n",temp_rc,msg);}return temp_rc;}

RplidarC1 lidar;

void connect_wifi(){
    WiFi.disconnect(true);   // Reset Wi-Fi
    WiFi.mode(WIFI_STA);     // Set to Station mode
    WiFi.begin(ssid, pass);  // ssid and pass are defined in credentials.h
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}


// main timer callback
unsigned long start_uart;
unsigned long start_publishing;
unsigned long start_processing;
unsigned long total_loop_time;
float loop_period = 0.0;
int loop_count = 0;

void lidar_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    start_uart = millis();
    int count = lidar.uartRx();
    unsigned long uart_time = millis()-start_uart;
    total_loop_time = millis() - start_publishing;

    start_processing = millis();
    lidar.processFrame(count);
    
    start_publishing = millis();
    rcl_ret_t ret_pub = rcl_publish(&publisher, &lidar.scan_msg, NULL);
    
    if(ret_pub != RCL_RET_OK){
      printf("rcl_publish returned %d\r\n", ret_pub);
      esp_restart();
    }
    
    unsigned long publish_time = millis()-start_publishing;
    loop_period = loop_period*0.9 + total_loop_time*0.1;
    Serial.printf("got %d points in %lu ms. Frame processing in %lu ms. Frame publishing in %lu. Total loop in %lu ms. Freq=%.1f Hz Serial2.available=%d\r\n",
        count, uart_time, start_publishing-start_processing, publish_time, total_loop_time, 1000.0/loop_period, Serial2.available() );
    
    loop_count++;



  }
}

rcl_ret_t init_ros() {
    // Micro-ROS initialization

    rcl_ret_t ret;
   
    struct my_micro_ros_agent_locator {
      IPAddress address;
      int port;
    } static locator;
    locator.address = ros2_agent_ipa; // ros2_agent_ipa and ros2_agent_port are defined in credentials.h
    locator.port = ros2_agent_port;

    printf("rmw_uros_set_custom_transport...\r\n");
    ret=rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
    if (RCL_RET_OK != ret){
      printf("rmw_uros_set_custom_transport error=%d\r\n",ret);
      return ret;
    }
    
    allocator = rcl_get_default_allocator();

    printf("rclc_support_init...\r\n");
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (RCL_RET_OK != ret){
      printf("rclc_support_init error=%d\r\n",ret);
      return ret;
    }
    printf("rclc_node_init_default...\r\n");
    ret = rclc_node_init_default(&node, "lidar_node", "", &support);
    if (RCL_RET_OK != ret){
      printf("rclc_node_init_default error=%d\r\n",ret);
      return ret;
    }
    printf("rclc_publisher_init_default...\r\n");
    ret = 
          rclc_publisher_init_default(
              &publisher,
              &node,
              ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
              "/scan");  
    if (RCL_RET_OK != ret){
        printf("rclc_node_init_default error=%d\r\n",ret);
        return ret;        
    }

    // create 20 msecs timer
    printf("create lidar timer...\r\n");
    const unsigned int lidar_timer_timeout = 20;
    ret = rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(lidar_timer_timeout),
      lidar_timer_callback);
    if(ret != RCL_RET_OK){
      printf("rclc_timer_init_default lidar error=%d",ret);
      return ret;
    }

    // create executor
    printf("create executor...\r\n");
    ret = rclc_executor_init(&executor, &support.context, 3, &allocator);
    if(ret != RCL_RET_OK){
      printf("rclc_executor_init error=&d\r\n",ret);
      return ret;
    }
    printf("add  time to executor...\r\n");
    ret = rclc_executor_add_timer(&executor, &timer);
    if(RCL_RET_OK !=ret){
      printf("rclc_executor_add_timer error=%d\r\n",ret);
      return ret;
    }
  
    return RCL_RET_OK;
}

// Setup function
void setup() {
    Serial.begin(115200);  // Initialize Serial for debugging
    
    connect_wifi();
   
    if( RCL_RET_OK != init_ros()){
      printf("init_ros error. Rebooting ...\r\n");
      esp_restart();
    }
    

    lidar.begin();
    delay(1000);
    lidar.resetLidar();
    delay(800);
    lidar.startLidar();
}

rcl_ret_t ret;
void loop() {
   // Check Wi-Fi connection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Wi-Fi disconnected, reconnecting...");
        connect_wifi();
    }

    if( RCL_RET_OK != (ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)))){
      printf("rclc_executor_spin_some error=%d\r\n",ret);
    }
}


