// RplidarC1.cpp
#include "RplidarC1.h"

RplidarC1::RplidarC1()
    : dataIndex(0), pointAlign(false), frameActive(false), frameComplete(true) {}

void RplidarC1::begin() {
    Serial2.setRxBufferSize(8000);
    Serial2.begin(460800, SERIAL_8N1, 16, 17);
    // Initialize LaserScan message
    scan_msg.header.frame_id.data = (char *)"laser_frame";
    scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
    scan_msg.angle_min = MIN_ANGLE;
    scan_msg.angle_max = MAX_ANGLE;
    scan_msg.angle_increment = ANGLE_INCREMENT;
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.1; // Assuming 10 Hz scan rate
    scan_msg.range_min = MIN_RANGE;
    scan_msg.range_max = MAX_RANGE;

}

void RplidarC1::resetLidar() {
    uint8_t resetCommand[] = {0xA5, 0x40};
    Serial2.write(resetCommand, sizeof(resetCommand));
    Serial.println("LIDAR reset command sent");
}

void RplidarC1::startLidar() {
    uint8_t startCommand[] = {0xA5, 0x20};
    Serial2.write(startCommand, sizeof(startCommand));
    Serial.println("LIDAR start command sent");
}

int RplidarC1::uartRx() {
    uint8_t byte;
    unsigned long timeout = millis() + 3000;
    
    frameComplete = false;
    
    while (millis() < timeout) {
        if (!pointAlign) {
            //while (Serial2.available() > 10)
            //     Serial2.read();
            // if(Serial2.available() < 5 ){
            //     return 0; //continue;
            // }
            byte = Serial2.read();
            if ((byte & 0x11) != 0x10)
                continue;
            byte = Serial2.read();
            if ((byte & 0x1) != 0x1)
                continue;
            for(int i=0;i<3;i++)
                byte = Serial2.read();   // read 3rd 4th and 5th byte of this point
            pointAlign = true;
            dataIndex = 0;
            printf("%lu point alignment\r\n",millis());
            continue;
        }
        if (!frameActive) {
            if (Serial2.available() < 10)
                continue;
            Serial2.read(DataBuffer, 5);
            auto *point = (stScanDataPoint_t *)DataBuffer;
            if ((point->quality & 0x01) && !(point->quality & 0x02) && (point->angle_low & 0x01)) {
                frameActive = true;
                dataIndex = 5;
            }
            continue;
        }
        // we are point aligned and frameAligned
        // read 5 bytes
        if (Serial2.available() < 10)
            continue;
        Serial2.read(DataBuffer + dataIndex, 5);
        auto *point = (stScanDataPoint_t *)(DataBuffer + dataIndex);
        if ((point->quality & 0x01) && !(point->quality & 0x02) && (point->angle_low & 0x01)) {
            if (dataIndex > 2000) {
                frameComplete = true;
                 //printf("%lu frameComplete at dataIndex=%d\r\n",millis(),dataIndex);
            } else {
                frameActive = false;
                pointAlign = false;
                dataIndex = 5;
                continue;
            }
            int count = dataIndex / 5;
            dataIndex = 5;
            return count;
        }
        dataIndex += 5;
        if (dataIndex >= BUF_SIZE) {
            printf("%lu Overflow error at dataIndex=%d\r\n", millis(),dataIndex);
            pointAlign = false;
            frameActive = false;
            dataIndex = 0;
            return 0;
        }
    }
    // in case of timeout
    pointAlign = false;
    frameActive = false;
    dataIndex = 0;
    return 0;
}

void RplidarC1::processFrame(int num_points) {
    //int num_points = dataIndex / 5; // Each point is 5 bytes
    
    for (int i = 0; i < num_points; i++) {
        stScanDataPoint_t *point = (stScanDataPoint_t *)(DataBuffer + i * 5);
        uint16_t distance = (point->distance_high << 8) | point->distance_low;
        //uint16_t angle = ((point->angle_high << 8) | point->angle_low) >> 1; // 0.01° per unit
        float angle = (point->angle_high * 128 + point->angle_low / 2) / 64.0;
        
        float posf = angle*(num_points-1)/360.0;
        int pos = num_points - 1 - int(posf+0.5);
        if(pos<0) pos = 0;
        if(pos>=num_points) pos = num_points-1;
        ranges[pos] = ((float)distance) / 4000.0; // Convert to meters (RPLIDAR C1 sends: 4 * distance(in mm)
        intensities[pos] = point->quality / 4;
        // printf("ranges[%d]=%.3f\r\n",pos,ranges[pos]);
    }

    // Populate the LaserScan message
    scan_msg.header.stamp.sec = millis() / 1000;
    scan_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    scan_msg.ranges.data = ranges;
    scan_msg.ranges.size = num_points;
    scan_msg.intensities.data = intensities;
    scan_msg.intensities.size = num_points;
    scan_msg.angle_increment = 2.0 * PI / num_points;
}

