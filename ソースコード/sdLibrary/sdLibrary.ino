/*
 * Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       -
 *    D3       SS
 *    CMD      MOSI
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      SCK
 *    VSS      GND
 *    D0       MISO
 *    D1       -
 */
#include "sd_lib.hpp"

MySD mysd = MySD();

void setup(){
    Serial.begin(115200);
    mysd.listDir(SD, "/", 0);
    mysd.createDir(SD, "/mydir");
    mysd.listDir(SD, "/", 0);
    mysd.removeDir(SD, "/mydir");
    mysd.listDir(SD, "/", 2);
    mysd.writeFile(SD, "/hello.txt", "Hello ");
    mysd.appendFile(SD, "/hello.txt", "World!\n");
    mysd.readFile(SD, "/hello.txt");
    mysd.deleteFile(SD, "/foo.txt");
    mysd.renameFile(SD, "/hello.txt", "/foo.txt");
    mysd.readFile(SD, "/foo.txt");
    mysd.testFileIO(SD, "/test.txt");
    mysd.displaySpaceInfo();
    mysd.appendLog("logInfo");
}

void loop(){
}
