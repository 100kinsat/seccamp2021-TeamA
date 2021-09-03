#ifndef __MYSD_H__ //インクルードガード
#define __MYSD_H__

#include "Arduino.h"

#include "FS.h"
#include "SD.h"
#include "SPI.h"

class MySD {
  public:
    MySD();

    void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
    void createDir(fs::FS &fs, const char * path);
    void removeDir(fs::FS &fs, const char * path);
    void readFile(fs::FS &fs, const char * path);
    void writeFile(fs::FS &fs, const char * path, const char * message);
    void appendFile(fs::FS &fs, const char * path, const char * message);
    void renameFile(fs::FS &fs, const char * path1, const char * path2);
    void deleteFile(fs::FS &fs, const char * path);
    void testFileIO(fs::FS &fs, const char * path);
    void displaySpaceInfo();
    void appendLog(const char * message);

  private:
    bool printWithTime(File file, const char * message);
};

#endif // __MYSD_H__
