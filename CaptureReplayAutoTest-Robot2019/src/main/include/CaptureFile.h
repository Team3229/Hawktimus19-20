/*
 * Team 3229
 * CaptureFile.h
 */

#ifndef CAPTUREFILE_H_
#define CAPTUREFILE_H_

#include <stdio.h>
#include <string>
#include "Debug.h"

class CaptureFile {

private:
  const int CHARSIZE = sizeof(char);
  const char *fileName;
  std::string str_filePath;
  FILE *fileHandle;
  bool isWrite;

public:
  CaptureFile();
  ~CaptureFile();
  void Open(std::string name, bool forWrite);
  void Close();
  void Read(void *buffer, int bufsize);
  void Write(void *buffer, int bufsize);
};

#endif
