/**
 * Team 3229
 * CaptureFile.cpp
 * 
 * This class is a simple file IO interface. The capture filename is specified
 * in the constructor and can be closed through a method or automatically in
 * the destructor.
 * 
 * This class uses C functions and not C++ IO because WPIlib somehow disables
 * normal fstream stuff.  Ugh.
 */        

#include "CaptureFile.h"

CaptureFile::CaptureFile(std::string name, bool forWrite) {
    // Setup the name and full path to file
    int size = sizeof(FILE_DIR) + name.length() + 1;
    fileName = name.c_str();
    filePath = new char[size];
    sprintf(filePath, "%s%s\0", FILE_DIR, fileName);
    str_filePath = std::string(filePath);

    // Setup r/w perms
    isWrite = forWrite;
    char * perms = "rb";
    if (isWrite)
        perms = "wb";

    // Open the file for read or write
    fileHandle = fopen(filePath, perms);

    // Verify file opened, print debug output on status
    if (fileHandle != NULL) {
        if (isWrite)
            debug("Opened file " + str_filePath + " for write.\n");
        else
            debug("Opened file " + str_filePath + " for read.\n");
    } else {
        debug("Error opening file " + str_filePath);
    }
}

CaptureFile::~CaptureFile() {
    // Close file
    Close();
}

void CaptureFile::Close() {
    if (fileHandle != NULL) {
        fflush(fileHandle);
        fclose(fileHandle);
    }
    // Set reference to null
    fileHandle = NULL;
}

void CaptureFile::Read(void * buffer, int bufsize) {
    if (fileHandle != NULL && buffer != NULL) {
        fread(buffer, CHARSIZE, bufsize, fileHandle);
    }
}

void CaptureFile::Write(void * buffer, int bufsize) {
    if (fileHandle != NULL && buffer != NULL) {
        fwrite(buffer, CHARSIZE, bufsize, fileHandle);
    }
}
