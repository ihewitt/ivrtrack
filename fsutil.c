/*
 * Basic file system helper utils
 */

#include <api_fs.h>
#include <api_os.h>
#include <stdlib.h>

#include "fsutil.h"

// Basic file exist check missing from API
bool FileExists(char* file) {
  int32_t fs = API_FS_Open(file, FS_O_RDONLY, 0);
  if (fs > 0) API_FS_Close(fs);
  return (fs > 0);
}

// Simple copy, only useful for small files
bool CopyFile(char* src, char* dst) {
  int32_t  fs;
  int32_t  fd;
  uint8_t* buf;
  int32_t  len;

  fs = API_FS_Open(src, FS_O_RDONLY, 0);
  if (fs < 0) { return false; }
  len = API_FS_GetFileSize(fs);
  buf = OS_Malloc(len + 16);

  fd = API_FS_Open(dst, FS_O_CREAT | FS_O_RDWR | FS_O_TRUNC, 0);

  API_FS_Read(fs, buf, len);
  API_FS_Write(fd, buf, len);

  API_FS_Close(fs);
  API_FS_Close(fd);
  OS_Free(buf);

  return true;
}
