#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#if defined(_WIN32) || defined(_MSC_VER) || defined(__MINGW32__)
  #include <direct.h>     // _mkdir
  #include <io.h>         // _open, _close
  #include <fcntl.h>      // _O_WRONLY, _O_CREAT
  #include <sys/utime.h>  // _utime
  #define mkdir(path, mode) _mkdir(path)
  #define open  _open
  #define close _close
  #define utime _utime
  #define DIR_HANDLE intptr_t
  #include <windows.h>    // only if you need Windows-specific APIs
#else
  #include <dirent.h>
  #include <unistd.h>
  #include <utime.h>
#endif

#include "utils.h"


int g_verbosity = 0;

int read_s16_be(unsigned char *buf) {
   unsigned tmp = read_u16_be(buf);
   return (tmp > 0x7FFF) ? -((int)0x10000 - (int)tmp) : (int)tmp;
}

float read_f32_be(unsigned char *buf) {
   union { uint32_t i; float f; } ret;
   ret.i = read_u32_be(buf);
   return ret.f;
}

int is_power2(unsigned int val) {
   while ((val & 1) == 0 && val > 1) val >>= 1;
   return (val == 1);
}

void fprint_hex(FILE *fp, const unsigned char *buf, int length) {
   for (int i = 0; i < length; i++) {
      fprint_byte(fp, buf[i]);
      fputc(' ', fp);
   }
}

void fprint_hex_source(FILE *fp, const unsigned char *buf, int length) {
   for (int i = 0; i < length; i++) {
      if (i > 0) fputs(", ", fp);
      fputs("0x", fp);
      fprint_byte(fp, buf[i]);
   }
}

void print_hex(const unsigned char *buf, int length) {
   fprint_hex(stdout, buf, length);
}

void swap_bytes(unsigned char *data, long length) {
   for (long i = 0; i < length; i += 2) {
      unsigned char tmp = data[i];
      data[i] = data[i+1];
      data[i+1] = tmp;
   }
}

void reverse_endian(unsigned char *data, long length) {
   for (long i = 0; i < length; i += 4) {
      unsigned char tmp = data[i];
      data[i] = data[i+3];
      data[i+3] = tmp;
      tmp = data[i+1];
      data[i+1] = data[i+2];
      data[i+2] = tmp;
   }
}

long filesize(const char *filename) {
   struct stat st;
   return (stat(filename, &st) == 0) ? st.st_size : -1;
}

void touch_file(const char *filename) {
   int fd = open(filename, O_WRONLY | O_CREAT, 0666);
   if (fd >= 0) {
      utime(filename, NULL);
      close(fd);
   }
}

long read_file(const char *file_name, unsigned char **data) {
   FILE *in = fopen(file_name, "rb");
   if (!in) return -1;

   fseek(in, 0, SEEK_END);
   long file_size = ftell(in);
   if (file_size > 256*1024*1024) return -2;

   unsigned char *in_buf = malloc(file_size);
   fseek(in, 0, SEEK_SET);
   long bytes_read = fread(in_buf, 1, file_size, in);
   fclose(in);

   if (bytes_read != file_size) return -3;
   *data = in_buf;
   return bytes_read;
}

long write_file(const char *file_name, unsigned char *data, long length) {
   FILE *out = fopen(file_name, "wb");
   if (!out) return -1;
   long bytes_written = fwrite(data, 1, length, out);
   fclose(out);
   return bytes_written;
}

void generate_filename(const char *in_name, char *out_name, char *extension) {
   char tmp_name[FILENAME_MAX];
   int len = strlen(in_name);
   strcpy(tmp_name, in_name);
   for (int i = len - 1; i > 0; i--) {
      if (tmp_name[i] == '.') {
         tmp_name[i] = '\0';
         break;
      }
   }
   sprintf(out_name, "%s.%s", tmp_name, extension);
}

char *basename(const char *name) {
   const char *base = name;
   while (*name) {
      if (*name++ == '/' || *name == '\\') {
         base = name;
      }
   }
   return (char *)base;
}

void make_dir(const char *dir_name) {
   struct stat st = {0};
   if (stat(dir_name, &st) == -1) {
      mkdir(dir_name, 0755);
   }
}

long copy_file(const char *src_name, const char *dst_name) {
   unsigned char *buf;
   long bytes_read = read_file(src_name, &buf);
   if (bytes_read <= 0) return -1;

   long bytes_written = write_file(dst_name, buf, bytes_read);
   free(buf);
   return (bytes_written == bytes_read) ? bytes_read : -1;
}

void dir_list_ext(const char *dir, const char *extension, dir_list *list) {
#if defined(_WIN32)
   WIN32_FIND_DATA find_data;
   HANDLE hFind;
   char search_path[FILENAME_MAX];
   snprintf(search_path, sizeof(search_path), "%s\\*.*", dir);

   hFind = FindFirstFile(search_path, &find_data);
   if (hFind == INVALID_HANDLE_VALUE) {
      fprintf(stderr, "Can't open '%s'\n", dir);
      exit(1);
   }

   char *pool = malloc(FILENAME_MAX * MAX_DIR_FILES);
   char *pool_ptr = pool;
   int idx = 0;

   do {
      if (!(find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
         if (!extension || str_ends_with(find_data.cFileName, extension)) {
            snprintf(pool_ptr, FILENAME_MAX, "%s/%s", dir, find_data.cFileName);
            list->files[idx++] = pool_ptr;
            pool_ptr += strlen(pool_ptr) + 1;
         }
      }
   } while (FindNextFile(hFind, &find_data) && idx < MAX_DIR_FILES);

   FindClose(hFind);
   list->count = idx;
   list->files[0] = pool;
#else
   DIR *dfd = opendir(dir);
   if (!dfd) {
      fprintf(stderr, "Can't open '%s'\n", dir);
      exit(1);
   }

   char *pool = malloc(FILENAME_MAX * MAX_DIR_FILES);
   char *pool_ptr = pool;
   int idx = 0;
   struct dirent *entry;
   while ((entry = readdir(dfd)) && idx < MAX_DIR_FILES) {
      if (!extension || str_ends_with(entry->d_name, extension)) {
         sprintf(pool_ptr, "%s/%s", dir, entry->d_name);
         list->files[idx++] = pool_ptr;
         pool_ptr += strlen(pool_ptr) + 1;
      }
   }
   closedir(dfd);
   list->count = idx;
   list->files[0] = pool;
#endif
}

void dir_list_free(dir_list *list) {
   if (list->files[0]) {
      free(list->files[0]);
      list->files[0] = NULL;
   }
}

int str_ends_with(const char *str, const char *suffix) {
   if (!str || !suffix) return 0;
   size_t len_str = strlen(str);
   size_t len_suffix = strlen(suffix);
   return (len_suffix <= len_str) &&
          (strncmp(str + len_str - len_suffix, suffix, len_suffix) == 0);
}
