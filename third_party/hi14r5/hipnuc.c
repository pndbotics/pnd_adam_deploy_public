#include "hipnuc.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdatomic.h>
#include <stdbool.h>

#include "serial_port.h"

#define REFRESH_INTERVAL 2000  // 以微秒为单位，对应于50Hz

float hi_pitch_imu, hi_yaw_imu, hi_roll_imu;
float hi_g_x_imu, hi_g_y_imu, hi_g_z_imu;
float hi_a_x_imu, hi_a_y_imu, hi_a_z_imu;
pthread_rwlock_t rwlock;
atomic_bool exit_flag;

// struct timeval start;
// struct timeval end;

int hi_imu_count = 0;

hipnuc_raw_t hipnuc_raw = {0};
uint8_t recv_buf[2048];
uint8_t sdata[1024];
int len;
int fd;

int hipnuc_open(char* port_name) {
  fd = serial_port_open(port_name);
  if (fd < 0) return 1;

  if (serial_port_configure(fd, 921600) < 0) {
    fprintf(stderr, "Cannot open %s\n", port_name);
    serial_port_close(fd);
    return 1;
  }

  // tcp_client_init("10.10.10.10", 8098);

  /* enable output */
  serial_send_then_recv(fd, "AT+EOUT=1\r\n", "OK\r\n", recv_buf, sizeof(recv_buf), 200);
}

void* hip_imu_run(void* arg) {
  while (atomic_load(&exit_flag) == false) {
    len = read(fd, recv_buf, sizeof(recv_buf));

    for (int i = 0; i < len; i++) {
      if (hipnuc_input(&hipnuc_raw, recv_buf[i])) {
        // printf("%0.4f\t%0.4f\t\t%0.4f\r\n", hipnuc_raw.hi91.roll, hipnuc_raw.hi91.pitch, hipnuc_raw.hi91.yaw);

        // printf("%f\t%f\t%f\r\n", hipnuc_raw.hi91.yaw, hipnuc_raw.hi91.pitch, hipnuc_raw.hi91.roll);

        pthread_rwlock_wrlock(&rwlock);
        hi_pitch_imu = hipnuc_raw.hi91.pitch;
        hi_yaw_imu = hipnuc_raw.hi91.yaw;
        hi_roll_imu = hipnuc_raw.hi91.roll;

        hi_g_x_imu = hipnuc_raw.hi91.gyr[0];
        hi_g_y_imu = hipnuc_raw.hi91.gyr[1];
        hi_g_z_imu = hipnuc_raw.hi91.gyr[2];

        hi_a_x_imu = hipnuc_raw.hi91.acc[0];
        hi_a_y_imu = hipnuc_raw.hi91.acc[1];
        hi_a_z_imu = hipnuc_raw.hi91.acc[2];
        pthread_rwlock_unlock(&rwlock);

        // printf("%f\t%f\t%f\r\n",
        //         hipnuc_raw.hi91.roll, hipnuc_raw.hi91.pitch, hipnuc_raw.hi91.yaw);

        // sprintf(sdata, "%f\t%f\t%f\r\n",
        //         hipnuc_raw.hi91.acc[0], hipnuc_raw.hi91.acc[1], hipnuc_raw.hi91.acc[2]);

        // sprintf(sdata, "%f\t%f\t%f\r\n",
        //         hipnuc_raw.hi91.gyr[0], hipnuc_raw.hi91.gyr[1], hipnuc_raw.hi91.gyr[2]);

        // gettimeofday(&end, NULL);
        // uint32_t timer = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
        // start.tv_sec = end.tv_sec;
        // start.tv_usec = end.tv_usec;

        // sprintf(sdata, "%d\r\n", timer);

        hi_imu_count++;
      }
    }
    usleep(REFRESH_INTERVAL);
  }
}

void hipnuc_read(float* imu_data) {
  pthread_rwlock_rdlock(&rwlock);
  imu_data[0] = hi_yaw_imu;
  imu_data[1] = hi_pitch_imu;
  imu_data[2] = hi_roll_imu;
  imu_data[3] = hi_g_x_imu;
  imu_data[4] = hi_g_y_imu;
  imu_data[5] = hi_g_z_imu;
  imu_data[6] = hi_a_x_imu;
  imu_data[7] = hi_a_y_imu;
  imu_data[8] = hi_a_z_imu;
  pthread_rwlock_unlock(&rwlock);
}

int hipnuc_init(void) {
  pthread_t imu_thread;
  int ret = -1;
  pthread_rwlock_init(&rwlock, NULL);
  atomic_store(&exit_flag, false);

  hipnuc_open("/dev/ttyUSB0");

  ret = pthread_create(&imu_thread, NULL, hip_imu_run, NULL);
  if (ret != 0) {
    printf("creat imu_thread fail.\n");
    return -1;
  }

  // while (1) {
  //   // printf("%d\r\n", hi_imu_count);
  //   hi_imu_count = 0;
  //   sleep(1);
  // }

  return 0;
}

int hipnuc_close(void) {
  atomic_store(&exit_flag, true);
  pthread_rwlock_destroy(&rwlock);
  return 0;
}
