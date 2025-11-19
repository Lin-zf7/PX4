/****************************************************************************
 * Example: serial_app_auto
 * 功能：
 *   - 开机自启
 *   - 接收串口字符
 *   - 如果是数字，返回 +1
 *   - 如果非数字，返回 "NONE"
 ****************************************************************************/

#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/app.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>

extern "C" __EXPORT int serial_app_main(int argc, char *argv[]);

static bool thread_should_exit = false;
static bool thread_running = false;
static px4_task_t thread_task;

// 串口线程
int serial_app_thread(int argc, char *argv[])
{
    const char *uart_name = "/dev/ttyS2"; // 目标串口
    PX4_INFO("Opening UART: %s", uart_name);

    int uart_fd = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (uart_fd < 0) {
        PX4_ERR("Failed to open UART device");
        return -1;
    }

    struct termios uart_config;
    tcgetattr(uart_fd, &uart_config);

    cfsetispeed(&uart_config, B115200);
    cfsetospeed(&uart_config, B115200);

    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    uart_config.c_cflag |= CS8;
    uart_config.c_cflag |= CREAD | CLOCAL;
    uart_config.c_iflag = 0;
    uart_config.c_oflag = 0;
    uart_config.c_lflag = 0;

    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd, TCSANOW, &uart_config);

    PX4_INFO("UART initialized at 115200 baud");

    char buffer[16];

    while (!thread_should_exit) {
        int n = read(uart_fd, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';

            for (int i = 0; i < n; ++i) {
                char c = buffer[i];
                char out_msg[16];

                if (isdigit(c)) {
                    int num = c - '0' + 1;
                    if (num >= 10) num = 0; // 保持一位数字循环
                    snprintf(out_msg, sizeof(out_msg), "%d\r\n", num);
                } else {
                    snprintf(out_msg, sizeof(out_msg), "NONE\r\n");
                }

                write(uart_fd, out_msg, strlen(out_msg));
                PX4_INFO("Received '%c', Sent '%s'", c, out_msg);
            }
        }

        px4_usleep(10000); // 10ms
    }

    close(uart_fd);
    thread_running = false;
    PX4_INFO("serial_app thread exiting");
    return 0;
}

// 主入口
int serial_app_main(int argc, char *argv[])
{
    if (thread_running) {
        PX4_WARN("serial_app already running");
        return 0;
    }

    thread_should_exit = false;

    thread_task = px4_task_spawn_cmd(
        "serial_app",
        SCHED_DEFAULT,
        SCHED_PRIORITY_DEFAULT,
        3000,
        serial_app_thread,
        (char *const *)argv);

    if (thread_task < 0) {
        PX4_ERR("task start failed");
        return -1;
    }

    thread_running = true;
    PX4_INFO("serial_app started automatically");

    return 0;
}
