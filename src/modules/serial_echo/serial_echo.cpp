#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/pwm_control.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

extern "C" __EXPORT int serial_echo_main(int argc, char *argv[]);

class SerialEcho : public ModuleBase<SerialEcho>
{
public:

    SerialEcho() = default;
    virtual ~SerialEcho() = default;

    // ================= 必须实现 =================
    static int print_usage(const char *reason = nullptr)
    {
        if (reason) PX4_WARN("%s", reason);
        PX4_INFO("Usage: serial_echo start");
        return 0;
    }

    static int custom_command(int argc, char *argv[])
    {
        return print_usage("Unknown command");
    }

    static SerialEcho *instantiate(int argc, char *argv[])
    {
        return new SerialEcho();
    }

    static int task_spawn(int argc, char *argv[])
    {
        _task_id = px4_task_spawn_cmd(
            "serial_echo",
            SCHED_DEFAULT,
            SCHED_PRIORITY_DEFAULT,
            2000,
            (px4_main_t)&run_trampoline,
            argv);

        if (_task_id < 0) {
            PX4_ERR("task start failed");
            return -errno;
        }

        return 0;
    }

    // ================= 核心线程入口 =================
    void run() override
    {
        PX4_INFO("serial_echo running...");

        // 打开串口
        int fd = ::open("/dev/ttyS4", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            PX4_ERR("open /dev/ttyS4 failed (%d)", errno);
            return;
        }

        struct termios cfg{};
        tcgetattr(fd, &cfg);
        cfmakeraw(&cfg);
        cfsetispeed(&cfg, B115200);
        cfsetospeed(&cfg, B115200);
        tcsetattr(fd, TCSANOW, &cfg);

        // uORB 发布器
        uORB::Publication<pwm_control_s> pwm_pub{ORB_ID(pwm_control)};

        char buf[256];
        char cmd_buffer[512];   // 命令累加缓冲区
        int cmd_len = 0;

        while (!should_exit()) {

            ssize_t n = read(fd, buf, sizeof(buf));

            if (n > 0) {

                // 回显收到的数据
                write(fd, buf, n);

                // 累加到命令缓冲
                for (int i = 0; i < n; i++) {
                    if (cmd_len < (int)sizeof(cmd_buffer)-1) {
                        cmd_buffer[cmd_len++] = buf[i];
                    }

                    // 完整命令：以 ';' 结尾
                    if (buf[i] == ';') {

                        cmd_buffer[cmd_len] = '\0';

                        process_command(cmd_buffer, cmd_len, pwm_pub);

                        cmd_len = 0; // 清空
                    }
                }
            }

            px4_usleep(2000);
        }

        PX4_INFO("serial_echo exit");
        close(fd);
    }


private:

    // ================= 指令解析 =================
    void process_command(const char *cmd, int len, uORB::Publication<pwm_control_s> &pub)
    {
        if (len <= 0) return;

        const char *p = strstr(cmd, "-prints:");
        if (!p) return;

        p += 8;  // 跳过 -prints:

        // 去掉结尾的 ';'
        int cmd_len = len - (p - cmd) - 1;
        if (cmd_len <= 0 || cmd_len >= 32) return;

        char cbuf[32]{};
        strncpy(cbuf, p, cmd_len);

        pwm_control_s msg{};
        msg.timestamp = hrt_absolute_time();

        //
        if (strncmp(cbuf, "AA", 2) == 0 && cmd_len >= 8) {
		char port_str[3] = {0};
		char duty_str[3] = {0};
		char freq_str[5] = {0};

		memcpy(port_str, cbuf + 2, 2);
		memcpy(duty_str, cbuf + 4, 2);
		memcpy(freq_str, cbuf + 6, 4);


		msg.port = atoi(port_str);
		msg.duty = atoi(duty_str) / 100.0f;
		msg.frequency = atoi(freq_str);


            PX4_INFO("Parsed AA: port=%d duty=%.2f freq=%d", msg.port, (double)msg.duty, msg.frequency);
        }

        else {
            PX4_WARN("Unknown cmd: %s", cbuf);
            return;
        }

        // 发布 uORB
        pub.publish(msg);
    }

};

extern "C" __EXPORT int serial_echo_main(int argc, char *argv[])
{
    return SerialEcho::main(argc, argv);
}
