/****************************************************************************
 * pwm_hw_out.cpp
 *
 * Subscribe to pwm_control uORB and output PWM on FMU (v6X) using up_pwm_servo_*
 * frequency field is ignored (hardware/driver does not support runtime frequency change)
 ****************************************************************************/

// 包含PX4平台通用的配置、模块、日志和任务头文件
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

// 包含uORB订阅功能和pwm_control主题的头文件
#include <uORB/Subscription.hpp>
#include <uORB/topics/pwm_control.h>

// 包含标准C库头文件
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>

// 使用extern "C"声明底层PWM驱动API（C语言实现）
extern "C" {
    // 初始化PWM硬件，pwm_mask指定要初始化的通道位掩码
    int up_pwm_servo_init(uint32_t pwm_mask);
    // 设置指定通道的PWM脉宽（单位微秒）
    void up_pwm_servo_set(unsigned channel, uint16_t pwm);
    // 反初始化PWM硬件
    int up_pwm_servo_deinit(uint32_t pwm_mask);
    // 更新PWM输出，通知硬件立即应用新的占空比
    void up_pwm_update(uint32_t pwm_mask);
    // 解锁或锁定PWM输出（安全开关）
    void up_pwm_servo_arm(bool arm, uint32_t pwm_mask);
}

// 声明模块的主入口函数，__EXPORT宏确保符号可见
extern "C" __EXPORT int pwm_hw_out_main(int argc, char *argv[]);

// PWMHW_OUT类定义，继承自PX4模块模板基类
class PWMHW_OUT : public ModuleBase<PWMHW_OUT>
{
public:
    // 默认构造函数和析构函数
    PWMHW_OUT() = default;
    ~PWMHW_OUT() override = default;

    // 静态方法声明：实例化、任务生成、使用说明和自定义命令处理
    static PWMHW_OUT *instantiate(int argc, char *argv[]);
    static int task_spawn(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]);
    // 主运行循环和停止函数
    void run() override;
    void stop();

private:
    // uORB订阅，监听pwm_control主题
    uORB::Subscription _sub{ORB_ID(pwm_control)};

    // 常量定义：最大支持端口数和对应的位掩码
    static constexpr unsigned MAX_PORTS = 8;
    static constexpr uint32_t ALL_MASK = ((1u << MAX_PORTS) - 1); // 生成0b11111111掩码，对应通道0-7

    // 标记PWM硬件是否已成功初始化
    bool _initialized{false};
};

// 实例化函数，创建类实例（忽略命令行参数）
PWMHW_OUT *PWMHW_OUT::instantiate(int, char **)
{
    return new PWMHW_OUT();
}

// 任务生成函数，由PX4模块框架调用启动新任务
int PWMHW_OUT::task_spawn(int argc, char *argv[])
{
    // 使用px4_task_spawn_cmd创建新任务
    _task_id = px4_task_spawn_cmd("pwm_hw_out",
                      SCHED_DEFAULT,                 // 默认调度策略
                      SCHED_PRIORITY_DEFAULT,        // 默认优先级
                      2000,                          // 栈大小
                      (px4_main_t)&run_trampoline,   // 入口函数（PX4框架提供）
                      nullptr);                       // 参数

    // 检查任务是否创建成功
    if (_task_id < 0) {
        PX4_ERR("task start failed");                // 打印错误信息
        return -errno;                               // 返回错误码
    }
    return PX4_OK;                                   // 返回成功
}

// 打印使用说明的函数
int PWMHW_OUT::print_usage(const char *reason)
{
    if (reason) PX4_WARN("%s", reason);              // 如果有原因，打印警告
    PRINT_MODULE_USAGE_NAME("pwm_hw_out", "driver"); // 模块名称和类型
    PRINT_MODULE_USAGE_COMMAND("start");             // 支持的命令
    return 0;
}

// 停止函数，清理PWM硬件资源
void PWMHW_OUT::stop()
{
    // 如果已初始化，则执行清理操作
    if (_initialized) {
        up_pwm_servo_arm(false, ALL_MASK);           // 先锁定PWM输出（安全第一）
        up_pwm_servo_deinit(ALL_MASK);               // 反初始化PWM硬件
        _initialized = false;                        // 更新初始化状态
    }
    PX4_INFO("pwm_hw_out stopped");                  // 打印停止信息
}

// 主运行循环函数，模块的核心逻辑
void PWMHW_OUT::run()
{
    // 打印启动信息，显示尝试初始化的端口范围
    PX4_INFO("pwm_hw_out started; attempting to init PWM for ports 1..%u", MAX_PORTS);

    // 尝试初始化PWM硬件，ALL_MASK指定所有通道
    int ret = up_pwm_servo_init(ALL_MASK);
    if (ret < 0) {
        // 初始化失败，记录错误但继续运行（以便记录接收到的消息）
        PX4_ERR("up_pwm_servo_init failed: %d", ret);
    } else {
        // 初始化成功，设置标志并解锁PWM输出
        _initialized = true;
        up_pwm_servo_arm(true, ALL_MASK);            // 解锁PWM输出（使能信号）
        PX4_INFO("PWM initialized, mask=0x%02x", ret); // 打印实际初始化的通道掩码
    }

    // 创建pwm_control消息结构体并初始化为零
    pwm_control_s msg{};

    // 主循环，直到收到退出信号
    while (!should_exit()) {
        // 阻塞等待新的uORB消息更新
        if (_sub.update(&msg)) {
            // 验证端口号是否在有效范围内（1-MAX_PORTS）
            if (msg.port < 1 || msg.port > (int)MAX_PORTS) {
                PX4_WARN("pwm_control: invalid port %u", msg.port); // 打印警告
                continue;                                           // 跳过本次处理
            }

            // 对duty cycle进行限幅，确保在[0.0, 1.0]范围内
            float duty = msg.duty;
            if (duty < 0.0f) duty = 0.0f;
            if (duty > 1.0f) duty = 1.0f;

            // 将duty cycle（0-1）映射到PWM脉宽（1000-2000微秒，标准舵机范围）
            // 加0.5f用于四舍五入，提高精度
            uint16_t pwm_us = (uint16_t)(1000 + duty * 1000.0f + 0.5f);

            // 最终安全限幅，防止计算错误导致脉宽超出安全范围
            if (pwm_us < 500) pwm_us = 500;
            if (pwm_us > 2500) pwm_us = 2500;

            // 将端口号（从1开始）转换为通道索引（从0开始）
            unsigned channel = (unsigned)(msg.port - 1);

            // 只有PWM硬件已初始化时才设置输出
            if (_initialized) {
                up_pwm_servo_set(channel, pwm_us);   // 设置指定通道的PWM值
                up_pwm_update(ALL_MASK);              // 通知硬件立即更新所有通道输出[1](@ref)
            }

            // 打印接收到的消息和处理结果（调试信息）
            PX4_INFO("pwm_control recv: ts=%" PRIu64 " port=%u duty=%.3f => %u us (freq ignored)",
                 msg.timestamp, msg.port, (double)msg.duty, pwm_us);
        }

        // 短暂睡眠，避免忙等待，节省CPU资源（uORB更新本身会阻塞）
        px4_usleep(5000);
    }

    // 循环退出前清理资源
    stop();
}

// 自定义命令处理函数（当前未实现特殊命令）
int PWMHW_OUT::custom_command(int argc, char *argv[])
{
    return print_usage("Unknown command");           // 打印使用说明
}

// 模块主函数，PX4模块框架的入口点
int pwm_hw_out_main(int argc, char *argv[])
{
    return PWMHW_OUT::main(argc, argv);              // 调用类的静态main方法
}
