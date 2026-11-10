# changelog

仓库里的两个目录对应两个版本的核心源码：

- `Rtos_test1/`：第一版（11.03 的工程）
- `Rtos_test2/`：第二版（11.05 的工程）

下面说的"第一版""第二版"就是指这两个目录。

第一版是原版，第二版是改完的版本

李泽锴 改于2026.11.10

---

## 一、任务函数简化

第一版里的任务函数基本是业务代码直接堆在里面，第二版把业务逻辑抽出去了，任务函数只剩一句调用。

### OfflineTask

第一版 的任务体是这样的：

```c
void OfflineTask(void *argument)
{
  int16_t red_pwm = 0;
  int8_t dir = 1;

  for (;;)
  {
    if (gm6020_motor1.offline_flag)
    {
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
      red_pwm += dir;
      if (red_pwm >= 500) { red_pwm = 500; dir = -1; }
      else if (red_pwm <= 0) { red_pwm = 0; dir = 1; }
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, red_pwm);
    }
    else
    {
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 500);
    }
    osDelay(2);
  }
}
```

第二版 变成了：

```c
void OfflineTask(void *argument)
{
  for (;;)
  {
    LED_Update(gm6020_motor1.offline_flag);
    osDelay(1);
  }
}
```

呼吸的算法、`red_pwm` 和 `dir` 这两个状态变量，全部搬到新建的 `LED.c` 里，变成文件内的 static 变量。任务里只留一个判断：调一次 `LED_Update`，把离线标志传进去

### PrintMotorSpeed

第一版的是解析串口指令、算两秒计时、拼字符串打印。中间还声明了 `last_print_time` 和 `tx_buf`：

```c
void PrintMotorSpeed(void *argument)
{
    uint32_t last_print_time = HAL_GetTick();
    static char tx_buf[64];

    for(;;)
    {
      if (UART_RxFlag)
      {
        UART_RxFlag = 0;
        if (strncmp(UART_ProcBuf, "motor", 5) == 0)
          target_rpm = (float)atof(&UART_ProcBuf[5]);
      }
      if (HAL_GetTick() - last_print_time >= 2000)
      {
        sprintf(tx_buf, "RPM:%d TGT:%.1f %s\r\n", ...);
        UART_SendString(tx_buf);
        last_print_time = HAL_GetTick();
      }
      osDelay(10);
    }
}
```

第二版 变成了：

```c
void PrintMotorSpeed(void *argument) {
    for(;;)
    {
      UART_MotorComm_Update(&target_rpm, &gm6020_motor1);
      osDelay(10);
    }
}
```

`UART_MotorComm_Update()` 是新加到 `UART.c` 里的函数，解析和打印都在里面。`last_print_time` 从任务里的局部变量改成了 `UART.c` 文件开头的 `static uint32_t`，`tx_buf` 也从任务里挪进去了

### 改了之后的想法

两个任务函数都从三四十行缩到了十行以内。我的理解是这样分层的：

任务只负责两件事——什么时候被唤醒、多久循环一次。至于"醒来之后具体做什么"，交给各自的模块函数。

好处是之后要改呼吸灯的算法，直接进 `LED.c` 就行，不用在 `freertos.c` 里翻。`freertos.c` 是 CubeMX 生成的，改它的心理负担比改自己写的文件大，抽出去之后这块就清爽了。

另外一个隐性好处：`LED_Update` 和 `UART_MotorComm_Update` 现在不依赖 FreeRTOS，是普通函数，以后想在裸机工程里复用也能直接拷。

---

## 二、函数前面的注释统一补了一遍

第一版 里的注释分两种：一种是单行 `//`，另一种是把参数名和说明连在一起写的 doxygen，比如：

```c
/**
 * @brief  PID参数初始化
 * @param  pid传入要初始化的PID结构体指针
 * @note   把我们在CubeMonitor里调好的参数写进去
 */
```

`pid传入要初始化的PID结构体指针` 这行，参数名和说明之间没有空格，Doxygen 生成文档时会解析不出来。而且 `PID_Init` 实际有六个参数，只写了一个

第二版 里统一改成了参数名和说明分开、每个参数单独一行的写法：

```c
/**
 * @brief  速度环 PID 参数初始化
 * @param  pid       指向 PID 结构体的指针
 * @param  max_out   输出限幅值（GM6020 电压指令满量程约 ±30000）
 * @param  max_iout  积分项限幅值，防止积分饱和
 * @param  kp        比例系数
 * @param  ki        积分系数
 * @param  kd        微分系数
 * @note   把调好的参数写进结构体，随后清空历史状态，防止残留数据让电机乱转
 */
```

涉及的文件和函数：

- `alg_pid.c`：`PID_Init`、`PID_Calc`、`PID_Reset`
- `dev_motor_gm6020.c`：`GM6020_Init`、`GM6020_SendCommand`、`GM6020_WDG`
- `UART.c`：`UART_Init`、`UART_SendString`、`HAL_UARTEx_RxEventCallback`、`UART_MotorComm_Update`
- `Servo.c`：`Servo_Init`、`Servo_Step`
- `Key.c`：`HAL_GPIO_EXTI_Callback`
- `LED.c`：`LED_Init`、`LED_Update`（新文件，一开始就按这个格式写）

内容上补的主要是三类信息：

**一是单位。** `fdb` 和 `set` 都标了 rpm，`GM6020_SendCommand` 的四个参数标了"电压指令"而不是含糊地说"指令值"

**二是量程。** 比如 `max_out` 那行写了"GM6020 电压指令满量程约 ±30000"，`GM6020_SendCommand` 的 note 里写了"有效范围约 ±30000，对应 ±24V"。这样看到数字就知道合不合理

**三是调用要求。** `UART_SendString` 的 note 里写了"DMA 发送期间缓冲区必须持续有效"，`GM6020_WDG` 的 note 里写了"调用周期必须明显小于 50ms"

写这些的时候我尽量写"为什么要这么用"，而不是把函数名翻译一遍。像 `PID_Calc` 那个 note，写的是"误差 = 目标 - 反馈；P、I、D 三项相加后先做积分限幅、再做总输出限幅"——讲的是执行顺序

---

## 三、任务属性的调整

这块不算重构，是随封装一起动的一些参数。

### PWM 初始化从 main 里挪进模块

第一版 的 `main.c` 里，初始化 LED 的 PWM 是直接摊开的：

```c
HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
```

第二版 换成了：

```c
LED_Init();
```

`LED_Init()` 内部做同样四件事。这个改动跟前面把呼吸算法抽出去是一套的——既然算法进了 `LED.c`，初始化也一起搬进去比较顺。`main.c` 现在就表达"要初始化 LED"这个意思，不用关心它占哪两个通道

### 打印任务的栈从 512 字改成 256 字

`PrintMotorTask` 的 `stack_size`，03 是 `512 * 4`，05 是 `256 * 4`，也就是从 2KB 降到 1KB

当时想的是任务函数现在只剩一行调用了，栈应该用不了那么多，省 1KB 下来

但这个判断我觉得有问题。栈的消耗不在任务函数本身，在它调用的东西链上。`UART_MotorComm_Update` 里面有 `sprintf` 加 `%.1f`，`vfprintf` 内部的格式化缓冲区全在栈上，这条链的深度并没有因为封装而变浅

所以这个调整严格说是偏冒险的，跑起来要盯一下栈的水位。真出问题的话应该改回 512

### osDelay 的周期跟着改了一个数

`OfflineTask` 里的 `osDelay` 从 2 改成 1

原因是呼吸的步进值也换了：第一版在任务里是自己算的，上限 500、每 2ms 加一，单程 1 秒；第二版 的 `LED_Update` 里上限是 1000、每 1ms 加一，周期看起来也是 2 秒

---

## 小结

1. 任务函数原来什么都干，现在只留调度骨架，业务逻辑分到 `LED.c` 和 `UART.c` 各自的函数里
2. 注释从"随手写一行"变成带参数说明的格式，重点补了单位、量程和调用约束
3. 初始化位置和任务参数（栈、周期）跟着封装一起调整，其中栈降到 256 这个决定还需要实测确认

整体感觉是：代码量没变少，但分布变得更清楚了
