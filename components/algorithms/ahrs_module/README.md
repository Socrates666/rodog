# AHRS 九轴姿态解算模块（C99）

> 版本：`1.0.0`  
> 作者：`<AUTHOR_PLACEHOLDER>`

本模块提供一个与平台/硬件解耦的九轴姿态解算实现，采用分层架构：

- **核心算法层**：纯 C99，无硬件依赖
- **适配层**：函数指针注册硬件接口
- **可选流水线层**：条件编译启用内部线程（FreeRTOS）

---

## 1. 目录结构

- `include/`：公开头文件（Doxygen 注释）
- `src/`：算法与接口实现
- `examples/`：裸机 + RTOS 使用示例
- `tests/`：Unity 单元测试入口

---

## 2. 功能特性

- Madgwick IMU 姿态解算
- 低通滤波预处理（加速度/陀螺仪/磁力计）
- 陀螺仪静止去偏置
- 加速度计校准（六面法，在线 min/max）
- 磁力计校准（硬铁偏置 + 软铁对角矩阵近似）
- 输出四元数、欧拉角、重力向量、线性加速度
- 纯 `float` 运算，适合 MCU
- 无动态内存分配，可重入、多实例

---

## 3. 条件编译开关

在 [include/ahrs_config.h](include/ahrs_config.h) 中配置：

- `AHRS_CFG_USE_MAG`：启用/禁用磁力计处理
- `AHRS_CFG_FILTER_MADGWICK`：选择 Madgwick 滤波
- `AHRS_CFG_ENABLE_PIPELINE`：启用流水线层
- `AHRS_CFG_PIPELINE_USE_FREERTOS`：流水线使用 FreeRTOS 后端

---

## 4. 核心 API 概览

### 核心层

- `ahrs_core_default_config()`：获取默认参数
- `ahrs_core_init()`：初始化实例
- `ahrs_core_set_config()` / `ahrs_core_get_config()`：动态调参
- `ahrs_core_load_calibration()` / `ahrs_core_get_calibration()`：加载/导出校准参数
- `ahrs_core_update()`：输入一帧数据并输出融合结果

### 校准层

- `ahrs_calibration_init()`：初始化校准参数
- `ahrs_calibration_accel_online_update()`：累积加计在线数据
- `ahrs_calibration_accel_online_finalize()`：完成六面标定
- `ahrs_calibration_mag_online_update()`：累积磁力计在线数据
- `ahrs_calibration_mag_online_finalize()`：完成硬铁/软铁近似标定

### 适配层

- `ahrs_adapter_init()`：注册硬件读数/时间函数
- `ahrs_adapter_poll_once()`：执行单次读取 + 融合

### 流水线层（可选）

- `ahrs_pipeline_init()`
- `ahrs_pipeline_start()`
- `ahrs_pipeline_stop()`
- `ahrs_pipeline_get_latest()`

---

## 5. 快速使用

### 5.1 裸机

参考 [examples/bare_metal_example.c](examples/bare_metal_example.c)。

流程：
1. 初始化 `ahrs_core_t`
2. 注册 `ahrs_hw_if_t`
3. 周期调用 `ahrs_adapter_poll_once()`

### 5.2 RTOS（FreeRTOS）

参考 [examples/rtos_example.c](examples/rtos_example.c)。

流程：
1. 打开 `AHRS_CFG_ENABLE_PIPELINE` 与 `AHRS_CFG_PIPELINE_USE_FREERTOS`
2. 初始化 `ahrs_pipeline_t`
3. 启动内部线程并读取 `latest_output`

---

## 6. 构建

该目录已提供 [CMakeLists.txt](CMakeLists.txt)，并采用 C99：

- `target_compile_features(... c_std_99)`

在 ESP-IDF 项目中，放在 `components/algorithms` 下会自动参与组件构建。

---

## 7. 单元测试入口

- 入口文件：[tests/test_main.c](tests/test_main.c)
- 测试框架：Unity

可根据项目测试系统将该文件纳入测试工程。

---

## 8. 校准参数持久化建议

模块只负责计算，不绑定存储介质。建议业务层将 `ahrs_calibration_t`：

- 通过 NVS/Flash/EEPROM 持久化
- 启动时调用 `ahrs_core_load_calibration()` 恢复

---

## 9. 注意事项

- 当前滤波主路径为 Madgwick IMU 模式；磁力计接口保留并可用于后续扩展 MARG 全量更新。
- 欧拉角存在万向节锁风险，建议控制链路优先使用四元数。
