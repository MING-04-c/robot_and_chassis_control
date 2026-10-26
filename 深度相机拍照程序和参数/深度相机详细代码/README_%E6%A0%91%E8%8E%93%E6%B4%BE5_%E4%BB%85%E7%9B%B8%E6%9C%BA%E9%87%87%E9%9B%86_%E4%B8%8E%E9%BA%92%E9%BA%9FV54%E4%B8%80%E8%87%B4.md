# 树莓派 5 ARM64 — Orbbec V5.4 原版相机采集包（只有相机）

## 0. 交付边界与现状

本包**只用于树莓派 5 拍摄一组 RGB + 深度、读取本次相机参数并保存到磁盘**。不包含 U-Net、二维 P1/P2、XYZ 计算、局域网发送、ROS、机械臂控制。

来源是麒麟机器实际跑过的 `paint-v54 camera` 相机链路。下列 **4 个相机核心 .py 文件逐字节不变**：

- `paint_rgbd_v5_4_camera_worker.py`：冻结版，真正调用旧 SDK Pipeline；
- `paint_rgbd_legacy_auto_recapture_v4.py`：冻结版 Profile 常量、数组/内参检查；
- `orbbec_rgbd_auto_capture_v1.py`：冻结版 Profile 选择、帧解码；
- `orbbec_d2c_capture_with_provenance_v3.py`：冻结版 SDK 当次内参/畸变存档。

新增但**不修改核心文件**：

- `pi5_capture_only.py`：树莓派 ARM64 单独入口，检测平台、Python、ARM64 SDK 并调用原 worker；最多重试 3 次；完成后校验所有原始文件 SHA256；额外保存 16 位深度 PNG；
- `start_pi5_camera.sh`：快捷启动；
- `print_orbbec_v54_capture_parameters.py`：麒麟先前的只读参数打印工具，字节不变；
- `tests/test_offline.py`：不连接相机的合成数据测试，不应误认为树莓派实测。

**未提供** Orbbec SDK 的 ARM64 二进制文件，也无法仅凭“树莓派 5 ARM64”证明旧 Python SDK v1.1.4 接口能够在树莓派 OS 上运行。原麒麟 SDK 是 `OrbbecSDK_Python_v1.1.4_linux_x64_release/python3.9`，**不可原样复制 .so 到树莓派**。必须另行获得**与你的 Orbbec 相机型号、树莓派系统及 Python ABI 兼容的 ARM64 旧接口 SDK**，包含可导入的 `Pipeline`、`ObTypes` 和软件 D2C 常量；不能用 Python SDK v2 直接替换而不重写接口。不要未经核实升级相机固件。

## 1. 与原麒麟 V5.4 完全一致的采集请求参数

| 项目 | 请求设置（原封不动） |
|---|---|
| Orbbec API | 旧 `Pipeline` + `ObTypes`（麒麟原先是 Python SDK v1.1.4） |
| 相机彩色 Profile | 640×480，30 FPS，MJPG 格式 ID=5，索引=1 |
| 原始深度 Profile | 640×400，30 FPS，深度格式 ID=11，索引=16 |
| 对齐方式 | `OB_PY_ALIGN_D2C_SW_MODE`，软件深度对齐 RGB |
| 对齐后深度尺寸 | 640×480，uint16 |
| 预热 | 跳过前 5 组带 RGB、深度的 Frameset |
| SDK 等待 | 上限 30 次，每次 `waitForFrames(750)` ms |
| RGB-D 时间戳门限 | 旧代码 `abs(systemTimeStamp差) <= 100`，**原始 SDK 单位未独立认证** |
| 完整采集尝试次数 | 默认最多 3 次；可调整为 1—12（只影响失败重试） |
| 相机内参和畸变 | **每次**从同一 Pipeline 会话的 `getCameraParam()` 保存 |
| 毫米/原始深度单位 | **每次**从 `depthFrame.getValueScale()` 保存；不是固定写死 1.0 |
| 曝光、增益、白平衡、激光功率 | **原 V5.4 没有明确设置/保存**，因此无法保证在不同平台完全相同 |
| 帧率 | 30 FPS 是所选流的 Profile，不等于这份单次采集程序连续保存 30 FPS |

注意：Profile 的**索引** 1、16 取自麒麟所用 SDK 的可枚举 Profile 列表。ARM 版本若 Profile 列表顺序不同，原代码将**安全报错**。先保留不动，取得树莓派实际 Profile 清单后再做有证据的最小适配；不能直接删掉索引检查，否则会降低“与原版本一致”的可证性。

## 2. 树莓派上先做只读系统检查

```bash
uname -m
cat /etc/os-release
python3 --version
command -v python3.9 || true
lsusb
lsusb -t
```

预期 `uname -m` 为 `aarch64`。普通树莓派 OS 可能预装 Python3.11，而原冻结绑定的运行环境为 Python3.9；如果缺少 Python3.9，请先确定与你获取的 ARM 旧 SDK 绑定一致的 Python ABI，**不要直接把旧 cp39 二进制装进 Python3.11**。

核对第三方依赖是否装在准备实际运行 SDK 的 Python 中：

```bash
/path/to/python3.9 -c 'import sys,cv2,numpy; print(sys.version); print(cv2.__version__,numpy.__version__)'
```

如果这条命令失败，先解决 Pi 本机的环境依赖；不要在麒麟电脑的 `industrial_ai` 环境里安装替代。

## 3. 将 ZIP 解压到树莓派（无须改动原麒麟工程）

例如传到 `/home/pi/`，解压得到包含 `pi5_capture_only.py` 的目录。以下命令均在**树莓派终端**运行，先执行：

```bash
cd /home/pi/PaintV54_RPi5_ARM64_CameraOnly_20261019
sha256sum -c SHA256SUMS.txt
```

必须全部显示 `OK`。若目录位置不同，只改 `cd` 路径即可。

## 4. 先只检查 SDK，不打开相机

前提：树莓派上已经有兼容原 API 的**ARM64 版** SDK 根目录，具有：

```
/实际ARM64旧SDK路径/python3.9/
├── lib/
│   ├── python_lib/   # Pipeline、ObTypes 对应 Python 绑定及原 SDK 模块
│   └── c_lib/        # ARM64 本地 Orbbec 库
└── Samples/          # 可选，按原路径暴露
```

命令示例，**将路径替换为真实路径**，不要照抄占位符：

```bash
/path/to/python3.9 -B pi5_capture_only.py check \
  --sdk-root /实际ARM64旧SDK路径/python3.9
```

成功应见到：

```
V5_LEGACY_CAMERA_IMPORT_PASS_NO_CAMERA_START
PI5_LEGACY_SDK_PREFLIGHT_PASS_NO_CAMERA_STARTED
```

这个阶段只证明旧 SDK 的模块/常量可以导入，不证明相机能开启。若出现 `SDK_NOT_ARM64_ELF`、`Pipeline`/`ObTypes` 导入失败、缺少 `SW-D2C` 等，**停止并记录日志**，不要拷入麒麟 x64 `.so`，也不要直接换成 `pyorbbecsdk2`。

## 5. 启动一次 RGB-D 拍照

确认 Orbbec 相机连接树莓派 USB，并且第 4 步成功后：

```bash
/path/to/python3.9 -B pi5_capture_only.py capture \
  --sdk-root /实际ARM64旧SDK路径/python3.9 \
  --out-root /home/pi/orbbec_v54_pi_captures \
  --max-capture-attempts 3
```

或者设置好环境变量，使用便捷启动器：

```bash
export PI_ORBBEC_SDK_ROOT=/实际ARM64旧SDK路径/python3.9
export PI_CAMERA_PYTHON=/path/to/python3.9
./start_pi5_camera.sh
```

便捷入口会自动从脚本目录找到代码，因此不依赖当前工作目录；但 **Pi 上尚未安装/确认的 SDK 和 Python 环境不能靠脚本自动生成**。

若出现 `REQUIRED_PROFILE_NOT_FOUND` 或 `EXPECTED_PROFILE_INDEX_MISMATCH`，说明树莓派的 SDK 实际枚举不同；原脚本故意拒绝开始错误配置采集。把 `sdk_01.log` 的 `PROFILE_AVAILABLE`、`MATCH_INDICES`、`SELECTED_PROFILE` 输出带回来再修适配层。

## 6. 成功后保存的文件（与麒麟原格式兼容）

```
/home/pi/orbbec_v54_pi_captures/
└── pi5_camera_v54_时间戳_唯一后缀/
    ├── sdk_01.log
    └── attempt_01/
        ├── rgb_001.png                        # 原版彩色 PNG（640×480）
        ├── d2c_depth_raw.npy                  # 原版 uint16 深度矩阵（480×640）
        ├── camera_parameters_same_session.json # 原版同一 SDK 会话的内参、畸变等
        ├── capture_manifest.json               # 原版深度比例、同帧证据、源数据SHA256
        ├── CAPTURE_COMPLETE.json                # 原版完成标记
        ├── d2c_depth_raw_uint16.png             # **额外新增**16位无损深度图
        └── depth_png_check.json                 # 新增PNG/NPY一致性及哈希证据
```

前五个文件**由原 V5.4 冻结采集器直接生成**，后两个是 Pi 包独立附加，不覆盖或修改前五个。PNG 颜色图是看得见的照片；深度 PNG 是**单通道 uint16 数值图**，普通图片查看器可能很黑，但每个像素的原始深度值被保留。

例如 `d2c_depth_raw_uint16.png` 中某像素值为 539，若当次 `depth_scale_mm_per_raw=1.0`，则深度约为539mm。换平台时应**以当次 manifest 的真实比例为准**。不允许对原始深度做8位拉伸或伪彩转换后把结果当作毫米图。

成功标记：`PI5_CAMERA_CAPTURE_PASS_NO_UNET_NO_ROBOT`，进程退出码 0。原采集器首先保存并检查三份源文件和 manifest，然后 Pi 启动器再次校验源 SHA256，并验证深度 PNG 与 NPY 逐像素相同；其中任一检查失败，就不会打印成功标记。

## 7. 查看本次实际相机内参和深度比例

注意这里读取的是树莓派**此次采集**的参数，不是麒麟旧文件：

```bash
/path/to/python3.9 -B print_orbbec_v54_capture_parameters.py \
  --capture '/home/pi/orbbec_v54_pi_captures/pi5_camera_v54_具体时间/attempt_01'
```

输出包含：本次 `rgb_intrinsic`、`d2c_intrinsic`、`rgb_distortion`、`d2c_distortion`、`sdk_isMirrored`、`depth_scale_mm_per_raw`、`intrinsic_used`、RGB/Depth 原始时间戳及哈希证据。以上字段为已验证的旧 SDK 快照；`device_serial` 原版记为 `null`，曝光/增益/白平衡没有认证读数，不能伪称“都一样”。

## 8. SDK 安装的严谨结论

- **核心采集代码完全相同** ≠ **x64 与 ARM64 SDK 二进制相同** ≠ **采集标定数值一定相同**。
- 只有在树莓派上的 `Pipeline`/`ObTypes` 旧接口、640×480/640×400 Profile、软件 D2C、深度比例和当次内参都验证通过后，才能称为“树莓派端与麒麟 V5.4 采集协议一致”。
- 需要保留原来的 JSON/NPY，即使只需要向麒麟传照片，也不能漏掉本次深度比例和内参，否则后续 XYZ 存在系统性风险。
- 当前版本只完成代码审查、模拟数据回归、架构检查；**尚未在用户树莓派 5 上用真实 Orbbec 相机进行采集**。
- 将来如果 Raspberry Pi 5 只有新版 `pyorbbecsdk2` 可用，就必须**只重写 Pi 端 SDK 适配层并重新验证输出合同**；严禁直接替换导入后声称与旧代码行为一致。

## 9. 官方 SDK 参考（用于寻找 ARM 版本，不是本包自带）

- Orbbec Python SDK v1 分支：https://github.com/orbbec/pyorbbecsdk/tree/main
- OrbbecSDK v1 ARM64 安装资料：https://github.com/orbbec/OrbbecSDK/blob/main/doc/tutorial/English/Installation_guidance.md
- 最新 Python SDK v2（API 不兼容旧脚本）：https://github.com/orbbec/pyorbbecsdk/tree/v2-main
