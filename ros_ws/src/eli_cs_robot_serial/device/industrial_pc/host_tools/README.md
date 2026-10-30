# 麒麟通讯工具

Python 3.8及以上，无需ROS、相机SDK或额外pip依赖。树莓派169.254.199.100，麒麟169.254.199.120。

**当前默认无密钥，不需要industrial_pc.key。** 两边一起更新本版本；保留SHA256、长度检查、断线重连和重复指令去重。

完整启动步骤见 [QUICKSTART.md](QUICKSTART.md)。

## 常用命令

```bash
python3 listen_robot.py                 # 常驻：收图、显示任务与连接状态
python3 start_capture.py                # 另一终端：只采HK一张和RGB/深度一组，不执行运动
python3 start_task.py                   # 启动已加载的运动工作流
python3 send_points.py --file result.json
python3 start_task.py --action resume   # 途中站继续
python3 start_task.py --action stop
python3 start_task.py --action status
```

回执只表示受理；采集/运动完成看后续状态。同一请求重发复用message_id，新一轮用新编号。任务与独立采集互斥。

## 照片现在这样保存

```text
received/
  2026.10.30.20.29/
    hk/
      20.29.12_345_hk.jpg
    RGB/
      20.29.12_345_rgb.png
      20.29.12_345_depth.png
```

第一层包含北京时间年月日时分；第二层只有 `hk/` 和 `RGB/`，没有额外的日期或采集时间目录。HK放海康照片；RGB同时放深度相机的彩色图和原始16位深度图。同一分钟多次采集共用目录，文件名按秒、毫秒区分；同一轮三图使用相同时间前缀。时间完全相同的不同采集会追加编号，重发同一采集不额外保存。旧双HK流第二张用 `_hk_02.jpg` 区分。

目录和前缀以实际HK采集时间为准，不表示两相机同时曝光。Pi和主机通过采集编号及传输的capture_name关联。两端均需更新本版；目录改名不改变深度数值。

ZIP校验和解压完成后即清理，不重复保留NPY与清单。集中索引 `.capture_index.json`、锁和监听状态保留在根目录，保存图像配对、参数、深度比例和去重信息。旧日期/时间目录仍可读取，新采集才使用新目录；要整理旧图可使用后面的复制工具。

主机读取每组文件和参数：

```python
import json
from pathlib import Path
from capture_storage import file_path
root = Path('received')
index = json.loads((root/'.capture_index.json').read_text())
for task_id, record in index['sessions'].items():
    if 'rgbd' not in record:
        continue
    rgb_file = file_path(root, record, 'rgb.png')
    depth_file = file_path(root, record, 'depth.png')
    params = record['rgbd']['camera_parameters']
    scale = record['rgbd']['depth_scale_mm_per_raw']
    manifest_sha256 = record['rgbd']['capture_manifest_sha256']
    # 原始深度像素乘scale得到毫米；内参和比例按本轮实际数据读取。
```

## 配置和通道

config.json中的robot_host为树莓派IP；output_dir为照片目录，相对路径以config所在目录为基准。默认use_shared_key=false。

- 9010：麒麟连接Pi，命令/状态/检测点，带长度头和SHA256。
- 9002：麒麟连接Pi，HK照片。任务采完才临时开放；waiting_sender是正常等待。
- 9030：Pi连接麒麟，RGB-D包；校验并保存图片后才回ACK，监听持续运行。

HK原通道仍无落盘ACK，以麒麟“保存完成”和文件为准。深度收到校验ACK才认为发送成功。SHA256是完整性校验，不冒充身份认证。

## 整理旧接收数据

旧数据可复制成新格式，源目录不会修改：

```bash
python3 capture_storage.py --source received --output received_clean
```

检查新目录后再自行切换目录；不要在监听正在写入时整理。同目录不能作为输入和输出。

`DEMO_XYZ_800mm.json`等是演示，不是实测点；通讯桥仅发布检测点观测；若Pi运行camera_detection_real默认TCP测试，匹配本次照片的实测点会自动触发探针触碰。线缆格式见 [TCP通讯协议_麒麟工控机.md](TCP通讯协议_麒麟工控机.md)。
# 手选像素点回传（后续通讯使用）

Pi的camera_detection_real默认TCP模式会采新图并等主机点，匹配后自动触碰。手选像素测试需针对本次新图生成结果，再发送：

```bash
python3 send_points.py --pixel-test --file test_pixel_xyz.json --rgb-file received/2026.10.30.20.29/RGB/20.29.12_345_rgb.png
```

Pi改用target_source:=local时不启动通讯，也不使用主机点。
