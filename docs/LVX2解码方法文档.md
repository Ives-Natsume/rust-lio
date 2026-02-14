# LVX2文件解码为Livox官方协议数据流的方法文档

## 文档版本
- **版本**: v1.0
- **日期**: 2026-02-11
- **适用设备**: Livox Mid-360
- **参考标准**: Livox SDK2 Communication Protocol v1.4.11

---

## 1. 概述

### 1.1 背景
LVX2是Livox官方定义的点云数据存储格式，用于离线保存和回放激光雷达数据。本文档详细说明如何将LVX2文件解码回Livox官方的UDP通信协议格式，以便在需要时重建原始数据流或进行自定义数据处理。

### 1.2 数据流关系
```
原始UDP数据流 (PCAP) → LVX2文件 (存储) → UDP协议数据 (解码)
    ↓                      ↓                    ↓
  端口56300点云        文件结构化存储         恢复协议格式
  端口56400 IMU         压缩存储              用于处理分析
```

### 1.3 核心端口说明
根据Livox Mid-360通信协议：
- **端口56300**: 点云数据（Point Cloud Data）
- **端口56400**: IMU数据（Inertial Measurement Unit）
- **端口56200**: 推送消息（设备信息）
- **端口56100**: 控制命令

---

## 2. LVX2文件格式详解

### 2.1 整体结构
```
┌─────────────────────────────────────┐
│      公共头 (Public Header)          │  24 字节
├─────────────────────────────────────┤
│      私有头 (Private Header)         │   5 字节
├─────────────────────────────────────┤
│      设备信息 (Device Info)          │  63 字节
├─────────────────────────────────────┤
│      帧0 (Frame 0)                   │
│  ┌─────────────────────────────┐   │
│  │  帧头 (Frame Header)        │   │  24 字节
│  ├─────────────────────────────┤   │
│  │  数据包1 (Package 1)        │   │  27字节头 + N字节数据
│  │  数据包2 (Package 2)        │   │  27字节头 + N字节数据
│  │  ...                        │   │
│  └─────────────────────────────┘   │
├─────────────────────────────────────┤
│      帧1 (Frame 1)                   │
│  ...                                │
├─────────────────────────────────────┤
│      帧N (Frame N)                   │
└─────────────────────────────────────┘
```

### 2.2 公共头结构 (24字节)

| 字段名 | 偏移量 | 长度 | 类型 | 说明 |
|--------|--------|------|------|------|
| signature | 0 | 16 | char[16] | 固定为"livox_tech"，用'\0'填充 |
| ver_a | 16 | 1 | uint8 | 主版本号，固定为2 |
| ver_b | 17 | 1 | uint8 | 次版本号，通常为0 |
| ver_c | 18 | 1 | uint8 | 修订版本号，通常为0 |
| ver_d | 19 | 1 | uint8 | 构建版本号，通常为0 |
| magic_code | 20 | 4 | uint32 | 魔术字，固定为0xAC0EA767 |

### 2.3 私有头结构 (5字节)

| 字段名 | 偏移量 | 长度 | 类型 | 说明 |
|--------|--------|------|------|------|
| duration | 0 | 4 | uint32 | 帧持续时间（固定50ms） |
| device_count | 4 | 1 | uint8 | 设备数量（通常为1） |

### 2.4 设备信息结构 (63字节)

| 字段名 | 偏移量 | 长度 | 类型 | 说明 |
|--------|--------|------|------|------|
| lidar_sn | 0 | 16 | char[16] | 激光雷达序列号 |
| hub_sn | 16 | 16 | char[16] | Hub序列号 |
| lidar_id | 32 | 4 | uint32 | 雷达ID（从IP地址转换） |
| lidar_type | 36 | 1 | uint8 | 雷达类型（247=Mid-360） |
| device_type | 37 | 1 | uint8 | 设备类型（固定9） |
| enable_extrinsic | 38 | 1 | uint8 | 是否启用外参（0/1） |
| offset_roll | 39 | 4 | float | 翻滚角偏移（单位：度） |
| offset_pitch | 43 | 4 | float | 俯仰角偏移（单位：度） |
| offset_yaw | 47 | 4 | float | 偏航角偏移（单位：度） |
| offset_x | 51 | 4 | float | X轴位置偏移（单位：米） |
| offset_y | 55 | 4 | float | Y轴位置偏移（单位：米） |
| offset_z | 59 | 4 | float | Z轴位置偏移（单位：米） |

### 2.5 帧头结构 (24字节)

| 字段名 | 偏移量 | 长度 | 类型 | 说明 |
|--------|--------|------|------|------|
| current_offset | 0 | 8 | uint64 | 当前帧在文件中的偏移量 |
| next_offset | 8 | 8 | uint64 | 下一帧的偏移量 |
| frame_index | 16 | 8 | uint64 | 帧索引号（从0开始递增） |

### 2.6 LVX2数据包头结构 (27字节)

| 字段名 | 偏移量 | 长度 | 类型 | 说明 |
|--------|--------|------|------|------|
| version | 0 | 1 | uint8 | 协议版本（对应UDP payload偏移0） |
| lidar_id | 1 | 4 | uint32 | 雷达ID |
| lidar_type | 5 | 1 | uint8 | 数据包中的雷达类型（固定8） |
| timestamp_type | 6 | 1 | uint8 | 时间戳类型（对应UDP offset 11） |
| timestamp | 7 | 8 | uint64 | 时间戳，单位：纳秒（对应UDP offset 28） |
| udp_count | 15 | 2 | uint16 | UDP包计数（对应UDP offset 7） |
| data_type | 17 | 1 | uint8 | 数据类型（对应UDP offset 10） |
| length | 18 | 4 | uint32 | 点云数据长度 |
| frame_count | 22 | 1 | uint8 | 帧内包计数（对应UDP offset 9） |
| reserved | 23 | 4 | uint8[4] | 保留字段（全0） |

---

## 3. UDP协议数据格式

### 3.1 UDP点云数据包格式（端口56300）

#### 3.1.1 UDP Payload头部 (36字节)

| 字段名 | 偏移量 | 长度 | 类型 | 说明 |
|--------|--------|------|------|------|
| version | 0 | 1 | uint8 | 包协议版本（当前为0） |
| length | 1 | 2 | uint16 | UDP数据段总长度（从version开始） |
| time_interval | 3 | 2 | uint16 | 帧内点云采样时间间隔（单位：0.1μs） |
| dot_num | 5 | 2 | uint16 | 当前UDP包中的点数 |
| udp_cnt | 7 | 2 | uint16 | 点云UDP包计数，每包递增1 |
| frame_cnt | 9 | 1 | uint8 | 点云帧计数，每帧加1 |
| data_type | 10 | 1 | uint8 | 数据类型（见3.1.3节） |
| time_type | 11 | 1 | uint8 | 时间戳类型（0=无同步，1=PTP，2=GPS） |
| reserved | 12 | 12 | uint8[12] | 保留字段 |
| crc32 | 24 | 4 | uint32 | CRC-32校验和（时间戳+数据段） |
| timestamp | 28 | 8 | uint64 | 点云时间戳（单位：纳秒） |

#### 3.1.2 点云数据段 (从偏移36开始)

根据data_type的不同，每个点的数据结构不同：

**Data Type 1: 笛卡尔坐标（32位）- 每点14字节**
| 字段 | 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|------|
| x | 0 | 4 | int32 | X坐标（单位：毫米） |
| y | 4 | 4 | int32 | Y坐标（单位：毫米） |
| z | 8 | 4 | int32 | Z坐标（单位：毫米） |
| reflectivity | 12 | 1 | uint8 | 反射率 |
| tag | 13 | 1 | uint8 | 点标记信息 |

**Data Type 2: 笛卡尔坐标（16位）- 每点8字节**
| 字段 | 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|------|
| x | 0 | 2 | int16 | X坐标（单位：厘米） |
| y | 2 | 2 | int16 | Y坐标（单位：厘米） |
| z | 4 | 2 | int16 | Z坐标（单位：厘米） |
| reflectivity | 6 | 1 | uint8 | 反射率 |
| tag | 7 | 1 | uint8 | 点标记信息 |

**Data Type 3: 球坐标 - 每点10字节**
| 字段 | 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|------|
| depth | 0 | 4 | uint32 | 深度（单位：毫米） |
| theta | 4 | 2 | uint16 | 天顶角[0,18000]（单位：0.01度） |
| phi | 6 | 2 | uint16 | 方位角[0,36000]（单位：0.01度） |
| reflectivity | 8 | 1 | uint8 | 反射率 |
| tag | 9 | 1 | uint8 | 点标记信息 |

#### 3.1.3 数据类型定义

| data_type值 | 说明 | 点数/包 |
|-------------|------|---------|
| 0 | IMU数据 | 1 |
| 1 | 点云数据1（笛卡尔32位） | 96 |
| 2 | 点云数据2（笛卡尔16位） | 96 |
| 3 | 点云数据3（球坐标） | 96 |

### 3.2 UDP IMU数据包格式（端口56400，data_type=0）

IMU数据包使用相同的36字节UDP Payload头部，但数据段为24字节IMU数据：

| 字段名 | 偏移量 | 长度 | 类型 | 说明 |
|--------|--------|------|------|------|
| gyro_x | 0 | 4 | float | 陀螺仪X轴（单位：rad/s） |
| gyro_y | 4 | 4 | float | 陀螺仪Y轴（单位：rad/s） |
| gyro_z | 8 | 4 | float | 陀螺仪Z轴（单位：rad/s） |
| acc_x | 12 | 4 | float | 加速度计X轴（单位：g） |
| acc_y | 16 | 4 | float | 加速度计Y轴（单位：g） |
| acc_z | 20 | 4 | float | 加速度计Z轴（单位：g） |

---

## 4. LVX2到UDP数据流的解码方法

### 4.1 解码流程图
```
┌─────────────┐
│  读取LVX2   │
│  文件头部   │
└──────┬──────┘
       │
       ↓
┌──────────────┐
│ 解析设备信息 │
│  提取lidar_id│
└──────┬───────┘
       │
       ↓
┌──────────────┐
│  遍历所有帧  │
└──────┬───────┘
       │
       ↓
┌──────────────────────┐
│ 对每个数据包：       │
│ 1. 读取LVX2包头(27B) │
│ 2. 读取点云/IMU数据  │
└──────┬───────────────┘
       │
       ↓
┌──────────────────────────┐
│ 重建UDP Payload：        │
│ 1. 构建36字节UDP头       │
│ 2. 附加数据段            │
│ 3. 计算CRC32校验和       │
└──────┬───────────────────┘
       │
       ↓
┌──────────────────────┐
│ 输出UDP协议数据      │
│ - 点云送端口56300    │
│ - IMU送端口56400     │
└──────────────────────┘
```

### 4.2 详细解码步骤

#### 步骤1: 读取并验证文件头

```
1. 读取24字节公共头
   - 验证signature是否为"livox_tech"
   - 验证magic_code是否为0xAC0EA767
   - 验证版本号ver_a是否为2

2. 读取5字节私有头
   - 记录duration（50ms）
   - 记录device_count

3. 读取63字节设备信息
   - 提取lidar_sn、lidar_id等信息
   - 这些信息在重建UDP包时会用到
```

#### 步骤2: 解析帧结构

```
当前位置 = 92字节（24+5+63）

循环处理每一帧：
  1. 读取24字节帧头
     - 读取current_offset（当前帧位置）
     - 读取next_offset（下一帧位置）
     - 读取frame_index（帧索引）
  
  2. 计算本帧数据大小
     frame_data_size = next_offset - current_offset - 24
  
  3. 解析本帧内的所有数据包
     当前包位置 = current_offset + 24
     
     while 当前包位置 < next_offset:
       处理单个数据包（见步骤3）
       当前包位置 += (27 + 数据段长度)
  
  4. 移动到下一帧
     如果next_offset == current_offset: 结束
     否则：当前位置 = next_offset
```

#### 步骤3: 解码单个数据包为UDP Payload

```
对于每个LVX2数据包：

1. 读取27字节LVX2包头
   提取：
   - version (offset 0, 1字节)
   - lidar_id (offset 1, 4字节)
   - timestamp_type (offset 6, 1字节)
   - timestamp (offset 7, 8字节)
   - udp_count (offset 15, 2字节)
   - data_type (offset 17, 1字节)
   - length (offset 18, 4字节)
   - frame_count (offset 22, 1字节)

2. 读取点云/IMU数据
   data = 读取length字节

3. 构建UDP Payload（36字节头 + 数据）
   
   a) 计算相关字段：
      - dot_num: 根据data_type计算
        * data_type=1: length / 14
        * data_type=2: length / 8
        * data_type=3: length / 10
        * data_type=0: 1 (IMU)
      
      - time_interval: 计算帧内时间间隔
        * 从时间戳推算或设为固定值
      
      - length字段: 36 + 数据长度
   
   b) 按UDP协议格式填充36字节头部：
      offset 0:  version (来自LVX2 offset 0)
      offset 1:  length (计算得出)
      offset 3:  time_interval (计算或估算)
      offset 5:  dot_num (计算得出)
      offset 7:  udp_count (来自LVX2 offset 15)
      offset 9:  frame_count (来自LVX2 offset 22)
      offset 10: data_type (来自LVX2 offset 17)
      offset 11: timestamp_type (来自LVX2 offset 6)
      offset 12: reserved (12字节，全0)
      offset 24: crc32 (暂时填0，稍后计算)
      offset 28: timestamp (来自LVX2 offset 7)
   
   c) 附加数据段
      offset 36: 点云/IMU数据 (length字节)
   
   d) 计算并填充CRC32校验和
      crc32_data = timestamp(8字节) + 数据段(length字节)
      使用CRC-32算法计算校验和
      填充到offset 24位置
```

#### 步骤4: 分类输出数据

```
根据data_type将UDP数据包分类：

IF data_type == 0:  # IMU数据
   输出到IMU数据流（对应端口56400）
   
ELSE IF data_type in [1, 2, 3]:  # 点云数据
   输出到点云数据流（对应端口56300）
```

### 4.3 关键字段映射表

| UDP Payload字段 | 偏移 | LVX2来源 | 计算方法 |
|----------------|------|----------|----------|
| version | 0 | LVX2包头offset 0 | 直接复制 |
| length | 1 | - | 计算：36 + 数据长度 |
| time_interval | 3 | - | 从时间戳计算或固定值 |
| dot_num | 5 | - | length / 点大小 |
| udp_cnt | 7 | LVX2包头offset 15 | 直接复制 |
| frame_cnt | 9 | LVX2包头offset 22 | 直接复制 |
| data_type | 10 | LVX2包头offset 17 | 直接复制 |
| time_type | 11 | LVX2包头offset 6 | 直接复制 |
| timestamp | 28 | LVX2包头offset 7 | 直接复制 |
| 数据段 | 36 | LVX2包头后的数据 | 直接复制 |

### 4.4 CRC32计算细节

根据Livox官方协议，CRC32使用以下参数：

- **算法**: CRC-32
- **多项式**: 0x04C11DB7
- **初始值**: 0xFFFFFFFF
- **结果异或值**: 0xFFFFFFFF
- **输入反转**: true
- **输出反转**: true

**计算范围**: 时间戳(8字节) + 数据段(N字节)

**伪代码**:
```
crc_input = timestamp_bytes + data_bytes
crc32_value = calculate_crc32(crc_input)
udp_payload[24:28] = crc32_value (little-endian)
```

---

## 5. 特殊情况处理

### 5.1 时间戳同步问题

**问题**: time_interval字段在LVX2中未直接存储

**解决方案**:
1. **方法A**: 使用固定值
   - Mid-360通常使用固定时间间隔
   - 可设置为合理的默认值（例如：5000 = 0.5ms）

2. **方法B**: 从连续包的时间戳计算
   - 当前包时间戳 - 上一包时间戳 = 实际时间间隔
   - 转换为0.1μs单位

3. **方法C**: 从帧持续时间推算
   - 帧持续时间50ms / 包数量 = 平均时间间隔

### 5.2 帧边界识别

**LVX2中的帧边界标识**:
- 通过frame_index递增识别新帧
- 通过next_offset跳转到下一帧
- 最后一帧的next_offset == current_offset

**UDP数据流的帧分割**:
- udp_cnt从0开始，每帧重置
- frame_cnt每帧递增1
- 50ms时间窗口内的所有包属于同一帧

### 5.3 多设备处理

如果LVX2文件包含多个设备（device_count > 1）：

1. 每个设备有独立的63字节设备信息块
2. 数据包中的lidar_id用于区分不同设备
3. 解码时需要维护每个设备的独立数据流

---

## 6. 数据验证方法

### 6.1 文件完整性验证

```
1. 公共头验证
   - signature == "livox_tech"
   - magic_code == 0xAC0EA767
   - ver_a == 2

2. 帧链式验证
   - 每帧的next_offset应该指向有效位置
   - 最后一帧的next_offset应等于current_offset
   - frame_index应该连续递增

3. 数据包验证
   - 每个包的length字段应合理（根据data_type校验）
   - timestamp应该单调递增（允许小范围波动）
```

### 6.2 解码数据验证

```
1. UDP Payload长度验证
   - length字段 == 36 + 实际数据长度

2. CRC32校验
   - 重新计算CRC32并与包中的值比较

3. 点数验证
   - dot_num * 点大小 == 数据段长度

4. 时间戳连续性
   - 相邻包的时间戳差应在合理范围内（0-5ms）

5. data_type一致性
   - 每个包的data_type应该是有效值(0-3)
```

---

## 7. 性能优化建议

### 7.1 内存优化

1. **流式读取**: 不要一次性加载整个文件到内存
2. **缓冲区管理**: 使用固定大小的缓冲区循环处理
3. **预分配**: 预先分配UDP包的内存空间

### 7.2 处理速度优化

1. **批量处理**: 一次处理多个包后再输出
2. **CRC计算优化**: 使用查表法加速CRC计算
3. **并行处理**: 对于多帧数据，可以并行解码

### 7.3 I/O优化

1. **文件映射**: 使用内存映射文件（mmap）提高读取速度
2. **顺序访问**: 利用next_offset顺序读取，避免随机访问
3. **缓存友好**: 按照访问顺序组织数据结构

---

## 8. 代码实现指导

### 8.1 推荐的类结构设计

```
LVX2Decoder类
  ├── 属性
  │   ├── file_handle: 文件句柄
  │   ├── public_header: 公共头信息
  │   ├── private_header: 私有头信息
  │   ├── device_info: 设备信息列表
  │   └── current_position: 当前读取位置
  │
  ├── 方法
  │   ├── open(file_path): 打开文件并读取头部
  │   ├── read_public_header(): 读取公共头
  │   ├── read_private_header(): 读取私有头
  │   ├── read_device_info(): 读取设备信息
  │   ├── read_frame(): 读取一帧数据
  │   ├── read_package(): 读取一个数据包
  │   ├── decode_package_to_udp(): 解码为UDP payload
  │   ├── calculate_crc32(): 计算CRC32校验和
  │   └── close(): 关闭文件

UDPPayload类
  ├── 属性
  │   ├── header: 36字节头部
  │   ├── data: 数据段
  │   └── total_length: 总长度
  │
  └── 方法
      ├── build_from_lvx2_package(): 从LVX2包构建
      ├── calculate_fields(): 计算相关字段
      ├── to_bytes(): 转换为字节流
      └── validate(): 验证数据完整性
```

### 8.2 主要函数伪代码

```python
function decode_lvx2_to_udp(lvx2_file_path, output_handler):
    # 1. 打开文件并读取头部
    decoder = LVX2Decoder()
    decoder.open(lvx2_file_path)
    
    # 2. 验证文件格式
    if not decoder.validate_headers():
        throw "Invalid LVX2 file"
    
    # 3. 获取设备信息
    device_info = decoder.get_device_info()
    
    # 4. 循环处理所有帧
    while not decoder.is_end_of_file():
        frame = decoder.read_frame()
        
        # 5. 处理帧内所有数据包
        for package in frame.packages:
            # 6. 解码为UDP payload
            udp_payload = decoder.decode_package_to_udp(package, device_info)
            
            # 7. 根据数据类型输出
            if udp_payload.data_type == 0:  # IMU
                output_handler.send_imu_data(udp_payload)
            else:  # Point Cloud
                output_handler.send_pointcloud_data(udp_payload)
    
    # 8. 关闭文件
    decoder.close()
    
    return success
```

### 8.3 关键算法实现提示

#### CRC32计算（Python示例）

```python
import struct
import binascii

def calculate_crc32_livox(data: bytes) -> int:
    """
    计算Livox协议的CRC32校验和
    参数: data - 时间戳(8字节) + 数据段(N字节)
    返回: CRC32值(uint32)
    """
    crc = binascii.crc32(data) & 0xFFFFFFFF
    return crc

def build_udp_payload(lvx2_package, point_data):
    """
    从LVX2数据包构建UDP Payload
    """
    # 构建36字节头部
    header = bytearray(36)
    
    # 填充字段
    struct.pack_into('<B', header, 0, lvx2_package.version)
    struct.pack_into('<H', header, 1, 36 + len(point_data))
    struct.pack_into('<H', header, 3, calculate_time_interval(...))
    struct.pack_into('<H', header, 5, len(point_data) // point_size)
    struct.pack_into('<H', header, 7, lvx2_package.udp_count)
    struct.pack_into('<B', header, 9, lvx2_package.frame_count)
    struct.pack_into('<B', header, 10, lvx2_package.data_type)
    struct.pack_into('<B', header, 11, lvx2_package.timestamp_type)
    # reserved 12-23 保持为0
    struct.pack_into('<Q', header, 28, lvx2_package.timestamp)
    
    # 计算CRC32
    crc_data = header[28:36] + point_data  # timestamp + data
    crc32 = calculate_crc32_livox(crc_data)
    struct.pack_into('<I', header, 24, crc32)
    
    # 组合完整的UDP payload
    udp_payload = bytes(header) + point_data
    
    return udp_payload
```

---

## 9. 测试与验证

### 9.1 单元测试建议

1. **文件头解析测试**
   - 测试正常LVX2文件的头部解析
   - 测试损坏文件的错误检测

2. **数据包解码测试**
   - 对比原始UDP包和解码后的UDP包
   - 验证CRC32计算的正确性

3. **边界条件测试**
   - 空文件处理
   - 单帧文件处理
   - 超大文件处理

### 9.2 集成测试方法

```
测试流程：
1. 使用pcap_to_lvx2.py将PCAP转换为LVX2
2. 使用解码器将LVX2转换回UDP数据流
3. 对比原始PCAP中的UDP payload与解码后的payload
4. 验证关键字段的一致性：
   - 时间戳
   - 点云数据
   - IMU数据
   - CRC32校验和
```

### 9.3 性能基准测试

建议测试指标：
- **解码速度**: 处理100MB LVX2文件的时间
- **内存占用**: 峰值内存使用量
- **准确率**: 与原始数据的一致性百分比

---

## 10. 常见问题与解决方案

### Q1: time_interval字段如何准确恢复？
**A**: 可以通过以下方法：
1. 从连续包的时间戳差值计算
2. 使用Mid-360的典型值（约4000-5000，即0.4-0.5ms）
3. 如果用于可视化，可以使用固定合理值

### Q2: CRC32校验失败怎么办？
**A**: 检查以下几点：
1. CRC计算范围是否正确（timestamp + data）
2. 字节序是否正确（小端序）
3. CRC算法参数是否匹配官方定义

### Q3: 解码后的点云数据是否需要坐标转换？
**A**: 不需要。LVX2中存储的坐标数据与UDP协议中的数据格式相同，都是相对于雷达坐标系的原始坐标。

### Q4: 如何处理timestamp_type不同的情况？
**A**: timestamp_type指示时间同步方式：
- 0: 无同步（设备开机时间）
- 1: PTP同步
- 2: GPS同步
解码时保持此字段不变，时间戳的语义保持原样。

### Q5: 多设备LVX2文件如何解码？
**A**: 
1. 依次读取所有设备信息块
2. 根据数据包中的lidar_id字段区分不同设备
3. 为每个设备维护独立的输出数据流

---

## 11. 参考资料

1. **Livox SDK2 官方仓库**
   - GitHub: https://github.com/Livox-SDK/Livox-SDK2
   - 包含完整的SDK源码和示例

2. **Livox Mid-360 通信协议文档**
   - 英文: https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html
   - 中文: https://livox-wiki-cn.readthedocs.io/zh_CN/latest/tutorials/new_product/mid360/mid360.html

3. **pcap_to_lvx2工具**
   - 本项目中的参考实现：[pcap_to_lvx2/pcap_to_lvx2.py](https://github.com/FelixCooper1026/pcap_to_lvx2)
   - 提供了LVX2文件创建的完整示例

4. **CRC算法参考**
   - CRC-32标准: ISO 3309
   - 在线CRC计算器用于验证

---

## 12. 附录

### 附录A: 数据类型快速参考表

| Type | 名称 | 每点字节数 | 每包点数 | 坐标系 |
|------|------|-----------|----------|--------|
| 0 | IMU数据 | 24 | 1 | N/A |
| 1 | 点云32位笛卡尔 | 14 | 96 | 笛卡尔 |
| 2 | 点云16位笛卡尔 | 8 | 96 | 笛卡尔 |
| 3 | 点云球坐标 | 10 | 96 | 球坐标 |

### 附录B: 时间戳类型说明

| time_type | 说明 | 时间基准 |
|-----------|------|----------|
| 0 | 无同步 | 设备开机时间 |
| 1 | PTP同步 | IEEE 1588v2.0时间 |
| 2 | GPS同步 | GPS标准时间 |

### 附录C: Tag信息位定义

| Bit位 | 说明 |
|-------|------|
| 0-1 | 邻近物体粘连点云（0=高置信度，1=中置信度，2=低置信度） |
| 2-3 | 雨雾尘等微小颗粒（0=高置信度，1=中置信度，2=低置信度） |
| 4-5 | 其他属性（0=高置信度，1=中置信度，2=低置信度） |
| 6-7 | 保留 |

### 附录D: 字节序说明

**Livox协议统一使用小端序（Little-Endian）**

示例：uint32值0x12345678在内存中的存储顺序为：
```
地址:   0x00  0x01  0x02  0x03
数据:   0x78  0x56  0x34  0x12
```

---

## 文档修订历史

| 版本 | 日期 | 修订内容 | 作者 |
|------|------|----------|------|
| v1.0 | 2026-02-11 | 初始版本，完整的LVX2解码方法文档 | Claude Sonnet 4.5 |

---

**文档结束**
