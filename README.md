# jetson_car_ws

在 Jetson 上執行的差速輪自走車 ROS 2 工作區：以 ZLAC706‑RC 驅動兩顆輪轂馬達、YDLidar 光達掃描，搭配 slam_toolbox 建圖與 Nav2 自主導航。

- **ROS 版本**：ROS 2 Humble（Ubuntu 22.04，arm64）
- **功能**：鍵盤遙控、輪式里程計、2D 雷射 SLAM 建圖、AMCL 定位與 Nav2 路徑規劃

## 目錄

1. [系統架構](#系統架構)
2. [硬體需求](#硬體需求)
3. [安裝](#安裝)
4. [編譯](#編譯)
5. [使用方式](#使用方式)
6. [參數說明](#參數說明)
7. [固定序列埠名稱](#固定序列埠名稱)
8. [疑難排解](#疑難排解)

## 系統架構

```
 teleop_twist_keyboard  or  Nav2
              |  /cmd_vel
              v
 zlac706_diffdrive_node  <== RS-485 (Modbus RTU) ==>  ZLAC706 x2
              |  /odom , TF odom -> base_link
              v
 slam_toolbox (mapping)  or  AMCL (localization)  <-- /scan --  ydlidar_scan_node  <==  YDLidar
              |  /map , TF map -> odom
              v
 Nav2 (planner / controller / costmap)
```

### 套件一覽

| 套件 | 說明 |
| --- | --- |
| `car_bringup` | 整車啟動：馬達驅動 + 光達 + `base_link → laser_frame` 靜態 TF |
| `zlac706_driver_cpp` | ZLAC706‑RC 差速輪驅動節點（C++），另含 SLAM 啟動檔與參數 |
| `ydlidar_scan_cpp` | YDLidar SDK 包裝節點，發佈 `/scan` |
| `car_navigation2` | Nav2 啟動檔、參數檔與地圖 |
| `zlac706_driver` | 早期的 Python 版驅動，已由 C++ 版取代，僅留作參考（見[備註](#python-版驅動)） |

### 主要介面

| 名稱 | 型別 | 方向 | 來源節點 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 訂閱 | `zlac706_diffdrive_node` |
| `/odom` | `nav_msgs/Odometry` | 發佈（20 Hz） | `zlac706_diffdrive_node` |
| TF `odom → base_link` | — | 發佈（20 Hz） | `zlac706_diffdrive_node` |
| `/scan` | `sensor_msgs/LaserScan` | 發佈（約 10 Hz） | `ydlidar_scan_node` |
| `/start_scan`、`/stop_scan` | `std_srvs/Empty` | 服務 | `ydlidar_scan_node` |
| TF `base_link → laser_frame` | — | 靜態 | `static_transform_publisher` |

## 硬體需求

| 項目 | 規格 |
| --- | --- |
| 主機 | Jetson（arm64），需能執行 Ubuntu 22.04。Orin 系列請安裝 JetPack 6 |
| 馬達驅動器 | ZLAC706‑RC × 2，RS‑485 Modbus RTU，38400 bps、8N1 |
| 驅動器位址 | 左輪 `0x02`、右輪 `0x01`（寫死在原始碼中） |
| USB‑RS485 轉接器 | 一個，兩顆驅動器接在同一條匯流排上 |
| 光達 | YDLidar，USB 序列埠 230400 bps（參數依 G2B 調校） |

> 舊款 Jetson Nano（JetPack 4，Ubuntu 18.04）無法直接用 apt 安裝 Humble，需改用 Docker 或自行從原始碼編譯 ROS 2。

## 安裝

### 1. ROS 2 Humble

依[官方文件](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)安裝 `ros-humble-desktop` 與 `ros-dev-tools`。

### 2. 相依套件

```bash
sudo apt update
sudo apt install build-essential cmake git \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard
```

### 3. YDLidar SDK

SDK 不包含在本工作區內，必須先安裝，否則 `ydlidar_scan_cpp` 無法編譯。

```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git ~/YDLidar-SDK
cd ~/YDLidar-SDK
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

### 4. 序列埠權限

```bash
sudo usermod -aG dialout $USER
```

執行後需**登出再登入**（或重新開機）才會生效。

## 編譯

```bash
git clone https://github.com/willie040600/jetson_car_ws.git ~/jetson_car_ws
cd ~/jetson_car_ws
source /opt/ros/humble/setup.bash

# 第一次使用 rosdep 才需要 init
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys ydlidar_sdk

colcon build --symlink-install
source install/setup.bash
```

之後每開一個新的終端機，都要先載入環境：

```bash
source /opt/ros/humble/setup.bash
source ~/jetson_car_ws/install/setup.bash
```

可以把這兩行加進 `~/.bashrc`。

## 使用方式

以下每個步驟各佔一個終端機。

### 步驟 1：確認序列埠

接上 USB‑RS485 轉接器與光達後，確認兩個裝置各自的埠號：

```bash
ls -l /dev/serial/by-id/
```

`ttyUSB0` / `ttyUSB1` 的編號會隨插拔順序對調。接反時兩個節點都能開啟序列埠，但都收不到回應。建議依[固定序列埠名稱](#固定序列埠名稱)一節設定固定名稱。

### 步驟 2：啟動底盤與光達

> **第一次測試請先把車架高，讓輪子離地。**

```bash
ros2 launch car_bringup car_bringup_launch.py \
  motor_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
```

| 參數 | 預設值 | 說明 |
| --- | --- | --- |
| `motor_port` | `/dev/ttyUSB0` | USB‑RS485 轉接器 |
| `lidar_port` | `/dev/ttyUSB1` | 光達 |

正常啟動時會看到：

```
序列埠已開啟: /dev/ttyUSB0 @ 38400bps
YDLidar started on /dev/ttyUSB1 @ 230400 bps
```

另開終端機檢查資料是否正常：

```bash
ros2 topic hz /scan                     # 約 10 Hz
ros2 topic echo /odom --once            # 有里程計輸出
ros2 run tf2_ros tf2_echo odom base_link
```

### 步驟 3：鍵盤遙控

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

預設速度是 0.5 m/s、1.0 rad/s，對室內小車偏快。開始移動前先按幾次 `z` 降速（每按一次降 10%，終端機會顯示目前數值），建議降到 0.15 m/s 左右；按 `q` 則是加速。

驅動節點超過 0.5 秒沒收到 `/cmd_vel` 就會自動把速度歸零，所以要**按住按鍵**車子才會持續移動。

測試時請確認：

- 按 `i` 前進時兩輪都往前轉。若某一輪反轉，調整 `invert_left` / `invert_right`。
- 按 `j` 時車子逆時針（向左）原地旋轉。若方向相反，代表左右輪位址接反。

### 步驟 4：建圖（SLAM）

保持步驟 2 執行中，另開終端機：

```bash
ros2 launch zlac706_driver_cpp slam_launch.py
```

開啟 RViz 觀察建圖結果：

```bash
rviz2
```

在 RViz 中將 **Fixed Frame** 設為 `map`，並新增 **Map**（topic `/map`）與 **LaserScan**（topic `/scan`）顯示。接著用步驟 3 的鍵盤遙控，慢速開車繞行整個空間。

地圖完成後存檔（不要關閉 SLAM）：

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_room
```

會產生 `my_room.pgm` 與 `my_room.yaml`。

### 步驟 5：導航（Nav2）

先關閉 SLAM（步驟 4），保持步驟 2 執行中，再啟動：

```bash
ros2 launch car_navigation2 navigation2.launch.py map:=$HOME/maps/my_room.yaml
```

| 參數 | 預設值 | 說明 |
| --- | --- | --- |
| `map` | 套件內的 `maps/room.yaml` | 地圖 yaml 的完整路徑 |
| `params_file` | 套件內的 `config/nav2_params.yaml` | Nav2 參數檔的完整路徑 |
| `use_rviz` | `true` | 是否同時開啟 RViz |
| `autostart` | `true` | 自動啟用 Nav2 各節點 |
| `use_sim_time` | `false` | 實體車請維持 `false` |

RViz 開啟後依序操作：

1. 點選 **2D Pose Estimate**，在地圖上標出車子目前的位置與朝向。
2. 確認雷射點與地圖牆面大致重合。
3. 點選 **Nav2 Goal**，在地圖上指定目的地。

> 必須先給初始位姿。否則 AMCL 不會發佈 `map → odom`，costmap 會持續回報 `Invalid frame ID "map"`。

若要把自己的地圖設為預設值，將 `.pgm` 與 `.yaml` 放進 `src/car_navigation2/maps/`、命名為 `room`，再重新執行 `colcon build`。

#### 在另一台電腦上開 RViz

Jetson 與電腦位於同一個區域網路、`ROS_DOMAIN_ID` 相同，且電腦上也安裝了 ROS 2 Humble 與 `ros-humble-nav2-bringup` 時：

```bash
# Jetson
ros2 launch car_navigation2 navigation2.launch.py use_rviz:=false

# 電腦
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### 其他啟動檔

| 指令 | 用途 |
| --- | --- |
| `ros2 launch ydlidar_scan_cpp ydlidar_launch.py` | 只啟動光達與靜態 TF，埠號取自 `params/ydlidar.yaml` |
| `ros2 launch zlac706_driver_cpp car_bringup_launch.py` | 與 `car_bringup` 相同，但預設埠號相反（馬達 `ttyUSB1`、光達 `ttyUSB0`） |
| `ros2 run zlac706_driver_cpp zlac706_diffdrive_node --ros-args -p port:=/dev/ttyUSB0` | 只啟動馬達驅動 |

## 參數說明

### 馬達驅動 `zlac706_diffdrive_node`

| 參數 | 預設值 | 說明 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB1` | RS‑485 序列埠 |
| `baudrate` | `38400` | 支援 9600 / 19200 / 38400 / 57600 / 115200 / 230400 |
| `wheel_radius` | `0.05368` | 輪半徑（公尺） |
| `wheel_separation` | `0.270` | 左右輪中心距（公尺） |
| `max_rpm` | `200.0` | 轉速上限 |
| `invert_left` | `false` | 左輪反向 |
| `invert_right` | `true` | 右輪反向（馬達左右對稱安裝） |
| `cmd_timeout` | `0.5` | 超過此秒數未收到 `/cmd_vel` 即停車 |
| `transaction_period` | `0.05` | 每筆 Modbus 交易的間隔（秒），**不可低於 0.05** |
| `reply_timeout` | `0.05` | 等待驅動器回應的時限（秒） |
| `debug_serial` | `false` | 讀取失敗時印出原始位元組 |

`wheel_radius` 與 `wheel_separation` 直接影響里程計準確度。換車體後請實測校正：直走 1 公尺校正輪半徑，原地轉一圈校正輪距。

目前啟動檔只開放 `port` 參數。要調整其他參數，可在啟動檔的 `parameters` 中加入，或用上表的 `ros2 run` 方式單獨啟動節點。

左右輪的 Modbus 位址是 `src/zlac706_driver_cpp/src/zlac706_diffdrive_node.cpp` 中的常數 `ADDR_L`、`ADDR_R`，修改後需重新編譯。

### 光達 `src/ydlidar_scan_cpp/params/ydlidar.yaml`

| 參數 | 目前值 | 說明 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | 會被 `car_bringup` 的 `lidar_port` 覆蓋 |
| `baudrate` | `230400` | 單通道機種（X2、X3 等）為 115200 |
| `isSingleChannel` | `false` | 單通道機種改為 `true` |
| `lidar_type` | `1` | 0＝TOF、1＝三角測距、2＝TOF_NET |
| `intensity` / `intensity_bit` | `true` / `8` | G2B 需固定為 8，設 0 會造成程式崩潰 |
| `sample_rate` | `9` | 設 5 會造成封包校驗失敗 |
| `ignore_array` | `"-85,85"` | 要濾掉的角度區間（度），依光達安裝方向與車體遮擋調整 |
| `angle_min` / `angle_max` | `-180.0` / `180.0` | 掃描角度範圍（度） |
| `range_min` / `range_max` | `0.12` / `16.0` | 有效距離（公尺） |
| `frequency` | `10.0` | 掃描頻率（Hz） |

換用其他型號的 YDLidar 時，請對照 [YDLidar SDK 文件](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/Dataset.md)修改上述參數。

### 光達安裝位置

`base_link → laser_frame` 的靜態 TF 定義在 `src/car_bringup/launch/car_bringup_launch.py`，目前為光達位於車體中心正上方 0.107 公尺、無旋轉。光達位置或朝向不同時請修改 `arguments`，順序為 `x y z yaw pitch roll`。

### 導航 `src/car_navigation2/config/nav2_params.yaml`

| 項目 | 目前值 |
| --- | --- |
| 區域規劃器 | DWB |
| 最大線速度 / 角速度 | 0.26 m/s / 1.0 rad/s |
| 機器人半徑 | 區域 costmap 0.32 m、全域 costmap 0.22 m |
| 膨脹半徑 | 0.55 m |
| 到點容許誤差 | 位置 0.25 m、角度 0.25 rad |

### 建圖 `src/zlac706_driver_cpp/config/mapper_params_online_async.yaml`

地圖解析度 0.05 m，雷射有效距離 0.12–12 m，每移動 0.2 m 或旋轉 0.2 rad 更新一次，已開啟迴環偵測。

## 固定序列埠名稱

最簡單的做法是直接使用 `/dev/serial/by-id/` 底下的路徑，它不會隨插拔順序改變：

```bash
ls -l /dev/serial/by-id/

ros2 launch car_bringup car_bringup_launch.py \
  motor_port:=/dev/serial/by-id/<RS485 轉接器> \
  lidar_port:=/dev/serial/by-id/<光達>
```

也可以自訂 udev 規則。先查出裝置資訊：

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial' | head
```

再建立 `/etc/udev/rules.d/99-jetson-car.rules`（把 `xxxx`、`yyyy`、`zzzz` 換成查到的值，兩個裝置各寫一行）：

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="yyyy", ATTRS{serial}=="zzzz", SYMLINK+="zlac706", MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

之後即可使用 `motor_port:=/dev/zlac706`。

> YDLidar SDK 附的 `startup/initenv.sh` 會把所有 CP2102 晶片的裝置都指到 `/dev/ydlidar`。如果 USB‑RS485 轉接器也是 CP2102，兩者會衝突，請改用上面的方式。

## 疑難排解

| 現象 | 可能原因與處理 |
| --- | --- |
| 編譯時找不到 `ydlidar_sdk` | 尚未安裝 YDLidar SDK，見[安裝第 3 步](#3-ydlidar-sdk) |
| `無法開啟序列埠 ...: Permission denied` | 使用者不在 `dialout` 群組，或加入後尚未重新登入 |
| `無法獨佔（可能已被其他行程佔用）` | 有另一個節點或程式正在使用同一個序列埠 |
| `讀取轉速回授失敗` | 馬達與光達的埠號接反、RS‑485 A/B 線接反、驅動器未上電，或鮑率不符。可加上 `-p debug_serial:=true` 單獨啟動節點查看原始位元組 |
| `YDLidar initialize failed` | 埠號錯誤，或 `ydlidar.yaml` 的參數與光達型號不符 |
| 找不到 `/dev/ttyUSB*` | 用 `dmesg \| tail` 確認裝置有被辨識。Ubuntu 22.04 上 `brltty` 可能會佔用 USB 轉序列埠晶片，可執行 `sudo apt remove brltty` 後重新插拔 |
| 直走正常但轉向相反 | 左右輪位址接反，交換 `ADDR_L` / `ADDR_R` 後重新編譯 |
| 車子前後顛倒 | 調整 `invert_left` / `invert_right` |
| 位址 `0x01` 的驅動器完全不回應 | `transaction_period` 被設得低於 0.05 |
| Nav2 回報 `Invalid frame ID "map"` | 尚未在 RViz 用 **2D Pose Estimate** 給初始位姿 |
| Nav2 的 TF 查詢全部逾時 | `use_sim_time` 被設為 `true`，實體車必須是 `false` |
| SLAM 地圖歪斜或重影 | 里程計不準，校正 `wheel_radius` 與 `wheel_separation`，並降低行駛速度 |

## 備註

### Python 版驅動

`zlac706_driver` 是早期以 rclpy 寫的驅動，功能與 C++ 版相同，但有幾點差異：

- 左右輪位址與 C++ 版**相反**（左 `0x01`、右 `0x02`）。
- 輪半徑與輪距的預設值不同（0.0508 m、0.23 m）。
- 需要額外安裝 `sudo apt install python3-serial`。

所有啟動檔使用的都是 C++ 版。兩個版本不能同時執行。

### 目錄結構

```
jetson_car_ws/
└── src/
    ├── car_bringup/            # 整車啟動檔
    ├── car_navigation2/        # Nav2 啟動檔、參數、地圖
    ├── ydlidar_scan_cpp/       # 光達節點與參數
    ├── zlac706_driver_cpp/     # 馬達驅動節點、SLAM 啟動檔與參數
    └── zlac706_driver/         # Python 版驅動（舊）
```
