# AMR Omni

Phần mềm cho xe tự hành omni: mô phỏng trên WSL2 Ubuntu 24.04 và định hướng
triển khai trên Jetson Nano. Repository hiện có baseline ROS 2 Jazzy/Gazebo
Harmonic, mô hình robot 4 bánh omni, micro-ROS serial transport cho STM32 và
PlatformIO firmware.

> **Nguồn luật phát triển:** [`RULE.md`](RULE.md). Repository cũng giữ file
> `AGENTS.md`/`AGENTS.MD` theo trạng thái Git hiện tại; khi làm việc với source,
> đọc `RULE.md` và file quy ước đang được project sử dụng trước.

## 1. Trạng thái hiện tại

### Đã có

- 9 ROS 2 Python packages trong `src/`: `omni_bringup`, `omni_control`,
  `omni_description`, `omni_hardware`, `omni_localization`, `omni_navigation`,
  `omni_perception`, `omni_safety`, `omni_simulation`.
- 1 ROS 1 bringup package trong `src/omni_bringup_ros1` (được gắn cờ `COLCON_IGNORE`
  để tránh xung đột khi build ROS 2 Jazzy; phục vụ môi trường Catkin trên Jetson Nano Melodic).
- ROS 2 overlay có thể build bằng `colcon` trên WSL2 Ubuntu 24.04 amd64.
- Unit/invariant tests cho kinematics, watchdog policy, mô tả Xacro, world và
  ROS/micro-ROS topic contract.
- Firmware STM32F407VG Discovery hoàn chỉnh (`firmware/stm32_f407vg_arduino_sim`) chạy
  FreeRTOS + micro-ROS với đầy đủ cơ chế an toàn: khóa trường dữ liệu độc lập (chống race condition),
  bảo vệ NaN cho bộ lọc Kalman, chống derivative kick cho bộ điều khiển PID 4 bánh xe,
  timeout phát hiện lỗi IMU 500ms, bitmask an toàn khi hủy micro-ROS entities, và publisher `/debug/data`.
- Node mô phỏng `stm32_simulator` tích hợp bộ điều khiển PID kín (SimplePID), phát dữ liệu
  telemetry `/debug/data` JSON và chuẩn hóa hằng số ma trận hiệp phương sai.
- Giao diện Web Dashboard & Debug Monitor chuyên nghiệp (FastAPI + Next.js + Recharts)
  theo phong cách phẳng (flat/industrial style, bo góc tối thiểu `rounded-sm`), hỗ trợ
  3 tab: Control Cockpit, System Config, Debug Monitor (đồ thị vận tốc 4 bánh xe trước/sau Kalman,
  Quaternion IMU, vận tốc góc, vận tốc dài đặt vs thực tế, và ma trận luồng dữ liệu).
- Bộ 12 script Bash tham số hóa hoàn chỉnh: `setup.sh`, `build.sh`, `run_sim.sh`,
  `run_robot.sh`, `flash_mcu.sh`, `run_web.sh`, `run_bridge.sh`, `calibrate.sh`,
  `diagnostics.sh`, `bag.sh`, `save_map.sh`, `install.sh`.
- Renode đã hỗ trợ các platform STM32 và có script mẫu cài tại `/opt/renode`.

### Giới hạn quan trọng và lưu ý

- PlatformIO project `stm32_f407vg_arduino_sim` chứa source firmware hoàn chỉnh;
  `stm32_f407vg_stm3cube_sim` là configuration scaffold.
- `firmware/stm32f1/README.md` và `firmware/stm32f4/README.md` chỉ là placeholder,
  không phải project build được.
- Package `omni_bringup_ros1` dùng cho Jetson Nano (ROS 1 Melodic); việc khởi động
  robot thực tế qua `run_robot.sh` yêu cầu môi trường aarch64 và workspace Catkin tương ứng.
- Renode mô phỏng SoC/peripheral STM32 độc lập. Đường Gazebo dùng `stm32_simulator` để mô
  phỏng cùng thuật toán và topic, còn HIL/USB thật kết nối qua `micro_ros_agent serial`.
- Gazebo Harmonic dùng `gpu_lidar` cho `/scan` ở cả visual và headless profile;
  headless vẫn cần EGL/DRM render backend và quyền truy cập `/dev/dri`. Host
  không có backend render khả dụng không thể xác minh `/scan` thật, và project
  không dùng scan tổng hợp để che lỗi môi trường.

## 2. Kiến trúc và môi trường

```text
shared Python core: kinematics / safety / ROS topic contract / test vectors
          ├── ROS 2 adapter ── WSL2 Ubuntu 24.04 amd64 ── Jazzy + Harmonic
          └── ROS 1 adapter ── Jetson Nano arm64 ───────── Melodic + native

STM32 firmware (PlatformIO) ── micro_ros_agent serial ── ROS 2 DDS
                         └── Renode peripheral simulation hoặc programmer thật
```

| Chức năng | Host chuẩn | Stack | Tình trạng |
|---|---|---|---|
| ROS 2/Gazebo simulation | WSL2 Ubuntu 24.04, x86_64 | ROS 2 Jazzy, Gazebo Harmonic | Baseline |
| STM32 simulation | WSL2/Linux x86_64 | Renode 1.16.1+, ELF | Có model; peripheral behavior còn giới hạn |
| Robot thật | Jetson Nano nguyên bản, aarch64 | Ubuntu 18.04, JetPack 4.x, ROS 1 Melodic | Chưa xác minh |
| Flash STM32 | Linux có programmer | PlatformIO/OpenOCD/ST-Link/DFU | Chỉ chạy với target cụ thể |

Không source ROS 1 và ROS 2 trong cùng một shell. Kiểm tra trước khi chạy:

```bash
echo "ROS_VERSION=${ROS_VERSION:-unset} ROS_DISTRO=${ROS_DISTRO:-unset}"
```

Build trong filesystem Linux của WSL (`/home/...`), không build dưới `/mnt/c`
hoặc `/mnt/d`. Không chạy ROS, Gazebo hoặc colcon bằng `sudo`.

## 3. Cài dependency và kiểm tra host

### 3.1. Kiểm tra read-only

Từ root repository:

```bash
cd /home/sonev/amr_omni
./scripts/setup.sh --check
```

Script kiểm tra:

- `/opt/ros/jazzy/setup.bash`, `ROS_VERSION=2`, `ROS_DISTRO=jazzy`;
- `colcon`, `ros2`, `xacro`, Gazebo `gz` nếu có;
- `renode` nếu đã cài;
- PlatformIO project khi truyền `--firmware`.

`setup.sh` không sửa `.bashrc`, `PATH`, ROS setup, udev hay apt source.

Kiểm tra cả hai scaffold PlatformIO:

```bash
./scripts/setup.sh --check \
  --firmware stm32_f407vg_arduino_sim \
  --firmware stm32_f407vg_stm3cube_sim
```

Dùng `--strict` để biến warning về Renode/Gazebo/source firmware thành failure:

```bash
./scripts/setup.sh --check --strict \
  --firmware stm32_f407vg_arduino_sim
```

### 3.2. Cài tool tùy chọn

Cài programmer tools bằng apt (cần quyền sudo và xác nhận):

```bash
./scripts/setup.sh --install-flash-tools --yes
```

Các gói là `openocd`, `stlink-tools`, `dfu-util`. Không dùng `sudo pip`.

Cài PlatformIO cho user hiện tại:

```bash
./scripts/setup.sh --install-platformio
```

Sau khi cài user-local, mở shell mới hoặc thêm `~/.local/bin` vào `PATH` theo
cấu hình cá nhân. Script không tự sửa `.bashrc`.

### 3.3. Renode

Kiểm tra Renode:

```bash
renode --version
```

Renode được cài bằng gói Debian stable trên host này và executable là
`/usr/bin/renode`. Có thể kiểm tra các file STM32:

```bash
find /opt/renode/scripts /opt/renode/platforms \
  -iname '*stm32*' -o -iname '*nucleo*' | sort
```

Các shortcut Renode được `run_sim.sh` hỗ trợ:

- `stm32f4_discovery` → `/opt/renode/scripts/single-node/stm32f4_discovery.resc`
- `stm32f103` → `/opt/renode/scripts/single-node/stm32f103.resc`
- `stm32f746` → `/opt/renode/scripts/single-node/stm32f746.resc`
- `stm32l072` → `/opt/renode/scripts/single-node/stm32l072.resc`

## 4. Bộ script và quy ước chung

Tất cả script phải được gọi từ bất kỳ current working directory nào trong git
workspace; script tự tìm project root bằng `git rev-parse`. Để xem toàn bộ tùy
chọn:

```bash
./scripts/setup.sh --help
./scripts/build.sh --help
./scripts/run_sim.sh --help
./scripts/run_robot.sh --help
./scripts/flash_mcu.sh --help
./scripts/run_web.sh --help
./scripts/run_bridge.sh --help
./scripts/calibrate.sh --help
./scripts/diagnostics.sh --help
./scripts/bag.sh --help
./scripts/save_map.sh --help
./scripts/install.sh --help
```

| Script | Vai trò chính | Lệnh thông dụng |
|---|---|---|
| [`setup.sh`](scripts/setup.sh) | Kiểm tra host, dependency, cài đặt PlatformIO và flash tools | `./scripts/setup.sh --check` |
| [`build.sh`](scripts/build.sh) | Build & test ROS 2 packages, PlatformIO firmware, Web bundle | `./scripts/build.sh --component all --test` |
| [`run_sim.sh`](scripts/run_sim.sh) | Chạy mô phỏng Gazebo Harmonic, Renode STM32 hoặc cả hai | `./scripts/run_sim.sh --headless` |
| [`run_robot.sh`](scripts/run_robot.sh) | Chạy hardware bringup trên Jetson Nano (ROS 1 Melodic aarch64) | `./scripts/run_robot.sh --workspace /opt/ros1_ws --package amr_bringup --launch robot.launch` |
| [`flash_mcu.sh`](scripts/flash_mcu.sh) | Nạp firmware an toàn cho STM32 qua PlatformIO/OpenOCD/ST-Link/DFU | `./scripts/flash_mcu.sh --mcu stm32f4 --device /dev/amr_mcu --firmware app.elf --yes` |
| [`run_web.sh`](scripts/run_web.sh) | Khởi động Web Dashboard (FastAPI backend + Next.js frontend) | `./scripts/run_web.sh` |
| [`run_bridge.sh`](scripts/run_bridge.sh) | Cầu nối giao tiếp hai chiều ROS 1 (Jetson) <-> ROS 2 (Host) | `./scripts/run_bridge.sh` |
| [`calibrate.sh`](scripts/calibrate.sh) | Hiệu chuẩn bán kính bánh xe, IMU gyro bias và quét LiDAR | `./scripts/calibrate.sh odometry` |
| [`diagnostics.sh`](scripts/diagnostics.sh) | Chẩn đoán toàn diện sức khỏe Jetson, cổng USB, pin LiPo và an toàn | `./scripts/diagnostics.sh all` |
| [`bag.sh`](scripts/bag.sh) | Ghi, phát lại và kiểm tra metadata rosbag các topic cảm biến | `./scripts/bag.sh record test01` |
| [`save_map.sh`](scripts/save_map.sh) | Lưu bản đồ OccupancyGrid Nav2 thành file YAML + PGM | `./scripts/save_map.sh my_map` |
| [`install.sh`](scripts/install.sh) | Cài đặt udev rules nhận diện USB cố định và systemd autostart | `./scripts/install.sh all` |

Quy ước exit code:

- `0`: workflow/kiểm tra thành công;
- `1`: validation hoặc tool/workflow thất bại;
- `2`: usage/syntax hoặc điều kiện bắt buộc không hợp lệ (chủ yếu `setup.sh`).

Script không tự build khi run, không tự flash, không tự enable actuator và
không chạy bằng root. Tất cả script đều hỗ trợ cờ `-h` / `--help` để xem đầy đủ hướng dẫn sử dụng.

## 5. Build ROS 2

### 5.1. Build toàn bộ ROS 2

```bash
cd /home/sonev/amr_omni
./scripts/build.sh --component ros2
source install/ros2_jazzy/local_setup.bash
```

Artifact được tách riêng:

```text
build/ros2_jazzy/
install/ros2_jazzy/
log/ros2_jazzy/
```

### 5.2. Build và test toàn bộ ROS 2

```bash
./scripts/build.sh --component ros2 --test
```

Tương đương thủ công:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/ros2_jazzy build \
  --symlink-install \
  --base-paths src \
  --build-base build/ros2_jazzy \
  --install-base install/ros2_jazzy \
  --event-handlers console_direct+

source install/ros2_jazzy/local_setup.bash
colcon --log-base log/ros2_jazzy test \
  --base-paths src \
  --build-base build/ros2_jazzy \
  --install-base install/ros2_jazzy \
  --test-result-base build/ros2_jazzy/test_results \
  --event-handlers console_direct+
colcon test-result \
  --test-result-base build/ros2_jazzy/test_results --verbose
```

### 5.3. Build/test từng package

Ví dụ `omni_control`:

```bash
./scripts/build.sh --component ros2 --package omni_control --test
```

Có thể lặp `--package`:

```bash
./scripts/build.sh --component ros2 \
  --package omni_control \
  --package omni_safety \
  --test
```

Chạy test trên artifact đã build:

```bash
./scripts/build.sh --component ros2 \
  --package omni_control --test-only
```

Xóa artifact ROS rồi build sạch:

```bash
./scripts/build.sh --component ros2 --clean --test
```

`--clean` chỉ xóa ba thư mục ROS chuẩn nằm trong project root. Không dùng
`--clean` với `--test-only`.

### 5.4. Build Web Dashboard (Next.js Bundle + FastAPI Static)

Giao diện Web bao gồm Next.js frontend (TypeScript, Tailwind, Recharts) và FastAPI backend. Lệnh build sẽ biên dịch frontend thành static bundle và đồng bộ vào thư mục `web/backend/static`:

```bash
# Build frontend static bundle vào web/backend/static
./scripts/build.sh --component web

# Build frontend và chạy bộ test API backend FastAPI
./scripts/build.sh --component web --test

# Dọn dẹp các thư mục build của Web (.next, out, static)
./scripts/build.sh --component web --clean
```

### 5.5. Build toàn bộ hệ thống (ROS 2, Firmware & Web)

Để build và kiểm thử tích hợp tất cả các thành phần trong repository:

```bash
# Build và chạy test cho toàn bộ: ROS 2 + Firmware + Web
./scripts/build.sh --component all --test --skip-empty

# Xóa toàn bộ artifact và build lại sạch từ đầu
./scripts/build.sh --component all --clean --test --skip-empty
```

> **Lưu ý về `--skip-empty`:** Dùng cờ này để bỏ qua các scaffold PlatformIO chưa có source (`stm32_f407vg_stm3cube_sim`). Nếu không có `--skip-empty`, `build.sh` sẽ đánh dấu SKIP cho project trống và không báo lỗi.

### 5.6. Lưu artifact sang thư mục khác

Các path tương đối được tính từ repository root. Path tuyệt đối cũng được hỗ
trợ cho build/test thường; chỉ nên dùng path ngoài repository khi không dùng
`--clean` để tránh script xóa nhầm dữ liệu ngoài scope:

```bash
./scripts/build.sh --component ros2 \
  --build-base build/ros2_jazzy_debug \
  --install-base install/ros2_jazzy_debug \
  --log-base log/ros2_jazzy_debug \
  --test-result-base build/ros2_jazzy_debug/test_results \
  --test
```

## 6. Build/test firmware PlatformIO

### 6.1. Liệt kê project

```bash
find firmware -name platformio.ini -printf '%h\n' | sort
```

Project Arduino hiện có source firmware và project STM32Cube vẫn là scaffold.
Lệnh sau build target Arduino:

```bash
./scripts/build.sh --component firmware \
  --firmware stm32_f407vg_arduino_sim
```

Sau khi build source firmware, có thể chọn environment cụ thể:

```bash
./scripts/build.sh --component firmware \
  --firmware stm32_f407vg_arduino_sim \
  --environment disco_f407vg
```

Chọn nhiều project bằng cách lặp tham số:

```bash
./scripts/build.sh --component firmware \
  --firmware stm32_f407vg_arduino_sim \
  --firmware stm32_f407vg_stm3cube_sim \
  --environment disco_f407vg
```

### 6.2. Test firmware

Firmware Arduino có test kinematics/Kalman trong `test/`:

```bash
./scripts/build.sh --component firmware \
  --firmware stm32_f407vg_arduino_sim \
  --environment disco_f407vg --test
```

Nếu chỉ muốn xác minh các project scaffold mà không coi chúng là pass:

```bash
./scripts/build.sh --component firmware --skip-empty
```

`--skip-empty` chỉ báo `SKIP`, không tạo firmware và không trả về pass cho
project trống. Không dùng tùy chọn này trong CI nếu CI yêu cầu mọi firmware
phải có source.

### 6.3. PlatformIO trực tiếp để debug

```bash
pio project config \
  --project-dir firmware/stm32_f407vg_arduino_sim
pio run \
  --project-dir firmware/stm32_f407vg_arduino_sim \
  --environment disco_f407vg -v
pio test \
  --project-dir firmware/stm32_f407vg_arduino_sim \
  --environment disco_f407vg
```

Board đang cấu hình là `disco_f407vg`, MCU STM32F407VGT6, 168 MHz, 128 KB RAM,
1 MB Flash. Chỉ đổi board/environment sau khi cập nhật `platformio.ini` và
kiểm tra linker/script tương ứng.

## 7. Chạy mô phỏng Gazebo

### 7.1. Chạy mặc định

Build trước, rồi:

```bash
./scripts/build.sh --component ros2
./scripts/run_sim.sh
```

Mặc định script chạy:

```text
ros2 launch omni_bringup simulation_bringup.launch.py \
  world:=src/omni_simulation/worlds/amr_lab.sdf \
  use_sim_time:=true headless:=false
```

Script không build implicit. Nếu chưa có `install/ros2_jazzy/local_setup.bash`,
nó dừng và chỉ dẫn chạy `build.sh`.

### 7.2. Headless và giới hạn thời gian

Dùng headless cho WSL không có GUI/GPU hoặc CI:

```bash
./scripts/run_sim.sh --headless
```

Tự dừng sau 30 giây thực:

```bash
./scripts/run_sim.sh --headless --duration 30
```

Chọn world khác:

```bash
./scripts/run_sim.sh \
  --world src/omni_simulation/worlds/amr_lab.sdf \
  --headless --duration 60
```

`--duration` ở đây là **real seconds**, không phải simulation seconds. Kết thúc
do `--duration` được coi là thành công (`0`); lỗi khởi động hoặc lỗi runtime
khác vẫn trả exit code khác `0`.

### 7.3. Truyền launch argument

Hai cách tương đương:

```bash
./scripts/run_sim.sh --headless \
  --ros-arg use_sim_time:=true \
  --ros-arg headless:=true

./scripts/run_sim.sh --headless -- \
  use_sim_time:=true headless:=true
```

Không truyền `enabled:=true` cho STM32 adapter nếu chưa xác minh firmware,
protocol và thiết bị serial. Launch baseline mặc định tắt adapter phần cứng.

## 8. Chạy STM32 bằng Renode

### 8.1. Smoke test board STM32F4 Discovery

Không cần build ROS để chạy Renode:

```bash
./scripts/run_sim.sh --mode renode \
  --board stm32f4_discovery \
  --renode-duration 5s
```

Lệnh này nạp script `/opt/renode/scripts/single-node/stm32f4_discovery.resc`,
cho Renode chạy virtual time 5 giây rồi thoát. Chế độ này không cần stdin và có
thể chạy trong CI/job nền. Có thể xem command mà không chạy:

```bash
./scripts/run_sim.sh --mode renode \
  --board stm32f4_discovery \
  --renode-duration 5s --dry-run
```

Mặc định Renode chạy non-interactive để hoạt động ổn định trong CI, `nohup` và
job redirect stdin. Nếu cần gõ lệnh trực tiếp trong Renode Monitor từ terminal,
dùng `--interactive`:

```bash
./scripts/run_sim.sh --mode renode \
  --board stm32f4_discovery --interactive
```

Không dùng `--interactive` trong job nền không có TTY; nếu không Renode có thể
gặp lỗi `Bad file descriptor` khi đọc stdin.

### 8.2. Board shortcut khác

```bash
./scripts/run_sim.sh --mode renode --board stm32f103 --renode-duration 5s
./scripts/run_sim.sh --mode renode --board stm32f746 --renode-duration 5s
./scripts/run_sim.sh --mode renode --board stm32l072 --renode-duration 5s
```

### 8.3. Script Renode và firmware ELF tùy ý

Dùng script `.resc` có sẵn hoặc script do người dùng viết:

```bash
./scripts/run_sim.sh --mode renode \
  --renode-script ./path/to/board.resc \
  --firmware ./path/to/firmware.elf \
  --renode-duration 10s
```

`--firmware` được truyền cho biến Renode `bin` trước khi include `.resc`; các
script chuẩn của Renode thường dùng `$bin?=...`. ELF phải tồn tại và được build
cho đúng CPU/peripheral map. Không truyền `.hex`/`.bin` qua tùy chọn này nếu
script cần `LoadELF`; hãy chỉnh `.resc` để dùng `LoadBinary` đúng địa chỉ.

Các lệnh Renode thường dùng trong monitor:

```text
mach create
machine LoadPlatformDescription @platforms/boards/stm32f4_discovery-kit.repl
sysbus LoadELF @/absolute/path/firmware.elf
start
emulation RunFor "5s"
q
```

Các file model được cài tại `/opt/renode/platforms/boards` và
`/opt/renode/platforms/cpus`. Peripheral map phải khớp firmware; Renode không
thay thế việc kiểm tra clock, linker address, UART và interrupt vector.

### 8.4. Chạy Gazebo và Renode cùng lúc

Dùng `--duration` để bảo đảm cả hai process được dừng:

```bash
./scripts/run_sim.sh --mode both \
  --headless --duration 30 \
  --board stm32f4_discovery \
  --renode-duration 25s
```

`--mode both` yêu cầu `--duration` trừ khi dùng `--dry-run`, tránh để process
Renode/Gazebo mồ côi sau khi shell bị đóng.

## 9. Flash STM32 an toàn

### 9.1. Quy tắc bắt buộc

`flash_mcu.sh`:

- bắt buộc `--mcu stm32f1|stm32f4`;
- bắt buộc device/programmer và firmware cụ thể;
- bắt buộc `--yes` cho thao tác thật;
- không chạy `sudo`, không tự chọn target mơ hồ;
- xác nhận firmware `.elf`, `.hex` hoặc `.bin` tồn tại;
- hỗ trợ dry-run trước khi flash;
- không coi Renode là programmer phần cứng.

Luôn dùng đường dẫn ổn định như `/dev/serial/by-id/...` thay vì đoán
`/dev/ttyACM0` nếu hệ thống có symlink đó.

### 9.2. PlatformIO upload

Sau khi project có source và upload protocol đúng trong `platformio.ini`:

```bash
./scripts/flash_mcu.sh \
  --mcu stm32f4 \
  --device /dev/serial/by-id/PROGRAMMER_OR_BOARD \
  --firmware firmware.elf \
  --backend pio \
  --project stm32_f407vg_arduino_sim \
  --environment disco_f407vg \
  --dry-run
```

Nếu command hiển thị đúng target, xác nhận thao tác:

```bash
./scripts/flash_mcu.sh \
  --mcu stm32f4 \
  --device /dev/serial/by-id/PROGRAMMER_OR_BOARD \
  --firmware firmware.elf \
  --backend pio \
  --project stm32_f407vg_arduino_sim \
  --environment disco_f407vg \
  --yes
```

Tên `--device` được PlatformIO chuyển thành `--upload-port`; giá trị chính xác
phụ thuộc upload protocol trong project. Kiểm tra `pio project config` trước.

### 9.3. OpenOCD, ST-Link và DFU

Cài tool:

```bash
./scripts/setup.sh --install-flash-tools --yes
```

OpenOCD dùng ST-Link và tự chọn config STM32F1/F4 theo `--mcu`:

```bash
./scripts/flash_mcu.sh --mcu stm32f4 \
  --device default --firmware app.elf \
  --backend openocd --dry-run
./scripts/flash_mcu.sh --mcu stm32f4 \
  --device default --firmware app.elf \
  --backend openocd --yes
```

Với probe có nhiều thiết bị, `--device` là ST-Link serial và được truyền qua
OpenOCD `hla_serial`; dùng `default` để OpenOCD tự tìm probe đầu tiên.

ST-Link CLI:

```bash
./scripts/flash_mcu.sh --mcu stm32f4 \
  --device STLINK_SERIAL --firmware app.bin \
  --backend st-flash --dry-run
```

DFU yêu cầu firmware `.bin` và device/USB selector đúng với `dfu-util`:

```bash
./scripts/flash_mcu.sh --mcu stm32f4 \
  --device 0483:df11 --firmware app.bin \
  --backend dfu-util --dry-run
```

`--reset` chỉ là yêu cầu ghi nhận theo backend; khả năng reset thực tế phụ
thuộc programmer/board. Trước khi `--yes`, kiểm tra MCU, device, boot mode,
nguồn, E-stop, firmware map và khả năng recovery.

## 10. ROS 1 trên Jetson Nano

Chỉ chạy trên Jetson Nano nguyên bản (`aarch64`, Ubuntu 18.04, JetPack 4.x,
ROS Melodic) và workspace ROS 1 đã build. Repository chưa cung cấp bringup
package ROS 1 nên phải truyền đầy đủ thông tin:

```bash
./scripts/run_robot.sh \
  --workspace /path/to/ros1_ws \
  --package <bringup_package> \
  --launch <robot.launch> \
  --dry-run
```

Khi dry-run đã pass và checklist phần cứng hoàn tất:

```bash
./scripts/run_robot.sh \
  --workspace /path/to/ros1_ws \
  --package <bringup_package> \
  --launch <robot.launch> \
  --ros-arg use_sim_time:=false \
  --dry-run
```

Sau khi dry-run pass, bỏ `--dry-run` để chạy launch file đã chỉ định. Lệnh thật
không có cờ `--yes`; script chỉ khởi động launch file sau khi kiểm tra
host/workspace. Không chạy trên WSL x86_64: script từ chối architecture khác
`aarch64`.

Cách dùng thực tế:

```bash
./scripts/run_robot.sh \
  --workspace /path/to/ros1_ws \
  --package <bringup_package> \
  --launch <robot.launch> \
  --ros-arg use_sim_time:=false
```

Trước đó phải kiểm tra E-stop, watchdog cục bộ, command timeout, battery,
nhiệt độ, sensor, serial/CAN/GPIO, quyền thiết bị và safe-stop khi mất mạng.
`run_robot.sh` tự chạy `roscore` mặc định, dọn process khi launch kết thúc;
dùng `--no-roscore` nếu ROS master đã được quản lý bên ngoài.

## 11. Test toàn bộ và tiêu chí pass/fail

### 11.1. Kiểm tra shell và help

```bash
bash -n scripts/*.sh
for script in scripts/*.sh; do
  "$script" --help >/dev/null
done
```

### 11.2. Workflow ROS 2 toàn bộ

```bash
./scripts/setup.sh --check
./scripts/build.sh --component ros2 --test
./scripts/run_sim.sh --mode renode \
  --board stm32f4_discovery --renode-duration 1s
./scripts/run_sim.sh --mode gazebo --headless --duration 10
```

### 11.3. Workflow toàn bộ ROS + firmware

Firmware Arduino có thể build/test; project STM32Cube trống nên dùng lệnh này
để bỏ qua scaffold một cách tường minh:

```bash
./scripts/build.sh --component all --test
```

Lệnh trên sẽ build/test ROS 2 rồi ghi nhận firmware scaffold là skip:

```bash
./scripts/build.sh --component all --test --skip-empty
```

Khi STM32Cube source/test đã được thêm, bỏ `--skip-empty`; khi đó mọi project
được chọn phải build và test pass.

### 11.4. Kết quả baseline đã xác nhận

Trên WSL2 Ubuntu 24.04 amd64 của project:

- `colcon build` ROS 2: 9 packages finished (`omni_bringup`, `omni_control`,
  `omni_description`, `omni_hardware`, `omni_localization`, `omni_navigation`,
  `omni_perception`, `omni_safety`, `omni_simulation`).
- Renode `1.16.1.16973`: chỉ xác nhận được việc nạp platform; firmware
  micro-ROS/peripheral device model cần một smoke test riêng.
- PlatformIO `pio project config`: các project parse được.
- PlatformIO Arduino `disco_f407vg`: build/link firmware micro-ROS thành công;
  STM32Cube project vẫn là scaffold và phải được skip nếu chọn toàn bộ firmware.
- `pytest` chạy bằng Conda Python 3.14 không phải đường kiểm thử ROS đúng của
  Jazzy trên host này; nó fail trước test vì Python 3.14 không thấy module
  `lark` trong dependency Python 3.12 của `/opt/ros/jazzy`. Dùng `colcon test`
  sau khi source `/opt/ros/jazzy/setup.bash`, hoặc một Python interpreter tương
  thích ROS Jazzy; không trộn Conda với ROS system Python.

Tiêu chí không được giả mạo:

- parse config không đồng nghĩa build firmware;
- Renode load platform không đồng nghĩa firmware của project đã chạy đúng;
- ROS build pass không đồng nghĩa Gazebo physics, serial, camera hoặc hardware
  thật đã pass;
- `--skip-empty` là skip, không phải pass.

## 12. Topic và contract chính

Simulation và phần cứng dùng chuỗi:

```text
cmd_vel → safe_cmd_vel → stm32_cmd_vel (Jetson bridge) → STM32/mô hình STM32
```

STM32 tính bốn tốc độ bánh nội bộ theo thứ tự:

```text
front_left, front_right, rear_left, rear_right
```

Các topic/contract liên quan gồm `/scan`, `/imu`, `/camera`, `estop`,
`safety_stop`, `odom`, `imu/data_raw`, `wheel_state`, `diagnostics`, `status` và `debug/data`.
Watchdog và STM32 đều phát lệnh zero khi mất command quá `0.25 s`. Với phần
cứng, `micro_ros_agent serial` là cầu nối XRCE-DDS duy nhất; firmware chỉ nhận
`stm32_cmd_vel`/`estop`, tính inverse kinematics + bốn PID tốc độ, và phát
telemetry.

Topic `debug/data` (`std_msgs/String` định dạng JSON) cung cấp dữ liệu nội bộ phục vụ chẩn đoán:
- `raw_wheel_speed_rad_s`: Tốc độ 4 bánh đo từ encoder trước bộ lọc Kalman;
- `filtered_wheel_speed_rad_s`: Tốc độ 4 bánh sau khi qua bộ lọc Kalman;
- `target_wheel_speed_rad_s`: Tốc độ bánh mục tiêu từ động học ngược;
- `motor_output`: Giá trị lệnh actuator điều khiển từ vòng kín PID;
- `imu_quaternion_xyzw`: Quaternion cảm biến IMU chính xác không lọc;
- `body_vx_mps`, `body_vy_mps`, `body_wz_rad_s`: Vận tốc tịnh tiến và vận tốc góc thân xe thực tế;
- `cmd_vx_mps`, `cmd_vy_mps`, `cmd_wz_rad_s`: Vận tốc tịnh tiến và vận tốc góc đặt.

### 12.1. Giao diện Web Dashboard & Debug Monitoring

Khởi động giao diện điều khiển và giám sát:

```bash
# Chế độ tiêu chuẩn: FastAPI backend phục vụ bundle frontend tĩnh tại cổng 8000
./scripts/run_web.sh

# Chạy với cổng hoặc host tùy chỉnh
./scripts/run_web.sh --port 8080 --host 0.0.0.0

# Chế độ phát triển (Hot-reload Next.js dev server tại cổng 3000)
./scripts/run_web.sh --dev

# Tự động build lại frontend bundle trước khi chạy
./scripts/run_web.sh --build
```

- Mở trình duyệt tại `http://localhost:8000` (hoặc `http://localhost:3000` nếu chạy `--dev`).
- Giao diện thiết kế phẳng (flat/industrial style, bo góc tối thiểu `rounded-sm`, viền sắc cạnh, font mono cho số liệu) gồm 3 tab điều khiển chuyên biệt:
  1. **Control Cockpit**:
     - Bản đồ 2D SLAM và chùm tia LiDAR trực quan hóa Canvas thời gian thực.
     - Camera live feed (RGB và Depth) hỗ trợ từ Gazebo hoặc USB camera Jetson.
     - Thẻ động học: tốc độ dài $V_x, V_y$, vận tốc góc $\omega_z$, góc quay La bàn Yaw.
     - Bộ điều khiển phím WASD toàn cục (phát lệnh `cmd_vel` ổn định ở tần số 12.5 Hz với streaming timer, tự động dừng xe khi thả phím hoặc chuyển tab/blur cửa sổ).
  2. **System Config**:
     - Hiệu chỉnh trực tiếp tham số động học xe: bán kính bánh xe $R$, khoảng cách trục $L, W$, giới hạn tốc độ.
     - Cấu hình an toàn: timeout watchdog lệnh, timeout E-Stop.
     - Cấu hình PID vòng kín vận tốc động cơ: $K_p, K_i, K_d$.
  3. **Debug Monitor**:
     - **Motor Speeds (rad/s)**: 4 đồ thị con Recharts cho 4 động cơ (FL, FR, RL, RR) so sánh đồng thời 3 đường: *Raw Speed* (trước bộ lọc Kalman), *Filtered Speed* (sau bộ lọc Kalman), và *Target Speed* (mục tiêu từ động học ngược).
     - **IMU Quaternion**: Hiển thị chính xác 4 giá trị số thực của Quaternion ($q_x, q_y, q_z, q_w$) với 4 chữ số thập phân, kèm đồ thị biến thiên theo thời gian.
     - **Angular Velocity**: Biểu đồ so sánh vận tốc góc $\omega_z$ lệnh đặt (`cmd_wz`) và vận tốc góc đo được thực tế (`actual_wz`).
     - **Linear Velocity**: Biểu đồ so sánh vận tốc tịnh tiến 2 trục: $V_x$ đặt vs thực tế, và $V_y$ đặt vs thực tế.
     - **Diagnostics & Stream Matrix**: Hiển thị trạng thái chi tiết của 4 bánh xe, thông số cảm biến và ma trận luồng dữ liệu Jetson $\leftrightarrow$ STM32 với tần số gói tin và độ trễ mili-giây.

### 12.2. Hướng dẫn chi tiết các scripts tiện ích bổ trợ

#### 12.2.1. `scripts/calibrate.sh` — Hiệu chuẩn phần cứng robot

Dùng để đo đạc và hiệu chuẩn bán kính bánh xe, bề rộng xe (trackwidth), độ lệch tĩnh cảm biến IMU và góc quét LiDAR:

```bash
# Chạy toàn bộ quy trình hiệu chuẩn theo thứ tự
./scripts/calibrate.sh all

# Hiệu chuẩn Odometry: xe chạy thẳng 1 mét với tốc độ 0.2 m/s để đo sai số thực tế
./scripts/calibrate.sh odometry --dist 1.0 --speed 0.2

# Đo độ trôi Gyroscope tĩnh của IMU trong 10 giây (yêu cầu đặt xe đứng yên trên mặt phẳng)
./scripts/calibrate.sh imu --duration 10

# Kiểm tra tần số quét và góc mở của cảm biến LiDAR
./scripts/calibrate.sh lidar --duration 10

# Kiểm tra chiều quay và phản hồi encoder của 4 động cơ (chế độ giả lập an toàn không phát lực)
./scripts/calibrate.sh motors --dry-run
```

#### 12.2.2. `scripts/diagnostics.sh` — Chẩn đoán toàn diện sức khỏe hệ thống

Kiểm tra tức thời hoặc giám sát liên tục tình trạng phần cứng, kết nối mạng và hệ thống an toàn:

```bash
# Chạy kiểm tra nhanh toàn bộ hệ thống
./scripts/diagnostics.sh

# Kiểm tra tài nguyên Jetson Nano (CPU, RAM, GPU nhiệt độ, Power mode 5W/10W)
./scripts/diagnostics.sh jetson

# Kiểm tra các cổng USB cố định (/dev/amr_mcu, /dev/amr_lidar, /dev/amr_imu)
./scripts/diagnostics.sh hardware

# Kiểm tra kết nối mạng Wi-Fi, độ trễ Ping, và trạng thái ROS Master/Nodes
./scripts/diagnostics.sh network

# Kiểm tra điện áp pin LiPo 3S, dòng xả và ngưỡng ngắt an toàn cut-off
./scripts/diagnostics.sh battery

# Giám sát vòng lặp liên tục hệ thống an toàn (E-Stop, Watchdog, Twist Mux) mỗi 3 giây
./scripts/diagnostics.sh safety --continuous

# Giám sát liên tục toàn bộ hệ thống với chu kỳ kiểm tra 5 giây
./scripts/diagnostics.sh all --continuous --interval 5
```

#### 12.2.3. `scripts/run_bridge.sh` — Cầu nối ROS 1 (Jetson) $\leftrightarrow$ ROS 2 (Host)

Phục vụ truyền nhận dữ liệu giữa Jetson Nano (ROS 1 Melodic) và máy tính trạm WSL2/Ubuntu 24.04 (ROS 2 Jazzy):

```bash
# Khởi động cầu nối với ROS 1 Master mặc định tại localhost
./scripts/run_bridge.sh

# Chỉ định địa chỉ IP cụ thể của Jetson Nano
./scripts/run_bridge.sh --master-uri http://192.168.1.100:11311
```

> **Cơ chế hoạt động:** Script tự động phát hiện và sử dụng gói `ros1_bridge` (Dynamic Bridge) nếu đã được cài đặt, hoặc chuyển sang chế độ WebSocket Rosbridge (cổng 9090) để backend web kết nối trực tiếp.

#### 12.2.4. `scripts/save_map.sh` — Lưu bản đồ SLAM OccupancyGrid

Sau khi điều khiển xe quét hoàn thành bản đồ môi trường bằng Nav2 / SLAM Toolbox:

```bash
# Lưu bản đồ mặc định thành maps/amr_lab_map.yaml và maps/amr_lab_map.pgm
./scripts/save_map.sh

# Đặt tên bản đồ tùy chọn
./scripts/save_map.sh warehouse_zone_a

# Lưu vào thư mục khác với ngưỡng chiếm dụng tùy biến
./scripts/save_map.sh factory_map --dir /home/sonev/amr_omni/maps --occ 0.70 --free 0.20
```

#### 12.2.5. `scripts/bag.sh` — Quản lý dữ liệu Rosbag

Tiện ích ghi và phát lại dữ liệu cảm biến, hỗ trợ cả ROS 2 Jazzy (`ros2 bag`) và ROS 1 Melodic (`rosbag`):

```bash
# Ghi lại các topic cảm biến thiết yếu (/cmd_vel, /odom, /scan, /imu/data_raw, /debug/data, ...)
./scripts/bag.sh record

# Ghi với tên file và tự động dừng sau 30 giây
./scripts/bag.sh record test_trajectory_01 --duration 30

# Ghi lại toàn bộ tất cả topic trong hệ thống
./scripts/bag.sh record full_telemetry --all-topics --duration 60

# Xem thông tin metadata và thống kê tần số của file bag đã ghi
./scripts/bag.sh info bags/test_trajectory_01

# Phát lại file bag với tốc độ 1.5x
./scripts/bag.sh play bags/test_trajectory_01 --rate 1.5

# Phát lại lặp vòng liên tục
./scripts/bag.sh play bags/test_trajectory_01 --loop
```

#### 12.2.6. `scripts/install.sh` — Cài đặt Udev Rules & Systemd Autostart

Thiết lập hệ thống trên robot thực tế trước khi triển khai:

```bash
# Cài đặt udev rules nhận diện cổng USB cố định (/dev/amr_mcu, /dev/amr_lidar, /dev/amr_imu)
./scripts/install.sh udev

# Cài đặt dịch vụ systemd amr_robot.service tự động bật robot khi cấp nguồn
./scripts/install.sh service --user robot

# Chỉ định đường dẫn workspace và launch file cho dịch vụ khởi động
./scripts/install.sh service --ros-workspace /opt/ros1_ws --ros-package amr_bringup --ros-launch robot.launch

# Cài đặt đồng thời cả udev rules và systemd service
./scripts/install.sh all

# Gỡ bỏ toàn bộ cấu hình đã cài đặt khỏi hệ thống
./scripts/install.sh --uninstall
```


## 13. Troubleshooting

### `ROS setup file not found` hoặc sai distro

```bash
env -u ROS_VERSION -u ROS_DISTRO bash --noprofile --norc
source /opt/ros/jazzy/setup.bash
./scripts/setup.sh --check
```

Không source Melodic rồi chạy script ROS 2.

### `colcon` báo option không hợp lệ

`--log-base` là option global của colcon, vì vậy script đặt nó trước subcommand
`build`/`test`. Nếu chạy thủ công, dùng:

```bash
colcon --log-base log/ros2_jazzy build ...
```

không dùng `colcon build ... --log-base ...`.

### Không có GUI Gazebo

Dùng:

```bash
./scripts/run_sim.sh --headless --duration 30
```

Kiểm tra `gz sim --version`, WSLg/`DISPLAY` và driver GPU nếu cần GUI. Không
đặt lại `DISPLAY` tự động trong script.

### PlatformIO báo undefined reference `setup/loop` hoặc `main`

Đây là trạng thái hiện tại của scaffold. Thêm source hợp lệ vào đúng
`firmware/<project>/src/`, rồi chạy lại `build.sh`; không dùng `--skip-empty` để
che lỗi đó.

### Renode không thấy ELF

```bash
file /absolute/path/app.elf
readelf -h /absolute/path/app.elf
./scripts/run_sim.sh --mode renode \
  --renode-script /absolute/path/board.resc \
  --firmware /absolute/path/app.elf \
  --renode-duration 1s
```

Kiểm tra `.resc` dùng `$bin`, `LoadELF`, linker address và UART tương ứng.

### Không có programmer/device

```bash
ls -l /dev/serial/by-id/ 2>/dev/null || true
lsusb 2>/dev/null || true
./scripts/setup.sh --install-flash-tools --yes
```

Không đổi sang `chmod 777`, không chạy toàn bộ bằng root. Xác định đúng target
trước `flash_mcu.sh --yes`.

### Test pytest fail khi đang ở Conda `(base)`

Không cài bừa bằng `sudo pip`. Tách environment ROS system khỏi Conda, source
Jazzy trước và dùng `colcon test`. Nếu cần Python unit test thuần, chạy test
module không import ROS bằng interpreter tương thích đã được project xác minh.

## 14. Quy tắc an toàn và phát triển

- Firmware STM32 là ngoại lệ cho quy tắc Python-first; tool/script/wrapper mới
  vẫn dùng Bash/Python phù hợp convention hiện tại.
- Không dùng `sudo pip`, không chạy ROS/Gazebo bằng root.
- Không tự sửa `.bashrc`, apt source, udev hoặc system configuration.
- Không commit `build/`, `install/`, `log/`, `.pio/`, binary firmware, secret,
  device path cá nhân hoặc machine-local config.
- Flash/reset/xóa artifact phải validate target; `flash_mcu.sh` yêu cầu `--yes`.
- Robot thật bắt buộc E-stop vật lý, watchdog cục bộ, command timeout và safe
  stop khi mất máy tính/mạng.
- Báo cáo test phải nêu host, OS, architecture, ROS distro, commit và phần đã
  hoặc chưa xác minh.

## 15. GitNexus

GitNexus là công cụ đọc quan hệ source, không thay thế test:

```bash
cd /home/sonev/amr_omni
node .gitnexus/run.cjs status
node .gitnexus/run.cjs analyze
```

Không commit `.gitnexus/`. Trước khi sửa symbol code phải xem impact; sau thay
đổi logic nên chạy detect changes theo quy ước trong `AGENTS.md`.

<!-- 
rm -f .git/index
git reset 
-->