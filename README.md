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

- 6 ROS 2 Python packages trong `src/`: `omni_bringup`, `omni_control`,
  `omni_description`, `omni_hardware`, `omni_safety`, `omni_simulation`.
- ROS 2 overlay có thể build bằng `colcon` trên WSL2 Ubuntu 24.04 amd64.
- Unit/invariant tests cho kinematics, watchdog policy, mô tả Xacro, world và
  ROS/micro-ROS topic contract.
- Renode đã hỗ trợ các platform STM32 và có script mẫu cài tại `/opt/renode`.
- Hai PlatformIO project mẫu cho STM32F407VG Discovery:
  - `firmware/stm32_f407vg_arduino_sim`
  - `firmware/stm32_f407vg_stm3cube_sim`
- Bộ script dùng tham số, không cần tạo thêm một file script cho mỗi board:
  `setup.sh`, `build.sh`, `run_sim.sh`, `run_robot.sh`, `flash_mcu.sh`.

### Chưa triển khai / giới hạn quan trọng

- PlatformIO project `stm32_f407vg_arduino_sim` chứa firmware FreeRTOS +
  micro-ROS; `stm32_f407vg_stm3cube_sim` vẫn là configuration scaffold.
- `firmware/stm32f1/README.md` và `firmware/stm32f4/README.md` chỉ là placeholder,
  không phải project build được.
- Chưa có ROS 1 bringup package trong repository và chưa xác minh trên Jetson
  Nano thật. `run_robot.sh` yêu cầu workspace/package/launch ROS 1 bên ngoài.
- Renode mô phỏng SoC/peripheral STM32 độc lập. Nó chưa tự động nối với ROS 2
  Gazebo hoặc giả lập serial STM32; đường Gazebo dùng `stm32_simulator` để mô
  phỏng cùng thuật toán và topic, còn HIL/USB thật vẫn cần `micro_ros_agent`.
- Gazebo dùng `stm32_simulator` để mô phỏng đúng ranh giới STM32: node nhận
  `stm32_cmd_vel`, tính động học ngược, chạy bốn PID tốc độ và phát lệnh
  actuator tới Gazebo; sensor joint/IMU đi ngược qua động học thuận và Kalman.
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
```

Quy ước exit code:

- `0`: workflow/kiểm tra thành công;
- `1`: validation hoặc tool/workflow thất bại;
- `2`: usage/syntax hoặc điều kiện bắt buộc không hợp lệ (chủ yếu `setup.sh`).

Script không tự build khi run, không tự flash, không tự enable actuator và
không chạy bằng root.

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

### 5.4. Lưu artifact sang thư mục khác

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
for script in scripts/setup.sh scripts/build.sh scripts/run_sim.sh \
             scripts/run_robot.sh scripts/flash_mcu.sh; do
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

- `colcon build` ROS 2: 6 packages finished.
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
`safety_stop`, `odom`, `imu/data_raw`, `wheel_state`, `diagnostics` và `status`.
Watchdog và STM32 đều phát lệnh zero khi mất command quá `0.25 s`. Với phần
cứng, `micro_ros_agent serial` là cầu nối XRCE-DDS duy nhất; firmware chỉ nhận
`stm32_cmd_vel`/`estop`, tính inverse kinematics + bốn PID tốc độ, và phát
telemetry.

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
