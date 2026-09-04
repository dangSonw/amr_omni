# AGENTS.MD — Luật phát triển AMR Omni

Đây là **nguồn luật duy nhất** cho con người và AI agent trong repository
`amr_omni`. Không tạo `.agents/rules/`, `.agents/skills/`, `.dockerignore` hoặc
tài liệu quy ước trùng lặp nếu chủ dự án không yêu cầu rõ. Các từ **PHẢI**,
**KHÔNG ĐƯỢC**, **NÊN** thể hiện mức bắt buộc, cấm và khuyến nghị mạnh.

## 1. Nền tảng đã chốt

| Vai trò | Hệ thống | Stack |
|---|---|---|
| Phát triển/mô phỏng | WSL2 Ubuntu 24.04, amd64 | ROS 2 Jazzy, Gazebo Harmonic |
| Robot thật | Jetson Nano, JetPack 4.x/L4T 32.x, Ubuntu 18.04, arm64 | ROS 1 Melodic, native |

- Không cài Jazzy trên Ubuntu 18.04; không nâng Jetson Nano gốc lên JetPack 5/6.
- Không chạy Gazebo trên Jetson production. Runtime Jetson ưu tiên native để giữ
  tương thích CUDA, CSI, GPIO, CAN và serial.
- Không thêm container nếu chưa chứng minh lợi ích và benchmark I/O/GPU/RAM.
- Melodic và Ubuntu 18.04 đã EOL: khóa dependency, cô lập Jetson trong mạng robot
  tin cậy và không mở dịch vụ ra Internet công cộng.

### Python-first

- Trong giai đoạn đầu, mọi node, thư viện thuật toán, tool, test và script do dự
  án viết PHẢI dùng Python.
- ROS 2 package dùng `ament_python`; ROS 1 package dùng catkin và cài Python
  script đúng quy ước.
- Chỉ thêm C/C++ sau khi benchmark chứng minh Python không đạt yêu cầu và chủ dự
  án chấp thuận. XML, YAML, Xacro/URDF, launch, map/world và CMake tối thiểu của
  catkin không vi phạm luật này.
- Firmware STM32 là ngoại lệ kỹ thuật; không thể mặc định viết bằng Python.
- Jazzy dùng Python 3 hiện đại; Melodic/18.04 mặc định gắn với Python 2.7. Không
  giả định một ROS node chạy nguyên trạng trên cả hai.
- Logic dùng chung PHẢI là module ROS-agnostic: không import `rclpy`, `rospy`,
  ROS message, Gazebo hoặc API Jetson. Adapter ROS 1/ROS 2 phải mỏng và tách biệt.
- Không đổi symlink `/usr/bin/python` và không ép thay thế Python hệ thống. Shared
  core phải ưu tiên subset Python 2.7/3 tương thích nếu cần chạy ở cả hai host;
  adapter ROS 2 có thể dùng Python 3 riêng, còn adapter ROS 1 phải tuân runtime
  thực tế trên Jetson.
- Shared core không được dùng f-string, `dataclasses`, type annotation bắt buộc,
  `pathlib` hoặc API chỉ có ở Python 3 nếu còn yêu cầu chạy với Python 2.7. Mọi
  giới hạn tương thích phải được kiểm tra bằng test trên đúng interpreter.

## 2. Kiến trúc

- `omni_interfaces`: message/service/action custom; ưu tiên interface ROS chuẩn.
- `omni_description`: Xacro/URDF, mesh, RViz và launch hiển thị.
- `omni_hardware`: adapter base, lidar, IMU, camera và I/O robot thật.
- `omni_control`: kinematics và controller.
- `omni_localization`: odometry, fusion và localization.
- `omni_navigation`: navigation, map và behavior tree.
- `omni_perception`: perception nodes.
- `omni_safety`: watchdog, E-stop state, giới hạn và fail-safe.
- `omni_simulation`: Gazebo world/model/config/launch.
- `omni_bringup`: entry point hệ thống, chỉ wiring; không chứa business logic.
- `firmware/mcu/{stm32f1,stm32f4}`: firmware theo MCU.
- `scripts`: setup/build/run/flash/calibration/diagnostics.

`omni_mulation` là lỗi chính tả; tên chuẩn cho code mới là `omni_simulation`.
Không tạo package mơ hồ như `utils`, `common`, `misc`. Dependency đi theo hướng
adapter/UI → application → shared core; shared core không phụ thuộc ROS/Gazebo.
Simulation và robot thật NÊN dùng cùng frame, topic contract, parameter schema,
kinematic model và test vector.

## 3. Naming

### File và package

- Folder, ROS package, Python package/module/file: ASCII `snake_case`.
- ROS package bắt đầu bằng `omni_` và mô tả đúng miền.
- ROS 2 launch: `<purpose>.launch.py`; ROS 1 launch: `<purpose>.launch`.
- Config: `<node_or_subsystem>.yaml`; cấm tên `config1`, `new`, `final`.
- Xacro chính: `<robot>.urdf.xacro`; component: `<component>.xacro`.
- Test: `test_<subject>.py`; Bash: `snake_case.sh`.
- Không dùng khoảng trắng, dấu tiếng Việt hoặc khác biệt chữ hoa/thường tùy ý.
- Chỉ dùng viết tắt chuẩn: `amr`, `imu`, `lidar`, `urdf`, `rviz`, `mcu`, `ekf`,
  `tf`.

### Python

- Biến/hàm/method: `snake_case`; class/exception: `PascalCase`; constant:
  `UPPER_SNAKE_CASE`; exception kết thúc bằng `Error`.
- Boolean bắt đầu bằng `is_`, `has_`, `can_`, `should_`; collection dùng số nhiều.
- Hàm dùng động từ rõ nghĩa như `compute_wheel_speeds()`, không dùng `process()`
  nếu có thể cụ thể hơn.
- Ghi đơn vị trong tên khi dễ nhầm: `timeout_sec`, `speed_mps`, `yaw_rad`,
  `frequency_hz`.
- Không shadow built-in (`id`, `type`, `list`, `map`, `input`) và không sửa
  `sys.path` để vá import.

### ROS graph/model

- Node/topic/service/action/namespace/parameter/frame: chữ thường `snake_case`.
- Library/node tái sử dụng không hard-code namespace bắt đầu bằng `/`.
- Custom interface dùng `PascalCase`, ví dụ `WheelState.msg`.
- Tuân REP-105: `map`, `odom`, `base_link`; frame không bắt đầu bằng `/`.
- Link/joint nhất quán, ví dụ `front_left_wheel_joint`.
- Thứ tự và dấu bánh xe phải khai báo một lần, có test; không dựa vào thứ tự
  dictionary hoặc message ngẫu nhiên.

## 4. Đường dẫn

- Code/config/script KHÔNG ĐƯỢC hard-code `/home/sonev`, `/mnt/c`, IP cá nhân
  hoặc tên workspace.
- Mọi đường dẫn tương đối phải neo vào Git root, package share hoặc thư mục file
  config; không phụ thuộc current working directory ngầm định.
- Bash xác định root bằng `git rev-parse --show-toplevel`; nếu chạy ngoài Git,
  dùng `${BASH_SOURCE[0]}` và `pwd -P`. Luôn quote biến đường dẫn.
- ROS 2 lookup resource bằng `ament_index_python`; ROS 1 bằng `rospkg`; Python
  package bằng `importlib.resources`. Chỉ dùng `os.path` thay `pathlib` trong
  compatibility code cần runtime cũ.
- Không dùng `..` xuyên package, không tạo symlink ra ngoài repo, không ghi log,
  bag, cache, calibration output hoặc model tải về vào source tree.
- Trên WSL, PHẢI build trong filesystem Linux (`/home/...`), không build dưới
  `/mnt/c` hoặc `/mnt/d` do I/O, permission, symlink và file-watch không ổn định.

## 5. GitNexus-first

Không đọc tuần tự từng file/toàn repository khi graph có thể khoanh vùng code.

1. Từ project root chạy `gitnexus status`.
2. Nếu thiếu/lỗi thời, chạy `gitnexus analyze` và không commit `.gitnexus/`.
3. Dùng trước khi mở file:
   - `gitnexus query "<concept/flow>"` để tìm luồng;
   - `gitnexus context <symbol>` để xem caller/callee/process;
   - `gitnexus impact <symbol>` trước khi đổi API;
   - `gitnexus trace <from> <to>` để tìm dependency path;
   - `gitnexus detect-changes` để đánh giá diff.
4. Sau đó chỉ đọc symbol/file đích; ưu tiên semantic/LSP tool.

Được fallback sang Serena/LSP/search có mục tiêu khi repo mới chưa đủ code,
GitNexus lỗi, hoặc cần kiểm tra YAML/XML/Xacro/URDF/RViz/world/shell/literal cụ
thể. GitNexus không thay thế đọc file đích, lint, test và runtime validation.

## 6. Luật môi trường

### Phát hiện host

- Không suy đoán host chỉ từ Ubuntu version.
- WSL: kiểm tra `WSL_INTEROP` hoặc `microsoft` trong kernel release.
- Jetson: kiểm tra `aarch64`, `/etc/nv_tegra_release` và device-tree model.
- Sau khi source, validate `ROS_DISTRO` là `jazzy` hoặc `melodic` đúng mục đích.
- Script phải fail-fast khi sai host; không tự cài stack của host khác.

### WSL2/Jazzy

- Source `/opt/ros/jazzy/setup.bash` rồi mới source workspace overlay.
- Dùng `colcon`; không dùng catkin trong workspace Jazzy.
- Dùng Gazebo Harmonic `gz sim`/`ros_gz`; không trộn Gazebo Classic tùy tiện.
- Ưu tiên WSLg; không tự override `DISPLAY` khi WSLg đang hoạt động.
- Không chạy ROS/Gazebo/colcon GUI bằng `sudo`.
- Không giả định WSL có systemd hoặc USB. Kiểm tra trước; USB attach bằng
  `usbipd` từ Windows, không “sửa” bằng `chmod 777`.
- Kiểm tra GPU/rendering trước khi bật acceleration; luôn có headless/CPU mode.

### Jetson/Melodic

- Source `/opt/ros/melodic/setup.bash` rồi catkin overlay. Không source Jazzy và
  Melodic trong cùng shell.
- Không chạy Gazebo/RViz/browser nặng trong robot production.
- Không nâng riêng CUDA/cuDNN/TensorRT ngoài phiên bản tương thích JetPack/L4T.
- Kiểm tra device và permission của serial, I2C, video, CAN, GPIO trước launch.
- Sửa permission bằng group/udev rule được review; cấm `chmod 777` và cấm chạy
  toàn stack bằng root.
- Queue/buffer phải hữu hạn. Benchmark RAM, CPU/GPU, nhiệt độ và power mode.
- Trước chuyển động: kiểm tra nguồn, E-stop, watchdog, command timeout, disk,
  nhiệt độ, clock và sensor health.

### Mạng và bridge

- ROS 1 dùng `ROS_MASTER_URI` và `ROS_IP`/`ROS_HOSTNAME` có thể route; không dùng
  localhost giữa hai máy. Không hard-code IP WSL vì NAT có thể đổi sau restart.
- Không tắt toàn bộ Windows Firewall; chỉ mở rule cần thiết trong private network.
- Chỉ bridge command/state/diagnostics cần thiết; xử lý camera/lidar raw tại nguồn
  nếu có thể.
- Custom type qua `ros1_bridge` phải được build/source ở cả ROS 1 và ROS 2; không
  giả định binary bridge biết custom message.
- Khai báo QoS theo dữ liệu; `/tf_static` cần durability phù hợp.
- Bridge không được là lớp E-stop duy nhất. E-stop vật lý và watchdog robot là
  bắt buộc.

## 7. Coding/config rules

- Tuân PEP 8, dòng mục tiêu tối đa 88 ký tự; module/class/hàm chỉ có trách nhiệm
  rõ ràng. Logic tính toán NÊN là pure function để test ngoài ROS.
- Cấm mutable default argument, nuốt exception, `print()` trong node production,
  `time.sleep()` trong callback và I/O không timeout.
- Shared core dùng `logging` chuẩn nếu cần; adapter dùng ROS logger.
- Constructor không thực hiện I/O lâu hoặc làm robot chuyển động.
- Validate type/range/quan hệ parameter khi startup; config sai phải fail trước
  khi actuator được enable.
- Command vận tốc phải có timeout; disconnect, stale/NaN/inf input, exception và
  shutdown phải đưa output về trạng thái an toàn.
- Dùng ROS clock đúng môi trường; simulation node hỗ trợ `use_sim_time`.
- Không đổi semantics interface mà giữ nguyên tên. Tần số/limit phải là parameter,
  không là magic number rải rác.
- YAML dùng 2 space, key `snake_case`, cấm tab. Launch chỉ wiring/remap/parameter,
  không chứa thuật toán.
- Xacro dùng SI (m, kg, s, rad), inertia hợp lệ/dương. Không duplicate wheel
  geometry, encoder resolution, gear ratio hoặc sign convention nhiều nơi.
- Secret, token, Wi-Fi, IP/device cá nhân dùng local override bị ignore.

## 8. Build và script

- Khai báo mọi dependency trong manifest; khóa dependency legacy dễ breaking.
- Cấm `sudo pip install`; không trộn Conda với ROS system Python nếu chưa kiểm
  tra interpreter, ABI và `PYTHONPATH`. Deactivate `(base)` nếu gây xung đột.
- Tách artifact ROS 1/ROS 2 và architecture; không dùng chung `build`, `install`,
  `devel`, `log`.
- Bash phải có shebang; nếu dùng Bash thì dùng `set -Eeuo pipefail`, usage/help,
  validate command/file, quote biến, fail với exit code khác 0 và setup idempotent.
- Không tự sửa `.bashrc`, apt source, udev hoặc system config mà không thông báo.
- Lệnh xóa/flash/reset phải validate target và yêu cầu xác nhận hoặc `--yes`.
- `run_sim.sh` chỉ chạy Jazzy trên host phù hợp; `run_robot.sh` chỉ chạy
  Jetson/Melodic; `flash_mcu.sh` phải yêu cầu MCU, device và firmware cụ thể.

## 9. Test và an toàn

Thứ tự xác minh: format/lint → unit test shared core → package build/test →
integration topic/TF/config → simulation headless → GUI → hardware-in-the-loop.

- Bug fix NÊN có regression test. Test có timeout, deterministic, không phụ thuộc
  Internet/device thật trừ khi được đánh dấu integration/hardware.
- Kinematics phải test zero, translation, rotation, forward/inverse consistency,
  saturation, wheel order và sign.
- Safety phải test stale command, disconnect, invalid input, exception, shutdown.
- Không tuyên bố chạy cả hai host nếu chưa test cả hai; báo rõ host đã test.
- Robot phải có E-stop vật lý, watchdog cục bộ, command timeout, giới hạn tốc độ/
  dòng và safe stop khi mất máy tính.
- Mặc định tốc độ/gia tốc bảo thủ. Reject NaN/inf/timestamp lỗi/out-of-range trước
  actuator. Log loop cao phải throttle.
- Diagnostics phải có connection, command age, frequency, battery, motor/sensor
  fault, CPU/RAM/temperature.

## 10. Git hygiene

- Commit tiếng Anh, imperative, một thay đổi logic; không trộn refactor lớn với
  feature hoặc format toàn repo.
- Không commit build/install/devel/log, `.gitnexus`, cache, bag, Gazebo log, core
  dump, model lớn, firmware binary, secret hoặc machine-local config.
- Không xóa/sửa thay đổi chưa commit ngoài phạm vi. Cấm `reset --hard`,
  `git clean -fd`, force push/rewrite history nếu chưa được yêu cầu.
- Linux phân biệt case còn Windows thường không: giữ đúng `AGENTS.MD`,
  `README.MD`, không tạo bản trùng khác chữ hoa/thường.

## 11. Quy trình bắt buộc cho AI agent

1. Xác định đích WSL/Jazzy simulation hay Jetson/Melodic robot và trình bày plan.
2. Kiểm tra Git status, bảo vệ thay đổi người dùng.
3. Dùng GitNexus-first theo Mục 5 rồi chỉ đọc file/symbol cần thiết.
4. Không tự thêm framework, dependency, container, C++ hoặc cấu trúc tài liệu.
5. Giữ shared core độc lập ROS và thay đổi nhỏ, rõ phạm vi.
6. Chạy lint/test/build khả dụng trên host hiện tại.
7. Kiểm tra diff, artifact, secret và hard-coded path cuối cùng.
8. Báo file đã đổi, lệnh/kết quả kiểm tra và phần chưa xác minh; không phóng đại.

## 12. Definition of Done

Thay đổi chỉ hoàn thành khi đúng layer/host, đúng naming/path/Python-first, shared
core không phụ thuộc ROS, interface/parameter/fail-safe hợp lệ, test khả dụng đã
chạy, diff không có secret/path cá nhân/artifact, impact được đánh giá khi đổi
API và mọi giới hạn chưa test được báo rõ. Nếu yêu cầu xung đột an toàn hoặc ma
trận tương thích, agent phải dừng phần rủi ro và hỏi chủ dự án.
