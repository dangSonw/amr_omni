# Original User Request

## Initial Request — 2026-09-19T09:58:21Z

Nâng cấp và hoàn thiện hệ thống robot tự hành Mecanum `amr_omni` dựa trên các tài liệu kỹ thuật chuyên sâu trong `amr_omni/temp/docs`: Triển khai bộ ước lượng vận tốc Encoder chính xác cao không trễ pha, thuật toán hiệu chuẩn nội tại IMU và lọc nhiễu, hiệu chuẩn LiDAR và hiệu chuẩn liên cảm biến (Spatial & Temporal) chống phân kỳ EKF khi robot rung lắc/chạy tốc độ cao; trong đó toàn bộ thuật toán tính toán hiệu chuẩn do vi điều khiển STM32 xử lý, còn Jetson Web UI chịu trách nhiệm gửi lệnh kích hoạt và hiển thị trực quan kết quả.

Working directory: /home/sonev/teamwork_projects/amr_omni_calib
Integrity mode: development

## Reference Materials
- `amr_omni/temp/docs/amr_omni.docx`: Báo cáo kiến trúc hệ thống, đối chiếu chuẩn công nghiệp, rủi ro và lộ trình ưu tiên (Single TF authority, STM32 Contract, Kinematics test, Covariance tuning).
- `amr_omni/temp/docs/Encoder.docx`: Thuật toán ước lượng vận tốc động cơ: Bộ quan sát bám pha bậc 2 (ODrive Second-Order PLL Tracking Observer) và phương pháp lai M/T của LinuxCNC.
- `amr_omni/temp/docs/Calib.docx`: Hiệu chuẩn nội tại IMU: Mô hình toán học Tedaldi et al. (ICRA 2014), thuật toán `imu_tk`, phương pháp 6 hướng ST AN4508 (`imu_calib`) và phân tích Allan variance (`imu_utils`).
- `amr_omni/temp/docs/Calib_2.docx`: Hiệu chuẩn không gian (Spatial lever-arm) và thời gian (Temporal latency) đa cảm biến liên tục B-spline theo nguyên lý Toolbox Kalibr (ETH Zurich).

## Requirements

### R1. Bộ ước lượng vận tốc Encoder độ chính xác cao trên STM32
Hiện thực thuật toán ước lượng vị trí và vận tốc bánh xe mượt mà, triệt tiêu sai số lượng tử hóa ở tốc độ rất thấp và hiện tượng trễ pha khi tăng tốc đột ngột (áp dụng Bộ quan sát trạng thái bám pha bậc 2 - Second-Order PLL Tracking Observer từ ODrive hoặc phương pháp lai M/T từ LinuxCNC theo tài liệu `Encoder.docx`). Tự động tính toán bù sai số bán kính bánh xe và đảm bảo tính nhất quán giữa forward và inverse kinematics.

### R2. Hiệu chuẩn nội tại và khử nhiễu cảm biến IMU trên vi điều khiển STM32
Hiện thực routine hiệu chuẩn cảm biến quán tính 6-DoF trực tiếp trên vi điều khiển STM32: ước lượng và bù trừ độ lệch tĩnh (bias tracking) của Gyroscope và ma trận tỉ lệ/lệch trục phi trực giao của Accelerometer theo các phương pháp chuẩn (như 6 hướng tĩnh hoặc nhận diện khoảng tĩnh). Dữ liệu sau hiệu chuẩn được lọc nhiễu và phát theo chuẩn toạ độ ENU (East-North-Up) của ROS.

### R3. Hiệu chuẩn liên cảm biến (Spatial & Temporal) và cấu hình bộ lọc EKF chống trôi khi rung lắc
Xác lập ma trận biến đổi toạ độ không gian chính xác (lever-arm extrinsics giữa tâm robot `base_link`, IMU và LiDAR) và bù độ trễ truyền thông thời gian (temporal latency). Cập nhật ma trận hiệp phương sai sai số đo đạc thực tế (Covariance matrix) vào bộ lọc EKF (`robot_localization`) thay thế cho các ma trận mặc định, đảm bảo hệ thống ước lượng odometry ổn định, không bị giật cục hay phân kỳ khi robot chạy nhanh, phanh gấp hoặc rung lắc trên mặt sàn không bằng phẳng. Cấu hình laser filter triệt tiêu các góc quét điểm mù trúng thân vỏ robot.

### R4. Giao thức điều khiển hiệu chuẩn và hiển thị trực quan trên Jetson Web UI
Xây dựng pipeline kích hoạt và trực quan hóa hiệu chuẩn hoàn chỉnh: Giao diện Web (FastAPI backend + Web frontend) cho phép người vận hành gửi lệnh điều khiển (bắt đầu hiệu chuẩn IMU, hiệu chuẩn bánh xe, đo phổ nhiễu), giao tiếp hai chiều với STM32 qua khung truyền nhị phân Serial Contract. STM32 thực thi tính toán và gửi kết quả ma trận hiệu chuẩn/bias về Jetson để tự động lưu vào file cấu hình ROS 2 YAML, đồng thời hiển thị trạng thái và đồ thị sai số trực quan trên Web UI.

### R5. Tối ưu hóa kiến trúc, ưu tiên thư viện và hàm có sẵn
Ưu tiên tái sử dụng tối đa các thư viện, package và hàm chuẩn có sẵn của ROS 2 và hệ thống (như `robot_localization`, `laser_filters`, các hàm ma trận/vector chuẩn). Mã nguồn triển khai phải tinh gọn, súc tích, cấu trúc rõ ràng, dễ đọc, dễ bảo trì và hạn chế tối đa việc tự viết lại các thuật toán đã có thư viện chuẩn giải quyết tốt.

## Acceptance Criteria

### Encoder & Kinematics
- [ ] Thuật toán ước lượng vận tốc encoder trên STM32 phản hồi mượt mà trong toàn bộ dải tốc độ (từ < 0.05 m/s đến > 1.5 m/s) mà không bị gai nhiễu lượng tử hóa hay trễ đáp ứng.
- [ ] Module Kinematics vượt qua bài kiểm tra tính nhất quán toán học `forward_kinematics(inverse_kinematics(v)) == v` với sai số < 1e-5 và triệt tiêu hoàn toàn giá trị NaN/Inf.

### IMU Calibration & Noise Filtering
- [ ] Routine hiệu chuẩn trên STM32 tính toán chính xác bias của Gyroscope, đảm bảo trôi dạt góc xoay tĩnh < 0.05 deg/s khi robot đứng yên.
- [ ] Dữ liệu IMU sau hiệu chuẩn tuân thủ nghiêm ngặt hệ toạ độ ENU, loại bỏ gia tốc trọng trường ở trạng thái tĩnh và không còn hiện tượng góc quay tự trôi.

### EKF & Sensor Fusion Robustness
- [ ] Ma trận Covariance của EKF được điền đầy đủ các giá trị phương sai thực tế đo đạc được từ cảm biến; tuyệt đối không còn phần tử đường chéo nào bằng 0.
- [ ] Dưới tác động mô phỏng tăng tốc đột ngột, phanh gấp và rung lắc mạnh, bộ lọc EKF duy trì ước lượng odometry liên tục, không bị nhảy bước (jump) hay phân kỳ toạ độ.
- [ ] Áp dụng triệt để nguyên tắc Single Authority TF: chỉ duy nhất một node broadcast transform `odom -> base_link`, loại bỏ hoàn toàn cảnh báo duplicate TF trong hệ thống.
- [ ] Laser filter lọc sạch các tia phản xạ trúng khung gầm robot mà không làm mất chùm tia quét môi trường thực tế.

### Web UI & Calibration Workflow
- [ ] Người dùng có thể khởi chạy quy trình hiệu chuẩn IMU và Encoder từ giao diện Web Jetson bằng 1 cú nhấp chuột.
- [ ] Trạng thái tiến trình hiệu chuẩn hiển thị theo thời gian thực (real-time progress/status).
- [ ] Tham số hiệu chuẩn sau khi STM32 tính toán xong được gửi về Web backend, tự động cập nhật vào file cấu hình YAML tương ứng và hiển thị bảng thông số trực quan trên giao diện.

### Test Automation & Code Quality
- [ ] Toàn bộ bộ test suite tự động (unit test kinematics, test STM32 contract bridge, test EKF config verification, test Web calibration API) vượt qua 100%.
- [ ] Mã nguồn tuân thủ tiêu chí ngắn gọn, súc tích, có chú thích giải thích rõ ràng và tài liệu hướng dẫn quy trình hiệu chuẩn từng bước.

## Follow-up — 2026-09-19T11:11:54Z

Người dùng yêu cầu tiếp tục công việc đã giao trước đó: tiếp tục triển khai từ Milestone 2 (IMU Intrinsic Calibration trên STM32), Milestone 3 (EKF Covariance Tuning & Laser Filter Masking), Milestone 4 (Binary Serial Protocol & FastAPI Web UI Calibration), và hoàn tất Milestone 5 (toàn bộ 151 E2E tests). Vui lòng tiếp tục điều phối và thực thi ngay.
