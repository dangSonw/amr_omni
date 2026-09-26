# KHUNG TIÊU CHUẨN ĐÁNH GIÁ DỰ ÁN KỸ THUẬT (REVIEW.MD)

> **Mục đích:** Tài liệu này đóng vai trò là bản đặc tả nguyên tắc, tiêu chí đánh giá và cấu trúc báo cáo chuẩn (`RESULT_REVIEW.md`). Dành cho AI hoặc kỹ sư thẩm định độc lập tiến hành rà soát mã nguồn, kiểm soát chất lượng kỹ thuật và đánh giá khả năng triển khai thực tế của dự án phần mềm/robot tự hành/hệ thống nhúng.

---

## 1. NGUYÊN TẮC THẨM ĐỊNH CỐT LÕI (CORE PRINCIPLES)

Khi tiến hành kiểm tra mã nguồn, AI đánh giá phải tuân thủ các nguyên tắc sau:
1. **Safety-First (An toàn là trên hết):** Bất kỳ cơ chế xử lý ngoại lệ nào dẫn đến trạng thái mất an toàn (*fail-open*, thả trôi hệ thống, bỏ qua giới hạn phần cứng) đều phải xếp vào mức độ lỗi cao nhất.
2. **Deterministic & Real-time (Tiền định & Thời gian thực):** Các luồng điều khiển, callback dữ liệu cảm biến tần số cao không được chứa tác vụ gây nghẽn (blocking I/O, tính toán nặng không giới hạn).
3. **Concurrency & Thread-Safety (Đồng thời & An toàn luồng):** Đảm bảo ranh giới luồng rõ ràng giữa luồng điều khiển chính, luồng nền, giao diện bất đồng bộ (async/WebSocket) và các biến trạng thái chia sẻ (shared states).
4. **Physical & Mathematical Fidelity (Chân thực toán học & vật lý):** Các phép biến đổi tọa độ, ma trận động học, thông số quán tính và giới hạn cơ cấu chấp hành phải phản ánh đúng thực tế cơ khí.
5. **Actionable & Non-Redundant (Hành động cụ thể, tránh trùng lặp):**
   * Tập trung vào nguyên nhân gốc rễ (root cause); nếu một lỗi gốc gây ra nhiều hệ quả ở các file khác nhau, gom nhóm thành một vấn đề duy nhất.
   * **Bỏ qua những phần đã được sửa** hoặc mã nguồn thử nghiệm tạm thời nếu không ảnh hưởng tới luồng vận hành chính.
   * Tránh bắt bẻ hình thức vụn vặt; ưu tiên các vấn đề ảnh hưởng trực tiếp đến độ ổn định, hiệu năng và an toàn phần cứng.

---

## 2. BỘ TIÊU CHÍ ĐÁNH GIÁ TỔNG QUAN (AUDIT CRITERIA)

### Tiêu chí 1: Kiến trúc hệ thống & Tính module hóa (Trọng số: 15%)
* Phân tầng logic rõ ràng: Tách biệt giữa tầng giao tiếp phần cứng (Hardware Layer), tầng tính toán động học/điều khiển (Control/Kinematics), tầng an toàn (Safety Layer), tầng điều hướng (Navigation) và tầng ứng dụng (Bridge/Web API).
* Cấu hình tham số tách biệt khỏi mã nguồn thực thi (YAML/Config); hỗ trợ cấu hình linh hoạt theo môi trường (mô phỏng vs thực tế).
* Chính sách truyền thông (QoS) được thiết lập phù hợp với tính chất của từng luồng dữ liệu (dữ liệu cảm biến streaming vs lệnh điều khiển tin cậy).

### Tiêu chí 2: Cơ chế an toàn & Giám sát (Trọng số: 20%)
* Cơ chế dừng khẩn cấp (E-Stop): Cả ở mức phần mềm và mức giao tiếp phần cứng.
* Bộ giám sát mất tín hiệu (Command Watchdog): Kiểm tra tần số và thời gian timeout của lệnh điều khiển; biên độ timeout phải đủ an toàn trước hiện tượng trễ chu kỳ (jitter) nhưng đủ nhanh để dừng máy khi mất kết nối.
* Vùng bảo vệ an toàn (Safety Zones): Logic phát hiện vật cản theo hướng chuyển động thực tế; xử lý giảm tốc mượt mà và dừng an toàn.

### Tiêu chí 3: Hiệu năng tính toán & Ngân sách thời gian thực (Trọng số: 20%)
* Ngân sách chu kỳ (Cycle Time Budget): Đảm bảo thời gian tính toán của từng chu kỳ điều khiển không vượt quá chu kỳ danh định.
* Tải tài nguyên mục tiêu: Đánh giá mức độ chiếm dụng CPU, RAM và băng thông truyền thông trên phần cứng tính toán đích (SBC, Jetson, PC công nghiệp).
* Độ trễ toàn trình (End-to-End Latency): Ước tính độ trễ từ lúc nhập lệnh điều khiển đến phản hồi cơ cấu chấp hành.

### Tiêu chí 4: Quản lý luồng, Bất đồng bộ & Toàn vẹn dữ liệu (Trọng số: 15%)
* Bảo vệ tài nguyên chia sẻ bằng cơ chế khóa thích hợp (Mutex/Locks), ngăn ngừa hiện tượng đọc dữ liệu không nguyên tử (torn read) hoặc tranh chấp (race conditions).
* Tính đồng bộ giữa luồng bất đồng bộ (Async Event Loop) và luồng xử lý đồng bộ/đa luồng.
* Nhất quán trạng thái ước lượng: Ngăn chặn hiện tượng ghi đè không điều kiện giữa các nguồn dữ liệu cảm biến khác nhau lên cùng một trạng thái.

### Tiêu chí 5: Động học & Tính toán vật lý (Trọng số: 10%)
* Tính chính xác của mô hình động học (thuận/nghịch), bảo toàn tỉ lệ vận tốc khi cơ cấu chấp hành đạt giới hạn bão hòa.
* Tính đúng đắn của các tham số vật lý trong mô hình mô phỏng (khối lượng, khối tâm, ma trận mômen quán tính).

### Tiêu chí 6: Khả năng sẵn sàng trên phần cứng thật (Trọng số: 10%)
* Tính ổn định của định danh thiết bị ngoại vi (cổng nối tiếp, cảm biến).
* Quy trình xử lý mất kết nối, tự động kết nối lại và tự kiểm tra lỗi (self-check/diagnostic) khi khởi động.
* Cấu hình quản lý tiến trình tự động (Services/Daemons) phục vụ triển khai thực tế.

### Tiêu chí 7: Chất lượng mã nguồn & Khả năng bảo trì (Trọng số: 10%)
* Tính gọn gàng, tuân thủ nguyên tắc DRY (Don't Repeat Yourself).
* Định kiểu dữ liệu (Type hinting), tài liệu hóa giao diện và khả năng kiểm thử tự động (Unit/Integration Tests).

---

## 3. PHÂN CẤP MỨC ĐỘ NGHIÊM TRỌNG (SEVERITY TAXONOMY)

* 🔴 **CRITICAL:** Vi phạm an toàn nghiêm trọng (fail-open), xung đột trạng thái gây mất lái, race condition gây crash/segfault hoặc nguy cơ hư hỏng thiết bị phần cứng.
* 🟠 **HIGH:** Vi phạm ngân sách thời gian thực, blocking trên luồng chính, biên độ timeout không hợp lý, nguy cơ rò rỉ tài nguyên hệ thống.
* 🟡 **MEDIUM:** Thiết kế chưa tối ưu, lặp mã logic, thiếu timeout kiểm soát trên vòng lặp phụ, cấu hình giao thức chưa khai thác hết hiệu năng.
* 🟢 **LOW / STYLE:** Bất cập về phong cách mã nguồn, thiếu chú thích tại các hằng số tính toán, định dạng dữ liệu hoặc log không cần thiết.

---

## 4. QUY CHUẨN CẤU TRÚC FILE BÁO CÁO (`amr_omni/temp/docs/RESULT_REVIEW.MD`)

AI thực hiện đánh giá **bắt buộc** xuất kết quả theo đúng cấu trúc chuẩn sau:

```markdown
# [TÊN DỰ ÁN] — BÁO CÁO ĐÁNH GIÁ TOÀN DIỆN & KIỂM SOÁT KỸ THUẬT

> **Thời điểm thẩm định:** [YYYY-MM-DD] | **Phạm vi thẩm định:** [Các module/thư mục được rà soát]  
> **Điểm tổng kết chất lượng:** [X.X / 10] | **Trạng thái:** [Sẵn sàng thử nghiệm / Cần xử lý lỗi chặn]

---

## 0. Tổng Quan Kiến Trúc & Luồng Dữ Liệu
- [Sơ đồ khối luồng dữ liệu kiến trúc (ASCII hoặc Mermaid)]
- [Đánh giá tổng quan về tính hợp lý và sự phân tầng của hệ thống]

---

## 1. Dự Toán Hiệu Năng & Tài Nguyên
### 1.1 Dự toán tải CPU trên phần cứng mục tiêu
| Tiến trình / Node | Tần số (Hz) | Đặc điểm xử lý | % CPU ước tính |
|---|---|---|---|
| ... | ... | ... | ... |

### 1.2 Phân bổ Bộ nhớ (RAM) & Băng thông
| Thành phần | Dung lượng RAM ước tính | Cảnh báo / Đánh giá |
|---|---|---|
| ... | ... | ... |

### 1.3 Phân tích Độ trễ Toàn trình (End-to-End Latency)
| Chặng truyền thông / Xử lý | Độ trễ ước tính | Nhận xét tính khả thi |
|---|---|---|
| ... | ... | ... |

---

## 2. Danh Sách Lỗi & Vấn Đề Kỹ Thuật

### 🔴 CRITICAL (Bắt buộc sửa trước khi vận hành phần cứng)
#### C1. [Tiêu đề lỗi vắn tắt] ([Đường dẫn file]:[Dòng liên quan nếu có])
- **Hiện tượng & Cơ chế phát sinh:** [Mô tả chi tiết nguyên nhân kỹ thuật]
- **Nguy cơ thực tế:** [Rủi ro an toàn, sai lệch thuật toán hoặc hỏng hóc]
- **Đoạn mã có vấn đề:**
```language
// Code snippet