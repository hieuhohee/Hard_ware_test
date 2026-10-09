# Quy tắc làm việc trong workspace MicroMouse

## Sao lưu Git trước khi sửa

Người dùng yêu cầu: **"Nhớ git sao lưu commit lại project trước khi thay đổi"**.

- Trước mỗi đợt thay đổi mã nguồn, cấu hình hoặc tài liệu, kiểm tra `git status` và lịch sử commit.
- Nếu có thay đổi chưa commit hoặc file nguồn mới thuộc project sắp sửa, tạo commit sao lưu trạng thái hiện tại **trước khi chỉnh sửa**. Quy tắc này là yêu cầu thường trực của người dùng, không cần hỏi lại quyền đồng ý về ý định; vẫn tuân thủ cơ chế cấp quyền ghi `.git` của môi trường.
- Nếu các file liên quan đã sạch và có commit, dùng commit đó làm mốc; không cần tạo commit rỗng cho từng thao tác nhỏ trong cùng đợt.
- Chỉ stage các file liên quan đã kiểm tra. Không gom thư mục build, cache, file tạm, bản clone tham khảo hoặc thông tin nhạy cảm vào backup một cách tự động. Không xóa/chỉnh sửa công việc khác của người dùng.
- Không amend, reset hoặc viết lại commit sao lưu để thay thế bản trước. Không push lên remote nếu chưa được yêu cầu.
- Nếu commit không thực hiện được, báo rõ nguyên nhân và chưa sửa file được yêu cầu cho đến khi có mốc sao lưu phù hợp.
- Khi báo kết quả, nêu mã commit sao lưu để người dùng có thể quay lại bản trước.

## Phạm vi project hiện tại

- `micromouse-control/`: điều khiển PID ba lớp, driver STM32, hướng dẫn và host tests.
- `hardware-tests/`: test module rời, cấu hình mẫu, hướng dẫn và host tests.
- Giữ riêng hai firmware; không tự gộp vòng lặp hoặc callback của chúng.

Commit gốc sao lưu hai project: `f93a0e3` (09/10/2026). Khi Git chưa có danh tính người dùng, commit này dùng danh tính tác giả `Codex <codex@local.invalid>` ở phạm vi lệnh, không đặt cấu hình global. Những lần sau ưu tiên danh tính Git đã được người dùng cấu hình; không tự giả danh người dùng.
