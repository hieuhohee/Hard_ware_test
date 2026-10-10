# Rà soát cấu hình IMU — 10/10/2026

**Kết quả:** cấu hình SPI/clock/pin đang khai báo hợp lệ cho ICM20602.
Đã chạy firmware C chỉ dùng GPIO/SPI và dò36 cấu hình;288/288 ID đều0x2E,
0 mẫu0x12. Chưa đạt nhận dạng/đọc gyro hợp lệ, không công bố test gyro xong.

## Cấu hình đã kiểm tra

| Mục | Mã/.ioc và phép đo thực |
|---|---|
| Clock nguồn | HSI16MHz, PLL M8/N168/P2/Q7; HSE25MHz chỉ là giá trị khai báo, không được chọn làm nguồn |
| Clock bus | HCLK168MHz, APB1 42MHz, APB2 84MHz: đọc HAL/RCC trên bo đúng |
| Flash/power | Voltage Scale1, FLASH_LATENCY_5 phù hợp thiết lập168MHz |
| SPI2 | Master, full-duplex2-lines,8-bit,MSB, TI/CRC disable |
| CPOL/CPHA | Mode0 mặc định; đã đo thêm mode3, hai mode chip hỗ trợ |
| NSS/CS | NSS software; PB12 GPIO output push-pull, HIGH trước khởi tạo/giao dịch; không dùng PB12 AF-NSS |
| Chân SPI | PB13=SCK, PB14=MISO, PB15=MOSI, AF5. Generated spi.c đúng mapping |
| GPIO speed | Very High mặc định; đã đo thêm Low để thử ảnh hưởng độ dốc cạnh |
| MISO pull | NoPull mặc định; đã đo thêm PullUp/PullDown trong lúc test, sau đó khôi phục |
| Baudrate mặc định | PCLK1/64=656250Hz. SPI lấy PCLK1, không lấy timer clock84MHz |
| Ngoại vi khác | TIM3 PB4/5, TIM4 PB6/7, TIM8 PC6/7, TIM5 PA0/1, ADC PA3/4 PB0/1 PC0, UART3 PC10/11. Không thấy dùng lại PB12–PB15 |
| Ngắt/DMA | Bài này HAL SPI polling, không dùng DMA/IRQ SPI hoặc EXTI INT để đọc ID |
| Cấu hình build | .ioc thay Toolchain sang CMake, không đổi các tham số SPI/clock trong diff hiện tại |
| CMake warning | configureArgs còn -DCMAKE_COMMAND=cube-cmake; không cần override biến này. Liên quan configure/build, chưa có bằng chứng gây ID sai; không sửa thay đổi người dùng trong settings.json |

NSS soft và CS GPIO là cách dùng chủ động; chọn lại Hardware NSS ở CubeMX
có thể làm hỏng cách driver điều khiển CS. HAL_GPIO MODE_AF_PP cho cả bộ
chân SPI là cấu hình CubeMX thông thường; khi master hoạt động, MISO là
đường vào của peripheral. Không đổi PB14 sang output GPIO để "sửa ID".

## Đã sửa gì để loại ảnh hưởng cấu hình

- Mode C3/C4 chỉ gọi MX_GPIO_Init và MX_SPI2_Init; không khởi tạo ADC,
  timer hay UART. Firmware mode0/1/2 vẫn có các init như trước.
- Thêm IcmC_ConfigAudit vào bài C4. Không sửa generated spi.c/gpio.c hoặc .ioc,
  không đổi chân PCB; chỉ thử HAL Init CPOL/CPHA/divider và GPIO speed/pull.
- Lưu36 profile và mọi ID vào icm_c_config.trial, kiểu số32-bit dễ đọc Watch.
  Chỉ chọn profile nếu8/8 ID12, HAL_OK, interface readback40; kiểm tra tiếp
  16 ID nếu có candidate. Không có candidate ở phép đo thật, nên đã khôi phục
  SPI/GPIO ban đầu, selected0, restore_ok1. Không chuyển ID2E thành12.
- Build, nạp/verify và đọc snapshot RAM trực tiếp đã hoàn tất.

## 36 profile và số liệu thực

Tổ hợp2 ×3 ×2 ×3 =36:

- Mode:0 hoặc3.
- Divider:64,128,256 →656.25,328.125,164.0625kHz tại PCLK1=42MHz.
- GPIO speed:Low(encoding0) hoặcVery High(encoding3).
- Pull MISO:None(0),Up(1),Down(2).
- Mỗi profile8 giao dịch WHO riêng. Mỗi profile viết I2C_IF40 rồi đọc lại.

| Kết quả | Số lượng |
|---|---:|
| ID0x2E | 288/288 |
| ID0x12 | 0/288 |
| Lỗi HAL trong matrix | 0 |
| Readback I2C_IF0x40 | 36/36 |
| ID2E với MISO NoPull | 96/96 |
| ID2E với MISO PullUp | 96/96 |
| ID2E với MISO PullDown | 96/96 |
| Profile được chọn | Không có |
| Khôi phục cấu hình ban đầu | Thành công |

Clock đọc được: HCLK168000000, PCLK1 42000000, PCLK2 84000000.
GPIO sau khôi phục: MODER A900028F, AFRH55500000, OSPEEDR FC0000C0,
PUPDR00000100. Giá trị GPIOB khác bản có init timer vì PB4 không được timer
khởi tạo; phần PB12/13/14/15 vẫn CS output/AF5 đúng. PUPDR bit ở PB4 là
trạng thái của chân khác, không phải Pull MISO còn sót lại.

SPI CR1 ngay sau HAL Init32C, SPE chưa bật; trong truyền HAL sẽ bật SPE.
Không coi khác biệt32C/36C là lỗi tốc độ SPI: khác nhau ở bit SPE.

ARM build PASS, FLASH10680B/RAM4216B. ELF chỉ có MX_GPIO_Init/MX_SPI2_Init
cho các peripheral init. Nạp/verify PASS, ST-Link SN37FF71064E5734368AEC1743,
tham chiếu3.26V, SWD125kHz. Snapshot config2208B ở0x200001D0; các địa chỉ
này chỉ đúng với ELF phiên này.

Raw snapshot: tmp/imu-probe/config-matrix.bin. Dữ liệu36 dòng và SHA256 ELF:
tmp/imu-probe/config-matrix.json. Không stage các file tạm/snapshot vào Git.

## Vì sao chưa thể báo hoàn thành

Tới thời điểm này, sai ID còn tồn tại sau khi cô lập ngoại vi và thử các
cấu hình được liệt kê. Giao diện I2C độc lập ở phiên trước cũng trả2E.
Chưa có bằng chứng để sửa một tham số CubeMX làm phản hồi2E trở thành12.
WHO_AM_I trong datasheet là chỉ đọc, không có cấu hình hợp lệ để gán ID mới.

Các phép đo không chứng nhận VDD/VDDIO tại IC, loại level shifter/regulator
của breakout hay mã silicon thực tế. Điện áp ST-Link là tại bo, không phải
phép đo trực tiếp nguồn chân ICM. Phần còn cần xác minh là đúng IC/module
và phản hồi trên chân của nó, hoặc đối chiếu module ICM20602 đã biết đúng.
Không kết luận chip giả/hỏng chỉ từ2E; cũng không nhận module này là ICM20602
để bật vòng điều khiển khi chưa có ID/readback/mẫu gyro hợp lệ.

## Sử dụng và sao lưu

Main đang GYRO_IDENTITY_ONLY=4: Begin C → ConfigAudit một lần → Poll C trong
while. Xem icm_c_config.done/selected/trial và icm_c.current_id/id_ok.
Đổi selector bằng3 để trở lại bài ID C đơn giản; bằng0 để bài gyro C++.
Không gọi ConfigAudit lặp trong while. CubeMX regenerate có thể thêm lại
các MX_* init vì .ioc vẫn giữ mapping toàn mouse; cần giữ nhánh cô lập C.

Mốc trước sửa: afef1ed. Chỉ sửa source C/header/main và báo cáo, không gom
.ioc/settings/generated library chưa commit của người dùng vào backup.

Nguồn chuẩn: [TDK ICM20602 DS-000176 rev1.1](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000176-icm-20602-v1.1.pdf).
