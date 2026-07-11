# Robot dò line ESP32 sử dụng điều khiển PID

## 1. Giới thiệu

Dự án xây dựng một robot tự hành có khả năng bám theo vạch đen bằng vi điều khiển ESP32, năm cảm biến dò line và mạch điều khiển động cơ L298N. ESP32 liên tục đọc trạng thái của dãy cảm biến, xác định độ lệch giữa vị trí robot và tâm đường line, sau đó áp dụng bộ điều khiển PID để điều chỉnh tốc độ của hai động cơ. Nhờ thay đổi chênh lệch tốc độ giữa bánh trái và bánh phải, robot có thể tự hiệu chỉnh hướng đi khi lệch khỏi đường line.

## 2. Chức năng chính

Hệ thống đọc đồng thời năm cảm biến dò line dạng tín hiệu số, quy đổi từng trạng thái cảm biến thành sai số trong khoảng từ `-4` đến `4` và sử dụng sai số này làm đầu vào cho bộ điều khiển PID. Khi robot đi đúng giữa đường line, sai số bằng `0` nên hai động cơ chạy gần như cùng tốc độ. Khi line lệch về một phía, chương trình tạo giá trị hiệu chỉnh để tăng tốc một bánh và giảm tốc bánh còn lại, giúp robot quay trở lại tâm line.

Ngoài khả năng bám line, chương trình còn tự giảm tốc độ nền từ `80` xuống `60` khi phát hiện sai số có độ lớn từ `3` trở lên.  

## 3. Thành phần phần cứng

Hệ thống gồm một bo mạch ESP32, năm cảm biến dò line có ngõ ra số, một mạch điều khiển động cơ L298N, hai động cơ DC giảm tốc, nguồn cấp phù hợp cho động cơ và vi điều khiển, khung xe, bánh xe cùng dây kết nối. Hai khối động cơ được điều khiển độc lập thông qua hai kênh của L298N, trong đó chân ENA và ENB nhận tín hiệu PWM để điều chỉnh tốc độ, còn các chân IN1–IN4 xác định chiều quay.

## 4. Sơ đồ kiến trúc hệ thống

<img width="1270" height="529" alt="image" src="https://github.com/user-attachments/assets/36131874-8a45-4c27-a7da-8bacbd0d2006" />

## 5. Kết nối chân

### 5.1. Cảm biến dò line

Các cảm biến được sắp xếp theo thứ tự từ trái sang phải là `S0`, `S1`, `S2`, `S3`, `S4`. Mức logic `1` được quy ước là cảm biến phát hiện vạch đen.

| Vị trí cảm biến | Chỉ số trong code | GPIO ESP32 |
|---|---:|---:|
| Ngoài cùng bên trái | `sensorPin[0]` | GPIO 36 |
| Bên trái | `sensorPin[1]` | GPIO 39 |
| Chính giữa | `sensorPin[2]` | GPIO 34 |
| Bên phải | `sensorPin[3]` | GPIO 35 |
| Ngoài cùng bên phải | `sensorPin[4]` | GPIO 32 |

### 5.2. Mạch điều khiển động cơ L298N

| Chức năng | Chân L298N | GPIO ESP32 |
|---|---|---:|
| PWM động cơ trái | ENA | GPIO 25 |
| Chiều động cơ trái | IN1 | GPIO 26 |
| Chiều động cơ trái | IN2 | GPIO 27 |
| PWM động cơ phải | ENB | GPIO 33 |
| Chiều động cơ phải | IN3 | GPIO 14 |
| Chiều động cơ phải | IN4 | GPIO 12 |

Trong hàm `motorControl()`, chương trình đặt `IN1` và `IN3` ở mức cao, đồng thời đặt `IN2` và `IN4` ở mức thấp để hai động cơ quay theo chiều tiến. Nếu một động cơ quay ngược do cách đấu dây thực tế, có thể đảo hai dây của động cơ đó hoặc đổi trạng thái hai chân điều khiển chiều tương ứng.

> ESP32, mạch cảm biến và L298N phải nối chung GND để tín hiệu điều khiển có cùng mức tham chiếu.

## 6. Quy đổi trạng thái cảm biến thành sai số

Hàm `readSensors()` đọc năm cảm biến và quy đổi vị trí của vạch đen thành một giá trị sai số. Sai số âm cho biết line nằm về phía trái của dãy cảm biến, sai số dương cho biết line nằm về phía phải, còn sai số bằng `0` nghĩa là cảm biến giữa đang nằm trên line.

| Trạng thái `S0 S1 S2 S3 S4` | Sai số |
|---|---:|
| `1 0 0 0 0` | `-4` |
| `1 1 0 0 0` | `-3` |
| `0 1 0 0 0` | `-2` |
| `0 1 1 0 0` | `-1` |
| `0 0 1 0 0` | `0` |
| `0 0 1 1 0` | `1` |
| `0 0 0 1 0` | `2` |
| `0 0 0 1 1` | `3` |
| `0 0 0 0 1` | `4` |

Những tổ hợp không có trong bảng sẽ không làm thay đổi giá trị `error`, vì trong hàm hiện tại không có nhánh `else` mặc định. Do đó, robot sẽ tiếp tục hiệu chỉnh theo sai số hợp lệ gần nhất nếu gặp một mẫu cảm biến chưa được định nghĩa.

## 7. Bộ điều khiển PID

Bộ điều khiển PID được cài đặt trong hàm `calculatePID()` với các hệ số ban đầu:

```cpp
float Kp = 28;
float Ki = 0;
float Kd = 10;
```

Giá trị điều khiển được tính theo công thức:

```text
PID = Kp × P + Ki × I + Kd × D
```

Trong đó, thành phần tỉ lệ `P` bằng sai số hiện tại, thành phần tích phân `I` là tổng sai số qua các chu kỳ và thành phần vi phân `D` là độ chênh lệch giữa sai số hiện tại với sai số trước đó. Giá trị PID sau khi tính được giới hạn trong khoảng từ `-80` đến `80` để tránh tạo mức chênh lệch tốc độ quá lớn.

Với cấu hình hiện tại, `Ki` bằng `0` nên thành phần tích phân chưa tham gia điều khiển. Robot chủ yếu phản ứng theo độ lệch hiện tại thông qua `Kp` và tốc độ thay đổi của độ lệch thông qua `Kd`.

## 8. Điều khiển tốc độ động cơ

Tốc độ nền của robot được đặt bằng biến:

```cpp
int baseSpeed = 80;
```

Khi robot chạy trên đoạn thẳng hoặc chỉ lệch nhẹ, tốc độ động được giữ bằng `80`. Khi `abs(error) >= 3`, chương trình giảm tốc độ động xuống `60` để robot không lao quá nhanh khi gặp góc cua lớn.

Tốc độ hai bên được tính như sau:

```text
Tốc độ trái = Tốc độ nền + PID
Tốc độ phải = Tốc độ nền - PID
```

Sau đó, mỗi giá trị được giới hạn trong khoảng PWM từ `0` đến `255`. Khi PID dương, động cơ trái chạy nhanh hơn và động cơ phải chạy chậm hơn, làm robot chuyển hướng về bên phải. Khi PID âm, động cơ trái chạy chậm hơn và động cơ phải chạy nhanh hơn, làm robot chuyển hướng về bên trái.

PWM được cấu hình với tần số `1000 Hz` và độ phân giải `8 bit` bằng hàm `ledcAttach()`. Chương trình hiện sử dụng cú pháp LEDC theo API mới của Arduino-ESP32, trong đó `ledcWrite()` nhận trực tiếp chân PWM.

## 9. Luồng hoạt động của chương trình

Khi ESP32 khởi động, hàm `setup()` mở cổng Serial ở tốc độ `115200`, cấu hình năm chân cảm biến ở chế độ đầu vào, cấu hình các chân điều khiển chiều động cơ ở chế độ đầu ra và gắn hai chân ENA, ENB vào bộ tạo PWM LEDC.

Trong mỗi vòng lặp, chương trình trước tiên cộng trạng thái của năm cảm biến. Nếu tổng bằng `5`, nghĩa là cả năm cảm biến đều ở mức `1`, hai động cơ được dừng và vòng lặp hiện tại kết thúc. Nếu điều kiện dừng không xảy ra, chương trình lần lượt đọc cảm biến, xác định sai số, tính giá trị PID và cập nhật tốc độ hai động cơ.

<img width="607" height="1032" alt="image" src="https://github.com/user-attachments/assets/e54f9576-8a7f-4dc8-9314-f65cac690597" />


