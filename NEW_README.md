# ESP32 Anti-Theft System với Hệ thống Từ điển Tiếng Việt

## Tổng quan

Hệ thống chống trộm xe thông minh sử dụng ESP32 với tính năng độc đáo: **Hệ thống từ điển tiếng Việt tích hợp** hỗ trợ xử lý từ ghép và quản lý từ vựng.

### Tính năng chính

🔒 **Chống trộm thông minh**
- Phát hiện chuyển động qua cảm biến MPU6050
- Theo dõi vị trí GPS (ATGM336H)
- Báo động 3 giai đoạn
- Điều khiển từ xa qua RF433MHz

📱 **Kết nối và thông báo**
- SMS qua module SIM A7682S
- WiFi và Blynk v1
- Thông báo real-time

🧠 **Hệ thống từ điển tiếng Việt** (NEW!)
- Xử lý từ ghép 2 tiếng
- Quản lý từ vựng thông minh
- Tích hợp @undertheseanlp/dictionary và @lvdat/phobo-contribute-words
- Điều khiển qua SMS

## API từ điển

```cpp
#include "vietnamese_dictionary.h"

// Khởi tạo
vietnameseDictionary.begin();

// Thêm từ ghép
vietnameseDictionary.addCompoundWord("chống", "trộm", "bảo vệ");

// Tìm kiếm
std::vector<String> results = vietnameseDictionary.searchWords("chống");
```

## Tác giả

**nguongthienTieu** - *Phát triển hệ thống từ điển tiếng Việt*
