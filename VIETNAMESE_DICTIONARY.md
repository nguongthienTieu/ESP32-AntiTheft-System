# Hệ thống Từ điển Tiếng Việt cho ESP32

## Tổng quan

Hệ thống từ điển tiếng Việt được tích hợp vào dự án ESP32 Anti-Theft System để hỗ trợ:
- Xử lý từ ghép 2 tiếng tiếng Việt
- Quản lý từ vựng (thêm, sửa, xóa)
- Theo dõi từ "chết" (không còn sử dụng)
- Tích hợp dữ liệu từ @undertheseanlp/dictionary và @lvdat/phobo-contribute-words

## Tính năng chính

### 1. Quản lý từ vựng cơ bản
- **Thêm từ mới**: Bổ sung từ vào từ điển
- **Cập nhật nghĩa**: Sửa đổi nghĩa của từ đã có
- **Xóa từ**: Loại bỏ từ khỏi từ điển
- **Tìm kiếm**: Tìm từ theo pattern

### 2. Hỗ trợ từ ghép tiếng Việt
- **Từ ghép 2 thành phần**: "chống trộm", "hệ thống", "báo động"
- **Tự động nhận diện**: Phát hiện từ ghép trong văn bản
- **Gợi ý từ ghép**: Đề xuất các từ ghép có thể từ thành phần đầu

### 3. Quản lý từ "chết"
- **Đánh dấu từ chết**: Đánh dấu từ không còn sử dụng
- **Khôi phục từ**: Khôi phục từ đã đánh dấu chết
- **Tự động dọn dẹp**: Xóa từ chết không sử dụng trong thời gian dài

### 4. Thống kê và phân tích
- **Thống kê sử dụng**: Theo dõi tần suất sử dụng từ
- **Từ phổ biến**: Danh sách từ được sử dụng nhiều nhất
- **Từ ít dùng**: Danh sách từ ít được sử dụng

## Sử dụng qua SMS

Hệ thống hỗ trợ các lệnh SMS để quản lý từ điển từ xa:

### Lệnh thống kê
```
DICT STATS
```
Trả về: Số lượng từ, từ ghép và từ chết trong hệ thống

### Thêm từ mới
```
DICT ADD xe:phương tiện di chuyển
```
Thêm từ "xe" với nghĩa "phương tiện di chuyển"

### Thêm từ ghép
```
DICT COMPOUND chống trộm:bảo vệ khỏi kẻ trộm
```
Thêm từ ghép "chống trộm" với nghĩa đã cho

### Đánh dấu từ chết
```
DICT DEAD từ_cũ
```
Đánh dấu "từ_cũ" là từ không còn sử dụng

### Khôi phục từ
```
DICT REVIVE từ_cũ
```
Khôi phục "từ_cũ" từ trạng thái chết

### Tìm kiếm từ
```
DICT SEARCH chống
```
Tìm tất cả từ chứa "chống"

### Dọn dẹp từ chết
```
DICT CLEANUP
```
Xóa các từ chết không sử dụng lâu

## Tích hợp dữ liệu

### @undertheseanlp/dictionary
Từ điển cơ sở với các từ tiếng Việt phổ biến:
- Từ vựng cơ bản hàng ngày
- Thuật ngữ kỹ thuật
- Từ ghép thông dụng

### @lvdat/phobo-contribute-words
Bộ sưu tập từ ghép tiếng Việt:
- Từ ghép 2 thành phần
- Nghĩa và cách sử dụng
- Từ mới xuất hiện

## Cấu trúc dữ liệu

### WordInfo
```cpp
struct WordInfo {
    String word;                // Từ gốc
    String meaning;            // Nghĩa
    uint8_t frequency;         // Tần suất sử dụng (0-255)
    bool isCompound;           // Có phải từ ghép không
    bool isDead;               // Từ "chết"
    unsigned long lastUsed;    // Thời gian sử dụng gần nhất
    uint16_t usageCount;       // Số lần sử dụng
};
```

### CompoundWord
```cpp
struct CompoundWord {
    String firstPart;          // Thành phần đầu
    String secondPart;         // Thành phần thứ hai
    String fullWord;           // Từ ghép hoàn chỉnh
    String meaning;            // Nghĩa của từ ghép
};
```

## Giới hạn hệ thống

Do ESP32 có RAM hạn chế:
- **Tối đa 1000 từ** trong từ điển chính
- **Tối đa 500 từ ghép**
- **Tự động dọn dẹp** từ chết sau 24 giờ không sử dụng

## API sử dụng trong code

```cpp
// Khởi tạo
vietnameseDictionary.begin();

// Thêm từ
vietnameseDictionary.addWord("xe", "phương tiện", false);

// Thêm từ ghép
vietnameseDictionary.addCompoundWord("chống", "trộm", "bảo vệ khỏi kẻ trộm");

// Tìm kiếm
std::vector<String> results = vietnameseDictionary.searchWords("chống");

// Đánh dấu từ chết
vietnameseDictionary.markWordAsDead("từ_cũ", "không còn dùng");

// Thống kê
uint16_t totalWords = vietnameseDictionary.getTotalWords();
```

## Tương lai phát triển

1. **Hỗ trợ dấu tiếng Việt**: Xử lý đầy đủ các ký tự có dấu
2. **Machine Learning**: Tự động phát hiện từ mới
3. **Đồng bộ từ xa**: Cập nhật từ điển từ server
4. **Phân loại từ**: Danh từ, động từ, tính từ
5. **Từ đồng nghĩa**: Hỗ trợ từ có nghĩa tương tự

## Cài đặt và sử dụng

1. Include header file:
```cpp
#include "vietnamese_dictionary.h"
```

2. Khởi tạo trong setup():
```cpp
if (vietnameseDictionary.begin()) {
    Serial.println("Từ điển đã sẵn sàng");
}
```

3. Sử dụng các API như mô tả ở trên

## Đóng góp

Để đóng góp vào hệ thống từ điển:
1. Fork dự án
2. Thêm từ vào file dữ liệu
3. Test với ESP32
4. Tạo Pull Request

---
*Phát triển bởi nguongthienTieu - 2025*