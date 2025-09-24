/**
 * Vietnamese Dictionary System for ESP32
 * Hỗ trợ từ ghép tiếng Việt và quản lý từ điển
 *
 * Sử dụng dữ liệu từ:
 * - @undertheseanlp/dictionary
 * - @lvdat/phobo-contribute-words
 *
 * Cập nhật lần cuối: 2025-01-27
 * Người phát triển: nguongthienTieu
 */

#ifndef VIETNAMESE_DICTIONARY_H
#define VIETNAMESE_DICTIONARY_H

#include <Arduino.h>
#include <Preferences.h>
#include <map>
#include <vector>
#include <set>

// Cấu trúc lưu trữ thông tin từ
struct WordInfo {
    String word;                // Từ gốc
    String meaning;            // Nghĩa
    uint8_t frequency;         // Tần suất sử dụng (0-255)
    bool isCompound;           // Có phải từ ghép không
    bool isDead;               // Từ "chết" (không sử dụng nữa)
    unsigned long lastUsed;    // Thời gian sử dụng gần nhất
    uint16_t usageCount;       // Số lần sử dụng
};

// Cấu trúc từ ghép 2 tiếng
struct CompoundWord {
    String firstPart;          // Thành phần đầu
    String secondPart;         // Thành phần thứ hai
    String fullWord;           // Từ ghép hoàn chỉnh
    String meaning;            // Nghĩa của từ ghép
};

class VietnameseDictionary {
private:
    Preferences preferences;
    std::map<String, WordInfo> dictionary;
    std::vector<CompoundWord> compoundWords;
    std::set<String> deadWords;
    
    static const uint16_t MAX_WORDS = 1000;          // Giới hạn từ vựng (do RAM ESP32)
    static const uint16_t MAX_COMPOUND_WORDS = 500;  // Giới hạn từ ghép
    static const uint32_t DEAD_WORD_TIMEOUT = 86400000; // 24h không dùng = từ chết
    
    // Các từ cơ bản từ @undertheseanlp/dictionary
    const char* basicWords[50] = {
        "xe", "máy", "chống", "trộm", "hệ", "thống", 
        "báo", "động", "cảnh", "báo", "tin", "nhắn",
        "vị", "trí", "GPS", "theo", "dõi", "an", "toàn",
        "bảo", "vệ", "chủ", "nhân", "phát", "hiện",
        "chuyển", "động", "cảm", "biến", "tín", "hiệu",
        "kết", "nối", "mạng", "WiFi", "SMS", "thông", "báo",
        "pin", "yếu", "sạc", "điện", "năng", "lượng",
        "ngủ", "thức", "hoạt", "động", "kiểm", "tra"
    };
    
    // Từ ghép phổ biến từ @lvdat/phobo-contribute-words
    const CompoundWord defaultCompounds[20] = {
        {"chống", "trộm", "chống trộm", "bảo vệ khỏi kẻ trộm"},
        {"hệ", "thống", "hệ thống", "tập hợp các thành phần"},
        {"báo", "động", "báo động", "cảnh báo nguy hiểm"},
        {"cảnh", "báo", "cảnh báo", "thông báo về nguy hiểm"},
        {"tin", "nhắn", "tin nhắn", "thông điệp văn bản"},
        {"vị", "trí", "vị trí", "tọa độ không gian"},
        {"theo", "dõi", "theo dõi", "giám sát liên tục"},
        {"an", "toàn", "an toàn", "không có nguy hiểm"},
        {"bảo", "vệ", "bảo vệ", "che chở, giữ gìn"},
        {"chủ", "nhân", "chủ nhân", "người sở hữu"},
        {"phát", "hiện", "phát hiện", "tìm ra, khám phá"},
        {"chuyển", "động", "chuyển động", "di chuyển"},
        {"cảm", "biến", "cảm biến", "thiết bị đo"},
        {"tín", "hiệu", "tín hiệu", "dấu hiệu thông tin"},
        {"kết", "nối", "kết nối", "liên kết với nhau"},
        {"thông", "báo", "thông báo", "đưa tin cho biết"},
        {"năng", "lượng", "năng lượng", "khả năng sinh công"},
        {"hoạt", "động", "hoạt động", "làm việc, vận hành"},
        {"kiểm", "tra", "kiểm tra", "xem xét, đánh giá"},
        {"bảo", "trì", "bảo trì", "duy trì, sửa chữa"}
    };

public:
    VietnameseDictionary();
    ~VietnameseDictionary();
    
    // Khởi tạo từ điển
    bool begin();
    void end();
    
    // Quản lý từ vựng
    bool addWord(const String& word, const String& meaning, bool isCompound = false);
    bool updateWord(const String& word, const String& newMeaning);
    bool removeWord(const String& word);
    WordInfo getWordInfo(const String& word);
    bool isWordExists(const String& word);
    
    // Quản lý từ ghép
    bool addCompoundWord(const String& firstPart, const String& secondPart, const String& meaning);
    bool isCompoundWord(const String& word);
    CompoundWord getCompoundInfo(const String& word);
    std::vector<CompoundWord> findPossibleCompounds(const String& firstPart);
    
    // Quản lý từ "chết"
    bool markWordAsDead(const String& word, const String& reason = "");
    bool reviveWord(const String& word);
    std::vector<String> getDeadWords();
    void cleanupDeadWords(); // Xóa từ chết quá lâu
    
    // Thống kê và phân tích
    void updateWordUsage(const String& word);
    std::vector<String> getMostUsedWords(uint8_t count = 10);
    std::vector<String> getLeastUsedWords(uint8_t count = 10);
    uint16_t getTotalWords();
    uint16_t getTotalCompoundWords();
    
    // Tìm kiếm và gợi ý
    std::vector<String> searchWords(const String& pattern);
    std::vector<String> suggestWords(const String& partial);
    bool isVietnameseWord(const String& word);
    
    // Xuất/nhập dữ liệu
    String exportToJSON();
    bool importFromJSON(const String& jsonData);
    void saveToPreferences();
    void loadFromPreferences();
    
    // Tích hợp với @undertheseanlp/dictionary và @lvdat/phobo-contribute-words
    void loadUndertheseanlpData();
    void loadPhoboContributeData();
    
    // Tiện ích
    String normalizeVietnameseText(const String& text);
    bool isValidVietnameseCharacter(char c);
    String removeVietnameseAccents(const String& text);
};

extern VietnameseDictionary vietnameseDictionary;

#endif // VIETNAMESE_DICTIONARY_H