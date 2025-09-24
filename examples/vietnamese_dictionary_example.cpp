/**
 * Ví dụ sử dụng Hệ thống Từ điển Tiếng Việt
 * Vietnamese Dictionary System Usage Example
 * 
 * File này minh họa cách sử dụng hệ thống từ điển trong ESP32
 */

#include "vietnamese_dictionary.h"

void demonstrateDictionaryUsage() {
    Serial.println("=== Demo Hệ thống Từ điển Tiếng Việt ===");
    
    // 1. Khởi tạo hệ thống từ điển
    if (!vietnameseDictionary.begin()) {
        Serial.println("Lỗi: Không thể khởi tạo từ điển!");
        return;
    }
    
    // 2. Thêm từ mới vào hệ thống
    vietnameseDictionary.addWord("cảm_biến", "thiết bị đo đạc tín hiệu", false);
    vietnameseDictionary.addWord("vi_điều_khiển", "chip điều khiển nhỏ", false);
    vietnameseDictionary.addWord("kết_nối", "liên kết thông tin", false);
    
    // 3. Thêm từ ghép 2 tiếng
    vietnameseDictionary.addCompoundWord("cảm", "biến", "thiết bị phát hiện tín hiệu vật lý");
    vietnameseDictionary.addCompoundWord("vi", "điều_khiển", "bộ xử lý tín hiệu nhỏ gọn");
    vietnameseDictionary.addCompoundWord("thông", "minh", "có khả năng xử lý thông tin");
    vietnameseDictionary.addCompoundWord("tự", "động", "hoạt động không cần can thiệp");
    
    // 4. Tìm kiếm từ trong hệ thống
    Serial.println("\n--- Tìm kiếm từ chứa 'cảm' ---");
    std::vector<String> searchResults = vietnameseDictionary.searchWords("cảm");
    for (const String& word : searchResults) {
        WordInfo info = vietnameseDictionary.getWordInfo(word);
        Serial.print("Từ: ");
        Serial.print(word);
        Serial.print(" - Nghĩa: ");
        Serial.print(info.meaning);
        Serial.print(" - Từ ghép: ");
        Serial.println(info.isCompound ? "Có" : "Không");
    }
    
    // 5. Gợi ý từ dựa trên phần đầu
    Serial.println("\n--- Gợi ý từ bắt đầu bằng 'th' ---");
    std::vector<String> suggestions = vietnameseDictionary.suggestWords("th");
    for (const String& suggestion : suggestions) {
        Serial.print("Gợi ý: ");
        Serial.println(suggestion);
    }
    
    // 6. Kiểm tra từ ghép
    Serial.println("\n--- Kiểm tra từ ghép ---");
    String testWords[] = {"cảm biến", "vi điều_khiển", "thông minh"};
    for (int i = 0; i < 3; i++) {
        if (vietnameseDictionary.isCompoundWord(testWords[i])) {
            CompoundWord compound = vietnameseDictionary.getCompoundInfo(testWords[i]);
            Serial.print("Từ ghép: ");
            Serial.print(compound.fullWord);
            Serial.print(" = ");
            Serial.print(compound.firstPart);
            Serial.print(" + ");
            Serial.print(compound.secondPart);
            Serial.print(" - Nghĩa: ");
            Serial.println(compound.meaning);
        }
    }
    
    // 7. Đánh dấu từ "chết" (không còn dùng)
    Serial.println("\n--- Quản lý từ chết ---");
    vietnameseDictionary.markWordAsDead("từ_cũ", "Không còn sử dụng trong ngữ cảnh hiện đại");
    
    std::vector<String> deadWords = vietnameseDictionary.getDeadWords();
    Serial.print("Số từ chết hiện tại: ");
    Serial.println(deadWords.size());
    
    // 8. Thống kê từ điển
    Serial.println("\n--- Thống kê từ điển ---");
    Serial.print("Tổng số từ: ");
    Serial.println(vietnameseDictionary.getTotalWords());
    Serial.print("Tổng số từ ghép: ");
    Serial.println(vietnameseDictionary.getTotalCompoundWords());
    
    // 9. Từ được dùng nhiều nhất
    std::vector<String> mostUsed = vietnameseDictionary.getMostUsedWords(5);
    Serial.println("Top 5 từ được dùng nhiều:");
    for (size_t i = 0; i < mostUsed.size(); i++) {
        Serial.print((i+1));
        Serial.print(". ");
        Serial.println(mostUsed[i]);
    }
    
    // 10. Xuất dữ liệu JSON (để backup hoặc đồng bộ)
    Serial.println("\n--- Xuất dữ liệu JSON ---");
    String jsonData = vietnameseDictionary.exportToJSON();
    Serial.println("Dữ liệu JSON đã được tạo (chỉ hiển thị 200 ký tự đầu):");
    Serial.println(jsonData.substring(0, 200) + "...");
    
    Serial.println("\n=== Demo hoàn thành ===");
}

// Hàm xử lý lệnh từ điển qua Serial
void handleSerialDictionaryCommands() {
    if (Serial.available()) {
        String command = Serial.readString();
        command.trim();
        
        if (command.startsWith("dict add ")) {
            // Lệnh thêm từ: dict add [từ]:[nghĩa]
            int colonPos = command.indexOf(":");
            if (colonPos > 0) {
                String word = command.substring(9, colonPos);
                String meaning = command.substring(colonPos + 1);
                word.trim();
                meaning.trim();
                
                if (vietnameseDictionary.addWord(word, meaning)) {
                    Serial.println("✓ Đã thêm từ: " + word);
                } else {
                    Serial.println("✗ Không thể thêm từ");
                }
            } else {
                Serial.println("Cú pháp: dict add [từ]:[nghĩa]");
            }
        }
        else if (command.startsWith("dict search ")) {
            // Lệnh tìm kiếm: dict search [pattern]
            String pattern = command.substring(12);
            pattern.trim();
            
            std::vector<String> results = vietnameseDictionary.searchWords(pattern);
            Serial.print("Tìm thấy ");
            Serial.print(results.size());
            Serial.println(" từ:");
            
            for (const String& word : results) {
                Serial.println("  - " + word);
            }
        }
        else if (command == "dict stats") {
            // Lệnh thống kê
            Serial.println("=== Thống kê từ điển ===");
            Serial.print("Tổng từ: ");
            Serial.println(vietnameseDictionary.getTotalWords());
            Serial.print("Từ ghép: ");
            Serial.println(vietnameseDictionary.getTotalCompoundWords());
            
            std::vector<String> deadWords = vietnameseDictionary.getDeadWords();
            Serial.print("Từ chết: ");
            Serial.println(deadWords.size());
        }
        else if (command == "dict help") {
            // Hiển thị trợ giúp
            Serial.println("=== Lệnh từ điển qua Serial ===");
            Serial.println("dict add [từ]:[nghĩa]  - Thêm từ mới");
            Serial.println("dict search [pattern]  - Tìm kiếm từ");
            Serial.println("dict stats            - Xem thống kê");
            Serial.println("dict demo             - Chạy demo");
            Serial.println("dict help             - Hiển thị trợ giúp này");
        }
        else if (command == "dict demo") {
            // Chạy demo
            demonstrateDictionaryUsage();
        }
        else {
            Serial.println("Lệnh không hợp lệ. Gõ 'dict help' để xem trợ giúp.");
        }
    }
}

// Hàm khởi tạo từ điển tùy chỉnh cho dự án cụ thể
void initializeProjectSpecificVocabulary() {
    Serial.println("Khởi tạo từ vựng chuyên biệt cho hệ thống chống trộm...");
    
    // Thêm từ vựng chuyên ngành IoT và bảo mật
    vietnameseDictionary.addWord("IoT", "Internet of Things - Internet vạn vật", false);
    vietnameseDictionary.addWord("ESP32", "vi điều khiển của Espressif", false);
    vietnameseDictionary.addWord("WiFi", "kết nối mạng không dây", false);
    vietnameseDictionary.addWord("Bluetooth", "công nghệ kết nối tầm ngắn", false);
    vietnameseDictionary.addWord("4G", "mạng di động thế hệ thứ 4", false);
    
    // Từ ghép chuyên ngành
    vietnameseDictionary.addCompoundWord("chống", "nhiễu", "khử tạp âm điện");
    vietnameseDictionary.addCompoundWord("bảo", "mật", "đảm bảo an toàn thông tin");
    vietnameseDictionary.addCompoundWord("giám", "sát", "theo dõi và kiểm soát");
    vietnameseDictionary.addCompoundWord("cảnh", "báo", "thông báo nguy hiểm");
    vietnameseDictionary.addCompoundWord("tự", "động", "hoạt động không cần can thiệp");
    vietnameseDictionary.addCompoundWord("từ", "xa", "điều khiển qua khoảng cách");
    vietnameseDictionary.addCompoundWord("thời", "gian_thực", "xử lý tức thì");
    
    Serial.println("Đã khởi tạo từ vựng chuyên biệt");
}