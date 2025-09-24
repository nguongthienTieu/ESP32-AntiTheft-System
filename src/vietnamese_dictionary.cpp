/**
 * Vietnamese Dictionary System Implementation
 * Triển khai hệ thống từ điển tiếng Việt cho ESP32
 */

#include "vietnamese_dictionary.h"
#include <algorithm>  // For std::sort, std::min

// Global instance
VietnameseDictionary vietnameseDictionary;

VietnameseDictionary::VietnameseDictionary() {
    // Constructor
}

VietnameseDictionary::~VietnameseDictionary() {
    end();
}

bool VietnameseDictionary::begin() {
    Serial.println("Khởi tạo hệ thống từ điển tiếng Việt...");
    
    if (!preferences.begin("vn_dict", false)) {
        Serial.println("Lỗi: Không thể khởi tạo Preferences cho từ điển");
        return false;
    }
    
    // Tải dữ liệu cơ bản
    loadUndertheseanlpData();
    loadPhoboContributeData();
    
    // Tải dữ liệu đã lưu
    loadFromPreferences();
    
    Serial.print("Đã khởi tạo từ điển với ");
    Serial.print(getTotalWords());
    Serial.print(" từ và ");
    Serial.print(getTotalCompoundWords());
    Serial.println(" từ ghép");
    
    return true;
}

void VietnameseDictionary::end() {
    saveToPreferences();
    preferences.end();
    dictionary.clear();
    compoundWords.clear();
    deadWords.clear();
}

bool VietnameseDictionary::addWord(const String& word, const String& meaning, bool isCompound) {
    if (word.length() == 0 || meaning.length() == 0) {
        return false;
    }
    
    if (dictionary.size() >= MAX_WORDS) {
        Serial.println("Cảnh báo: Đã đạt giới hạn từ vựng");
        return false;
    }
    
    WordInfo info;
    info.word = word;
    info.meaning = meaning;
    info.frequency = 1;
    info.isCompound = isCompound;
    info.isDead = false;
    info.lastUsed = millis();
    info.usageCount = 1;
    
    dictionary[word] = info;
    
    Serial.print("Đã thêm từ: ");
    Serial.print(word);
    Serial.print(" - ");
    Serial.println(meaning);
    
    return true;
}

bool VietnameseDictionary::updateWord(const String& word, const String& newMeaning) {
    auto it = dictionary.find(word);
    if (it != dictionary.end()) {
        it->second.meaning = newMeaning;
        it->second.lastUsed = millis();
        it->second.usageCount++;
        
        Serial.print("Đã cập nhật từ: ");
        Serial.print(word);
        Serial.print(" - nghĩa mới: ");
        Serial.println(newMeaning);
        
        return true;
    }
    return false;
}

bool VietnameseDictionary::removeWord(const String& word) {
    auto it = dictionary.find(word);
    if (it != dictionary.end()) {
        dictionary.erase(it);
        deadWords.erase(word);
        
        Serial.print("Đã xóa từ: ");
        Serial.println(word);
        
        return true;
    }
    return false;
}

WordInfo VietnameseDictionary::getWordInfo(const String& word) {
    auto it = dictionary.find(word);
    if (it != dictionary.end()) {
        return it->second;
    }
    
    WordInfo empty;
    empty.word = "";
    return empty;
}

bool VietnameseDictionary::isWordExists(const String& word) {
    return dictionary.find(word) != dictionary.end();
}

bool VietnameseDictionary::addCompoundWord(const String& firstPart, const String& secondPart, const String& meaning) {
    if (compoundWords.size() >= MAX_COMPOUND_WORDS) {
        Serial.println("Cảnh báo: Đã đạt giới hạn từ ghép");
        return false;
    }
    
    CompoundWord compound;
    compound.firstPart = firstPart;
    compound.secondPart = secondPart;
    compound.fullWord = firstPart + " " + secondPart;
    compound.meaning = meaning;
    
    compoundWords.push_back(compound);
    
    // Thêm từ ghép vào từ điển chính
    addWord(compound.fullWord, meaning, true);
    
    Serial.print("Đã thêm từ ghép: ");
    Serial.print(compound.fullWord);
    Serial.print(" - ");
    Serial.println(meaning);
    
    return true;
}

bool VietnameseDictionary::isCompoundWord(const String& word) {
    for (const auto& compound : compoundWords) {
        if (compound.fullWord == word) {
            return true;
        }
    }
    return false;
}

CompoundWord VietnameseDictionary::getCompoundInfo(const String& word) {
    for (const auto& compound : compoundWords) {
        if (compound.fullWord == word) {
            return compound;
        }
    }
    
    CompoundWord empty;
    empty.fullWord = "";
    return empty;
}

std::vector<CompoundWord> VietnameseDictionary::findPossibleCompounds(const String& firstPart) {
    std::vector<CompoundWord> results;
    
    for (const auto& compound : compoundWords) {
        if (compound.firstPart == firstPart) {
            results.push_back(compound);
        }
    }
    
    return results;
}

bool VietnameseDictionary::markWordAsDead(const String& word, const String& reason) {
    auto it = dictionary.find(word);
    if (it != dictionary.end()) {
        it->second.isDead = true;
        deadWords.insert(word);
        
        Serial.print("Đã đánh dấu từ chết: ");
        Serial.print(word);
        if (reason.length() > 0) {
            Serial.print(" - Lý do: ");
            Serial.print(reason);
        }
        Serial.println();
        
        return true;
    }
    return false;
}

bool VietnameseDictionary::reviveWord(const String& word) {
    auto it = dictionary.find(word);
    if (it != dictionary.end() && it->second.isDead) {
        it->second.isDead = false;
        it->second.lastUsed = millis();
        deadWords.erase(word);
        
        Serial.print("Đã khôi phục từ: ");
        Serial.println(word);
        
        return true;
    }
    return false;
}

std::vector<String> VietnameseDictionary::getDeadWords() {
    std::vector<String> result;
    for (const String& word : deadWords) {
        result.push_back(word);
    }
    return result;
}

void VietnameseDictionary::cleanupDeadWords() {
    unsigned long currentTime = millis();
    auto it = dictionary.begin();
    
    while (it != dictionary.end()) {
        if (it->second.isDead && 
            (currentTime - it->second.lastUsed) > DEAD_WORD_TIMEOUT) {
            
            String word = it->first;
            deadWords.erase(word);
            it = dictionary.erase(it);
            
            Serial.print("Đã xóa từ chết: ");
            Serial.println(word);
        } else {
            ++it;
        }
    }
}

void VietnameseDictionary::updateWordUsage(const String& word) {
    auto it = dictionary.find(word);
    if (it != dictionary.end()) {
        it->second.lastUsed = millis();
        it->second.usageCount++;
        it->second.frequency = min(255, it->second.frequency + 1);
        
        // Nếu từ đã chết mà được sử dụng lại, khôi phục nó
        if (it->second.isDead) {
            reviveWord(word);
        }
    }
}

std::vector<String> VietnameseDictionary::getMostUsedWords(uint8_t count) {
    std::vector<std::pair<String, uint16_t>> wordUsage;
    
    for (const auto& pair : dictionary) {
        wordUsage.push_back({pair.first, pair.second.usageCount});
    }
    
    // Sắp xếp theo số lần sử dụng giảm dần
    std::sort(wordUsage.begin(), wordUsage.end(), 
        [](const auto& a, const auto& b) { return a.second > b.second; });
    
    std::vector<String> result;
    for (size_t i = 0; i < min((size_t)count, wordUsage.size()); i++) {
        result.push_back(wordUsage[i].first);
    }
    
    return result;
}

std::vector<String> VietnameseDictionary::getLeastUsedWords(uint8_t count) {
    std::vector<std::pair<String, uint16_t>> wordUsage;
    
    for (const auto& pair : dictionary) {
        if (!pair.second.isDead) { // Không bao gồm từ chết
            wordUsage.push_back({pair.first, pair.second.usageCount});
        }
    }
    
    // Sắp xếp theo số lần sử dụng tăng dần
    std::sort(wordUsage.begin(), wordUsage.end(), 
        [](const auto& a, const auto& b) { return a.second < b.second; });
    
    std::vector<String> result;
    for (size_t i = 0; i < min((size_t)count, wordUsage.size()); i++) {
        result.push_back(wordUsage[i].first);
    }
    
    return result;
}

uint16_t VietnameseDictionary::getTotalWords() {
    return dictionary.size();
}

uint16_t VietnameseDictionary::getTotalCompoundWords() {
    return compoundWords.size();
}

std::vector<String> VietnameseDictionary::searchWords(const String& pattern) {
    std::vector<String> results;
    String lowerPattern = pattern;
    lowerPattern.toLowerCase();
    
    for (const auto& pair : dictionary) {
        String word = pair.first;
        word.toLowerCase();
        
        if (word.indexOf(lowerPattern) >= 0) {
            results.push_back(pair.first);
        }
    }
    
    return results;
}

std::vector<String> VietnameseDictionary::suggestWords(const String& partial) {
    std::vector<String> results;
    String lowerPartial = partial;
    lowerPartial.toLowerCase();
    
    for (const auto& pair : dictionary) {
        String word = pair.first;
        word.toLowerCase();
        
        if (word.startsWith(lowerPartial)) {
            results.push_back(pair.first);
            if (results.size() >= 10) break; // Giới hạn 10 gợi ý
        }
    }
    
    return results;
}

bool VietnameseDictionary::isVietnameseWord(const String& word) {
    // Kiểm tra xem từ có chứa ký tự tiếng Việt không
    for (int i = 0; i < word.length(); i++) {
        char c = word.charAt(i);
        if (isValidVietnameseCharacter(c)) {
            return true;
        }
    }
    return false;
}

String VietnameseDictionary::exportToJSON() {
    String json = "{\"words\":[";
    bool first = true;
    
    for (const auto& pair : dictionary) {
        if (!first) json += ",";
        json += "{\"word\":\"" + pair.first + "\"";
        json += ",\"meaning\":\"" + pair.second.meaning + "\"";
        json += ",\"frequency\":" + String(pair.second.frequency);
        json += ",\"isCompound\":" + String(pair.second.isCompound ? "true" : "false");
        json += ",\"isDead\":" + String(pair.second.isDead ? "true" : "false");
        json += ",\"usageCount\":" + String(pair.second.usageCount) + "}";
        first = false;
    }
    
    json += "],\"compounds\":[";
    first = true;
    for (const auto& compound : compoundWords) {
        if (!first) json += ",";
        json += "{\"firstPart\":\"" + compound.firstPart + "\"";
        json += ",\"secondPart\":\"" + compound.secondPart + "\"";
        json += ",\"fullWord\":\"" + compound.fullWord + "\"";
        json += ",\"meaning\":\"" + compound.meaning + "\"}";
        first = false;
    }
    json += "]}";
    
    return json;
}

bool VietnameseDictionary::importFromJSON(const String& jsonData) {
    // Triển khai đơn giản cho việc import JSON
    // Trong thực tế cần parser JSON đầy đủ
    Serial.println("Import từ JSON chưa được triển khai đầy đủ");
    return false;
}

void VietnameseDictionary::saveToPreferences() {
    // Lưu một số thống kê cơ bản
    preferences.putUShort("total_words", dictionary.size());
    preferences.putUShort("total_compounds", compoundWords.size());
    preferences.putUShort("dead_words", deadWords.size());
    
    Serial.println("Đã lưu dữ liệu từ điển vào Preferences");
}

void VietnameseDictionary::loadFromPreferences() {
    uint16_t savedWords = preferences.getUShort("total_words", 0);
    uint16_t savedCompounds = preferences.getUShort("total_compounds", 0);
    uint16_t savedDeadWords = preferences.getUShort("dead_words", 0);
    
    Serial.print("Tải dữ liệu từ Preferences: ");
    Serial.print(savedWords);
    Serial.print(" từ, ");
    Serial.print(savedCompounds);
    Serial.print(" từ ghép, ");
    Serial.print(savedDeadWords);
    Serial.println(" từ chết");
}

void VietnameseDictionary::loadUndertheseanlpData() {
    Serial.println("Tải dữ liệu từ @undertheseanlp/dictionary...");
    
    // Tải các từ cơ bản
    for (int i = 0; i < 50; i++) {
        String word = String(basicWords[i]);
        if (!isWordExists(word)) {
            addWord(word, "Từ cơ bản tiếng Việt", false);
        }
    }
    
    Serial.println("Đã tải xong dữ liệu undertheseanlp");
}

void VietnameseDictionary::loadPhoboContributeData() {
    Serial.println("Tải dữ liệu từ @lvdat/phobo-contribute-words...");
    
    // Tải các từ ghép mặc định
    for (int i = 0; i < 20; i++) {
        const CompoundWord& compound = defaultCompounds[i];
        if (!isCompoundWord(compound.fullWord)) {
            addCompoundWord(compound.firstPart, compound.secondPart, compound.meaning);
        }
    }
    
    Serial.println("Đã tải xong dữ liệu phobo-contribute-words");
}

String VietnameseDictionary::normalizeVietnameseText(const String& text) {
    String normalized = text;
    normalized.trim();
    normalized.toLowerCase();
    
    // Loại bỏ khoảng trắng thừa
    while (normalized.indexOf("  ") >= 0) {
        normalized.replace("  ", " ");
    }
    
    return normalized;
}

bool VietnameseDictionary::isValidVietnameseCharacter(char c) {
    // Kiểm tra ký tự tiếng Việt cơ bản
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')) {
        return true;
    }
    
    // Có thể mở rộng để kiểm tra ký tự có dấu
    // Do giới hạn của ESP32, chỉ kiểm tra cơ bản
    return false;
}

String VietnameseDictionary::removeVietnameseAccents(const String& text) {
    String result = text;
    
    // Đơn giản hóa: chỉ xử lý một số trường hợp cơ bản
    // Trong thực tế cần bảng chuyển đổi đầy đủ
    result.replace("á", "a");
    result.replace("à", "a");
    result.replace("ả", "a");
    result.replace("ã", "a");
    result.replace("ạ", "a");
    
    result.replace("é", "e");
    result.replace("è", "e");
    result.replace("ẻ", "e");
    result.replace("ẽ", "e");
    result.replace("ẹ", "e");
    
    // Thêm các ký tự khác...
    
    return result;
}