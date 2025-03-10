#ifndef MOCK_WSTRING_H
#define MOCK_WSTRING_H

#include <string>
#include <cstring>

class String {
private:
    std::string m_str;

public:
    // Constructors
    String() : m_str() {}
    String(const char* cstr) : m_str(cstr ? cstr : "") {}
    String(const std::string& str) : m_str(str) {}
    String(int val) : m_str(std::to_string(val)) {}
    String(float val, int precision = 2) : m_str(std::to_string(val)) {
        // Crude approximation of float to string with precision
        size_t dotPos = m_str.find('.');
        if (dotPos != std::string::npos) {
            m_str = m_str.substr(0, dotPos + precision + 1);
        }
    }

    // Conversion methods
    const char* c_str() const { return m_str.c_str(); }
    std::string toStdString() const { return m_str; }

    // Length
    size_t length() const { return m_str.length(); }
    bool isEmpty() const { return m_str.empty(); }

    // Comparison
    bool equals(const String& other) const { return m_str == other.m_str; }
    bool equalsIgnoreCase(const String& other) const {
        if (m_str.length() != other.m_str.length()) return false;
        for (size_t i = 0; i < m_str.length(); ++i) {
            if (std::tolower(m_str[i]) != std::tolower(other.m_str[i])) return false;
        }
        return true;
    }

    // Conversion methods
    int toInt() const { return std::stoi(m_str); }
    float toFloat() const { return std::stof(m_str); }

    // Operators
    String& operator=(const char* cstr) {
        m_str = cstr ? cstr : "";
        return *this;
    }

    String& operator=(const String& other) {
        m_str = other.m_str;
        return *this;
    }

    bool operator==(const String& other) const { return m_str == other.m_str; }
    bool operator!=(const String& other) const { return m_str != other.m_str; }

    // Concatenation
    String operator+(const String& other) const {
        return String(m_str + other.m_str);
    }

    // Indexing
    char operator[](size_t index) const {
        return m_str[index];
    }

    // Substring
    String substring(size_t start, size_t length = std::string::npos) const {
        return String(m_str.substr(start, length));
    }

    // Other useful methods
    int indexOf(char ch) const {
        size_t pos = m_str.find(ch);
        return pos == std::string::npos ? -1 : pos;
    }

    int indexOf(const String& substr) const {
        size_t pos = m_str.find(substr.c_str());
        return pos == std::string::npos ? -1 : pos;
    }

    void trim() {
        // Remove leading and trailing whitespace
        size_t start = m_str.find_first_not_of(" \t\n\r");
        size_t end = m_str.find_last_not_of(" \t\n\r");
        if (start == std::string::npos) {
            m_str.clear();
        } else {
            m_str = m_str.substr(start, end - start + 1);
        }
    }

    // Stream-like methods
    String& toLowerCase() {
        for (char& c : m_str) {
            c = std::tolower(c);
        }
        return *this;
    }
};

#endif // MOCK_WSTRING_H