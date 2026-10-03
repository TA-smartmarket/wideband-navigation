#include "navigation/json_parser.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace nav {

namespace {

const char* skipWhitespace(const char* cursor, const char* end) {
    while (cursor < end) {
        const char c = *cursor;
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            ++cursor;
        } else {
            break;
        }
    }
    return cursor;
}

/// Unescape a JSON string body [begin, end) into `out`.
bool unescapeString(const char* begin, const char* end, char* out, std::size_t out_size) {
    std::size_t written = 0;
    for (const char* cursor = begin; cursor < end; ++cursor) {
        char c = *cursor;
        if (c == '\\' && (cursor + 1) < end) {
            ++cursor;
            switch (*cursor) {
                case '"': c = '"'; break;
                case '\\': c = '\\'; break;
                case '/': c = '/'; break;
                case 'b': c = '\b'; break;
                case 'f': c = '\f'; break;
                case 'n': c = '\n'; break;
                case 'r': c = '\r'; break;
                case 't': c = '\t'; break;
                case 'u': {
                    // Only the ASCII range of \uXXXX is supported; anything else
                    // becomes '?' because the navigation contract is ASCII only.
                    if (cursor + 4 >= end) {
                        return false;
                    }
                    char digits[5] = {cursor[1], cursor[2], cursor[3], cursor[4], '\0'};
                    const unsigned long code = std::strtoul(digits, nullptr, 16);
                    c = (code < 0x80) ? static_cast<char>(code) : '?';
                    cursor += 4;
                    break;
                }
                default:
                    return false;
            }
        }
        if (out != nullptr && written + 1 < out_size) {
            out[written] = c;
        }
        ++written;
    }
    if (out != nullptr && out_size > 0) {
        out[written < out_size ? written : out_size - 1] = '\0';
    }
    return true;
}

/// Parse a string token, returning the body bounds (excluding the quotes).
bool parseStringBounds(const char* cursor, const char* end, const char*& body_begin,
                       const char*& body_end, const char*& next) {
    if (cursor >= end || *cursor != '"') {
        return false;
    }
    ++cursor;
    body_begin = cursor;
    while (cursor < end) {
        if (*cursor == '\\') {
            cursor += 2;  // skip the escaped character
            continue;
        }
        if (*cursor == '"') {
            body_end = cursor;
            next = cursor + 1;
            return true;
        }
        ++cursor;
    }
    return false;
}

/// Parse a value and report where it ended.
struct ParsedValue {
    JsonValue value;
    const char* next{nullptr};
};

ParsedValue parseValueEx(const char* cursor, const char* end, int depth);

ParsedValue parseObjectEx(const char* cursor, const char* end, int depth) {
    ParsedValue result;
    const char* object_begin = cursor;
    ++cursor;  // consume '{'
    cursor = skipWhitespace(cursor, end);
    if (cursor < end && *cursor == '}') {
        result.value = JsonValue(object_begin, cursor + 1, JsonValue::Type::OBJECT);
        result.next = cursor + 1;
        return result;
    }
    for (;;) {
        cursor = skipWhitespace(cursor, end);
        const char* key_begin = nullptr;
        const char* key_end = nullptr;
        const char* after_key = nullptr;
        if (!parseStringBounds(cursor, end, key_begin, key_end, after_key)) {
            return result;
        }
        cursor = skipWhitespace(after_key, end);
        if (cursor >= end || *cursor != ':') {
            return result;
        }
        ++cursor;
        cursor = skipWhitespace(cursor, end);
        const ParsedValue value = parseValueEx(cursor, end, depth + 1);
        if (!value.value.valid()) {
            return result;
        }
        cursor = skipWhitespace(value.next, end);
        if (cursor < end && *cursor == ',') {
            ++cursor;
            continue;
        }
        if (cursor < end && *cursor == '}') {
            result.value = JsonValue(object_begin, cursor + 1, JsonValue::Type::OBJECT);
            result.next = cursor + 1;
            return result;
        }
        return result;
    }
}

ParsedValue parseArrayEx(const char* cursor, const char* end, int depth) {
    ParsedValue result;
    const char* array_begin = cursor;
    ++cursor;  // consume '['
    cursor = skipWhitespace(cursor, end);
    if (cursor < end && *cursor == ']') {
        result.value = JsonValue(array_begin, cursor + 1, JsonValue::Type::ARRAY);
        result.next = cursor + 1;
        return result;
    }
    for (;;) {
        cursor = skipWhitespace(cursor, end);
        const ParsedValue value = parseValueEx(cursor, end, depth + 1);
        if (!value.value.valid()) {
            return result;
        }
        cursor = skipWhitespace(value.next, end);
        if (cursor < end && *cursor == ',') {
            ++cursor;
            continue;
        }
        if (cursor < end && *cursor == ']') {
            result.value = JsonValue(array_begin, cursor + 1, JsonValue::Type::ARRAY);
            result.next = cursor + 1;
            return result;
        }
        return result;
    }
}

ParsedValue parseNumberEx(const char* cursor, const char* end) {
    ParsedValue result;
    const char* begin = cursor;
    if (cursor < end && (*cursor == '-' || *cursor == '+')) {
        ++cursor;
    }
    bool has_digit = false;
    while (cursor < end) {
        const char c = *cursor;
        if ((c >= '0' && c <= '9')) {
            has_digit = true;
            ++cursor;
        } else if (c == '.' || c == 'e' || c == 'E' || c == '-' || c == '+') {
            ++cursor;
        } else {
            break;
        }
    }
    if (!has_digit) {
        return result;
    }
    result.value = JsonValue(begin, cursor, JsonValue::Type::NUMBER);
    result.next = cursor;
    return result;
}

ParsedValue parseValueEx(const char* cursor, const char* end, int depth) {
    ParsedValue result;
    if (depth > 12 || cursor >= end) {
        return result;  // bounded recursion depth
    }
    cursor = skipWhitespace(cursor, end);
    if (cursor >= end) {
        return result;
    }
    switch (*cursor) {
        case '{':
            return parseObjectEx(cursor, end, depth);
        case '[':
            return parseArrayEx(cursor, end, depth);
        case '"': {
            const char* body_begin = nullptr;
            const char* body_end = nullptr;
            const char* next = nullptr;
            if (!parseStringBounds(cursor, end, body_begin, body_end, next)) {
                return result;
            }
            result.value = JsonValue(body_begin, body_end, JsonValue::Type::STRING);
            result.next = next;
            return result;
        }
        case 't':
            if (end - cursor >= 4 && std::strncmp(cursor, "true", 4) == 0) {
                result.value = JsonValue(cursor, cursor + 4, JsonValue::Type::BOOLEAN);
                result.next = cursor + 4;
            }
            return result;
        case 'f':
            if (end - cursor >= 5 && std::strncmp(cursor, "false", 5) == 0) {
                result.value = JsonValue(cursor, cursor + 5, JsonValue::Type::BOOLEAN);
                result.next = cursor + 5;
            }
            return result;
        case 'n':
            if (end - cursor >= 4 && std::strncmp(cursor, "null", 4) == 0) {
                result.value = JsonValue(cursor, cursor + 4, JsonValue::Type::NULL_VALUE);
                result.next = cursor + 4;
            }
            return result;
        default:
            return parseNumberEx(cursor, end);
    }
}

}  // namespace

JsonValue jsonParse(const char* text) {
    if (text == nullptr) {
        return JsonValue();
    }
    return jsonParse(text, std::strlen(text));
}

JsonValue jsonParse(const char* text, std::size_t length) {
    if (text == nullptr || length == 0) {
        return JsonValue();
    }
    const char* end = text + length;
    const ParsedValue parsed = parseValueEx(text, end, 0);
    if (!parsed.value.valid()) {
        return JsonValue();
    }
    const char* trailing = skipWhitespace(parsed.next, end);
    if (trailing != end) {
        return JsonValue();  // trailing garbage is a syntax error
    }
    return parsed.value;
}

const char* jsonSkipWhitespace(const char* cursor, const char* end) {
    return skipWhitespace(cursor, end);
}

// ---------------------------------------------------------------------------
// JsonValue accessors
// ---------------------------------------------------------------------------

JsonValue JsonValue::member(const char* key) const {
    if (type_ != Type::OBJECT || key == nullptr) {
        return JsonValue();
    }
    const char* cursor = begin_ + 1;  // skip '{'
    const char* end = end_ - 1;       // exclude '}'
    cursor = skipWhitespace(cursor, end);
    while (cursor < end) {
        const char* key_begin = nullptr;
        const char* key_end = nullptr;
        const char* after_key = nullptr;
        if (!parseStringBounds(cursor, end, key_begin, key_end, after_key)) {
            return JsonValue();
        }
        cursor = skipWhitespace(after_key, end);
        if (cursor >= end || *cursor != ':') {
            return JsonValue();
        }
        ++cursor;
        cursor = skipWhitespace(cursor, end);
        const ParsedValue value = parseValueEx(cursor, end, 1);
        if (!value.value.valid()) {
            return JsonValue();
        }
        const std::size_t key_length = static_cast<std::size_t>(key_end - key_begin);
        if (std::strlen(key) == key_length && std::strncmp(key_begin, key, key_length) == 0) {
            return value.value;
        }
        cursor = skipWhitespace(value.next, end);
        if (cursor < end && *cursor == ',') {
            ++cursor;
            cursor = skipWhitespace(cursor, end);
            continue;
        }
        break;
    }
    return JsonValue();
}

JsonValue JsonValue::element(int index) const {
    if (type_ != Type::ARRAY || index < 0) {
        return JsonValue();
    }
    const char* cursor = begin_ + 1;  // skip '['
    const char* end = end_ - 1;       // exclude ']'
    int current = 0;
    for (;;) {
        cursor = skipWhitespace(cursor, end);
        if (cursor >= end) {
            return JsonValue();
        }
        const ParsedValue value = parseValueEx(cursor, end, 1);
        if (!value.value.valid()) {
            return JsonValue();
        }
        if (current == index) {
            return value.value;
        }
        ++current;
        cursor = skipWhitespace(value.next, end);
        if (cursor < end && *cursor == ',') {
            ++cursor;
            continue;
        }
        return JsonValue();
    }
}

int JsonValue::size() const {
    if (type_ != Type::ARRAY) {
        return 0;
    }
    const char* cursor = begin_ + 1;
    const char* end = end_ - 1;
    int count = 0;
    for (;;) {
        cursor = skipWhitespace(cursor, end);
        if (cursor >= end) {
            return count;
        }
        const ParsedValue value = parseValueEx(cursor, end, 1);
        if (!value.value.valid()) {
            return count;
        }
        ++count;
        cursor = skipWhitespace(value.next, end);
        if (cursor < end && *cursor == ',') {
            ++cursor;
            continue;
        }
        return count;
    }
}

bool JsonValue::asBool(bool fallback) const {
    if (type_ != Type::BOOLEAN) {
        return fallback;
    }
    return (end_ - begin_) == 4 && begin_[0] == 't';
}

float JsonValue::asFloat(float fallback) const {
    if (type_ != Type::NUMBER) {
        return fallback;
    }
    char buffer[32];
    const std::size_t length =
        static_cast<std::size_t>(end_ - begin_) < sizeof(buffer) - 1
            ? static_cast<std::size_t>(end_ - begin_)
            : sizeof(buffer) - 1;
    std::memcpy(buffer, begin_, length);
    buffer[length] = '\0';
    char* parse_end = nullptr;
    const double parsed = std::strtod(buffer, &parse_end);
    if (parse_end == buffer) {
        return fallback;
    }
    if (parsed != parsed || parsed > 3.0e38 || parsed < -3.0e38) {
        return fallback;
    }
    return static_cast<float>(parsed);
}

int JsonValue::asInt(int fallback) const {
    if (type_ != Type::NUMBER) {
        return fallback;
    }
    const float value = asFloat(static_cast<float>(fallback));
    return static_cast<int>(value);
}

uint64_t JsonValue::asUint64(uint64_t fallback) const {
    if (type_ != Type::NUMBER) {
        return fallback;
    }
    char buffer[32];
    const std::size_t length =
        static_cast<std::size_t>(end_ - begin_) < sizeof(buffer) - 1
            ? static_cast<std::size_t>(end_ - begin_)
            : sizeof(buffer) - 1;
    std::memcpy(buffer, begin_, length);
    buffer[length] = '\0';
    if (buffer[0] == '-') {
        return fallback;  // negative timestamps are invalid
    }
    char* parse_end = nullptr;
    const unsigned long long parsed = std::strtoull(buffer, &parse_end, 10);
    if (parse_end == buffer) {
        return fallback;
    }
    return static_cast<uint64_t>(parsed);
}

bool JsonValue::asString(char* out, std::size_t out_size) const {
    if (type_ != Type::STRING || out == nullptr || out_size == 0) {
        return false;
    }
    return unescapeString(begin_, end_, out, out_size);
}

std::size_t JsonValue::stringLength() const {
    if (type_ != Type::STRING) {
        return 0;
    }
    return static_cast<std::size_t>(end_ - begin_);
}

}  // namespace nav
