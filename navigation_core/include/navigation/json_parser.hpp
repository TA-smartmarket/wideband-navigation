// Minimal dependency-free JSON reader.
//
// Rationale: the ESP32 firmware must parse a position line per UWB update and
// occasionally a config blob without pulling in a heavyweight JSON library and
// without heap allocation.  This parser is a strict, non-allocating recursive
// descent reader over a caller-provided buffer; every accessor is bounds
// checked and returns the supplied fallback on any mismatch.
//
// Supported syntax: objects, arrays, strings (with the common escapes), numbers,
// true/false/null.  Duplicate keys resolve to the first occurrence.
#pragma once

#include <cstddef>
#include <cstdint>

namespace nav {

class JsonValue {
public:
    enum class Type : uint8_t { INVALID = 0, OBJECT, ARRAY, STRING, NUMBER, BOOLEAN, NULL_VALUE };

    JsonValue() = default;
    JsonValue(const char* begin, const char* end, Type type) : begin_(begin), end_(end), type_(type) {}

    Type type() const { return type_; }
    bool valid() const { return type_ != Type::INVALID; }
    bool isObject() const { return type_ == Type::OBJECT; }
    bool isArray() const { return type_ == Type::ARRAY; }
    bool isString() const { return type_ == Type::STRING; }
    bool isNumber() const { return type_ == Type::NUMBER; }
    bool isBoolean() const { return type_ == Type::BOOLEAN; }

    /// Look up a member of an object.  Returns an invalid value when absent.
    JsonValue member(const char* key) const;

    /// Array element by index; invalid when out of range.
    JsonValue element(int index) const;

    /// Array length (0 for non-arrays).
    int size() const;

    bool asBool(bool fallback = false) const;
    float asFloat(float fallback = 0.0f) const;
    int asInt(int fallback = 0) const;
    uint64_t asUint64(uint64_t fallback = 0) const;

    /// Copy the string payload into `out` (always NUL terminated).
    bool asString(char* out, std::size_t out_size) const;

    /// Length of the string payload (without the terminator).
    std::size_t stringLength() const;

    const char* raw() const { return begin_; }

private:
    const char* begin_{nullptr};
    const char* end_{nullptr};
    Type type_{Type::INVALID};
};

/// Parse a complete JSON document.  Trailing whitespace is allowed, trailing
/// garbage is not.  Returns an invalid value on syntax errors.
JsonValue jsonParse(const char* text);

/// Parse a complete JSON document of known length (NUL termination not required).
JsonValue jsonParse(const char* text, std::size_t length);

/// Skip whitespace inside a region; exposed for the streaming line reader.
const char* jsonSkipWhitespace(const char* cursor, const char* end);

}  // namespace nav
