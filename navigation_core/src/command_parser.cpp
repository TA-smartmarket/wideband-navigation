#include "navigation/command_parser.hpp"

#include <cstring>

#include "navigation/types.hpp"

namespace nav {

namespace {

/// Case-insensitive ASCII comparison, bounded by `length` characters.
bool equalsIgnoreCase(const char* a, const char* b, std::size_t length) {
    for (std::size_t i = 0; i < length; ++i) {
        char ca = a[i];
        char cb = b[i];
        if (ca >= 'A' && ca <= 'Z') ca = static_cast<char>(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = static_cast<char>(cb - 'A' + 'a');
        if (ca != cb) {
            return false;
        }
        if (ca == '\0') {
            return true;
        }
    }
    return b[length] == '\0';
}

/// Parse a decimal number (optionally signed, with a fractional part) that must
/// consume the whole remaining string.
bool parseNumber(const char* text, float& out) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    const char* cursor = text;
    if (*cursor == '-' || *cursor == '+') {
        ++cursor;
    }
    bool has_digit = false;
    while ((*cursor >= '0' && *cursor <= '9') || *cursor == '.') {
        if (*cursor != '.') {
            has_digit = true;
        }
        ++cursor;
    }
    while (*cursor == ' ' || *cursor == '\t') {
        ++cursor;
    }
    if (!has_digit || *cursor != '\0') {
        return false;
    }
    char buffer[24];
    std::size_t length = 0;
    for (const char* p = text; *p != '\0' && length + 1 < sizeof(buffer); ++p) {
        buffer[length++] = *p;
    }
    buffer[length] = '\0';
    out = static_cast<float>(std::strtod(buffer, nullptr));
    return true;
}

/// Parse a decimal integer that must consume the whole remaining string.
bool parseInteger(const char* text, int& out) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    const char* cursor = text;
    bool negative = false;
    if (*cursor == '-' || *cursor == '+') {
        negative = (*cursor == '-');
        ++cursor;
    }
    if (*cursor < '0' || *cursor > '9') {
        return false;
    }
    long value = 0;
    while (*cursor >= '0' && *cursor <= '9') {
        value = value * 10 + (*cursor - '0');
        if (value > 1000000L) {
            return false;  // reject absurd node ids early
        }
        ++cursor;
    }
    while (*cursor == ' ' || *cursor == '\t') {
        ++cursor;
    }
    if (*cursor != '\0') {
        return false;
    }
    out = static_cast<int>(negative ? -value : value);
    return true;
}

}  // namespace

const char* commandName(CommandKind kind) {
    switch (kind) {
        case CommandKind::HELP: return "help";
        case CommandKind::STATUS: return "status";
        case CommandKind::GRAPH: return "graph";
        case CommandKind::ROUTE: return "route";
        case CommandKind::POSITION: return "position";
        case CommandKind::DESTINATION: return "destination";
        case CommandKind::START: return "start";
        case CommandKind::STOP: return "stop";
        case CommandKind::CANCEL: return "cancel";
        case CommandKind::ESTOP: return "estop";
        case CommandKind::CLEAR_ESTOP: return "clear_estop";
        case CommandKind::REPLAN: return "replan";
        case CommandKind::DEBUG_ON: return "debug on";
        case CommandKind::DEBUG_OFF: return "debug off";
        case CommandKind::RPM: return "rpm";
        case CommandKind::MOVE_REVOLUTIONS: return "move";
        case CommandKind::ZERO_POSITION: return "zero";
        case CommandKind::STEPPER_STATUS: return "stepper";
        case CommandKind::RESUME: return "resume";
        case CommandKind::UNKNOWN: return "unknown";
        default: return "none";
    }
}

const char* commandHelpText() {
    return
        "Commands (prefix with CMD: when sharing the link with position JSON)\n"
        "  help            show this text\n"
        "  status          human readable navigation status\n"
        "  graph           graph summary (nodes, edges)\n"
        "  route           planned route and distance\n"
        "  position        latest accepted UWB position\n"
        "  destination <n> set destination node\n"
        "  start           start/resume navigation to the held destination\n"
        "  stop            stop and keep the destination\n"
        "  cancel          cancel navigation and clear the destination\n"
        "  estop           software emergency stop\n"
        "  clear_estop     clear the emergency stop (does not resume motion)\n"
        "  replan          force a replan on the next cycle\n"
        "  debug on|off    enable/disable verbose logging\n"
        "\n"
        "Stepper bench control (NEMA 17 + A4988):\n"
        "  rpm <l> [r]     run the axes at a signed RPM (r defaults to l)\n"
        "  move <l> [r]    turn the axes by a number of revolutions, then stop\n"
        "  zero            declare the current position as step 0\n"
        "  stepper         report position (steps/rev), RPM, rate and direction\n"
        "  resume          return to navigation velocity mode\n";
}

bool parseCommand(const char* line, std::size_t length, ParsedCommand& out) {
    out = ParsedCommand{};
    if (line == nullptr || length == 0) {
        return false;
    }

    // Work on a bounded copy so callers may pass a non-terminated buffer.
    char buffer[160];
    std::size_t copied = 0;
    for (; copied < length && copied + 1 < sizeof(buffer); ++copied) {
        buffer[copied] = line[copied];
    }
    buffer[copied] = '\0';
    return parseCommand(buffer, out);
}

bool parseCommand(const char* line, ParsedCommand& out) {
    out = ParsedCommand{};
    if (line == nullptr) {
        return false;
    }

    const char* cursor = line;
    while (*cursor == ' ' || *cursor == '\t') {
        ++cursor;
    }
    if (*cursor == '\0') {
        return false;
    }

    // A JSON position line is never a command.
    if (*cursor == '{') {
        return false;
    }

    if (std::strncmp(cursor, "CMD:", 4) == 0) {
        cursor += 4;
        while (*cursor == ' ' || *cursor == '\t') {
            ++cursor;
        }
    } else if (std::strncmp(cursor, "POS:", 4) == 0) {
        return false;
    }

    copyBounded(out.raw, sizeof(out.raw), cursor);

    // Split the verb from an optional argument.
    char verb[32];
    std::size_t index = 0;
    while (cursor[index] != '\0' && cursor[index] != ' ' && cursor[index] != '\t' &&
           index + 1 < sizeof(verb)) {
        verb[index] = cursor[index];
        ++index;
    }
    verb[index] = '\0';

    const char* rest = cursor + index;
    while (*rest == ' ' || *rest == '\t') {
        ++rest;
    }
    int argument = -1;
    if (*rest != '\0') {
        int parsed = 0;
        if (parseInteger(rest, parsed)) {
            argument = parsed;
        }
        // Numeric form for value commands: one or two space separated numbers.
        const char* second = rest;
        float first_value = 0.0f;
        // Take the first token.
        char token[24];
        std::size_t n = 0;
        while (second[n] != '\0' && second[n] != ' ' && second[n] != '\t' &&
               n + 1 < sizeof(token)) {
            token[n] = second[n];
            ++n;
        }
        token[n] = '\0';
        if (parseNumber(token, first_value)) {
            out.arg1 = first_value;
            out.has_arg1 = true;
            const char* third = second + n;
            while (*third == ' ' || *third == '\t') {
                ++third;
            }
            if (*third != '\0') {
                float second_value = 0.0f;
                if (parseNumber(third, second_value)) {
                    out.arg2 = second_value;
                    out.has_arg2 = true;
                }
            }
        }
    }
    out.argument = argument;

    if (equalsIgnoreCase(verb, "help", 4)) {
        out.kind = CommandKind::HELP;
    } else if (equalsIgnoreCase(verb, "status", 6)) {
        out.kind = CommandKind::STATUS;
    } else if (equalsIgnoreCase(verb, "graph", 5)) {
        out.kind = CommandKind::GRAPH;
    } else if (equalsIgnoreCase(verb, "route", 5)) {
        out.kind = CommandKind::ROUTE;
    } else if (equalsIgnoreCase(verb, "position", 8)) {
        out.kind = CommandKind::POSITION;
    } else if (equalsIgnoreCase(verb, "destination", 11) || equalsIgnoreCase(verb, "dest", 4)) {
        out.kind = CommandKind::DESTINATION;
    } else if (equalsIgnoreCase(verb, "start", 5)) {
        out.kind = CommandKind::START;
    } else if (equalsIgnoreCase(verb, "stop", 4)) {
        out.kind = CommandKind::STOP;
    } else if (equalsIgnoreCase(verb, "cancel", 6)) {
        out.kind = CommandKind::CANCEL;
    } else if (equalsIgnoreCase(verb, "estop", 5)) {
        out.kind = CommandKind::ESTOP;
    } else if (equalsIgnoreCase(verb, "clear_estop", 11)) {
        out.kind = CommandKind::CLEAR_ESTOP;
    } else if (equalsIgnoreCase(verb, "replan", 6)) {
        out.kind = CommandKind::REPLAN;
    } else if (equalsIgnoreCase(verb, "rpm", 3)) {
        out.kind = CommandKind::RPM;
    } else if (equalsIgnoreCase(verb, "move", 4) || equalsIgnoreCase(verb, "turn", 4)) {
        out.kind = CommandKind::MOVE_REVOLUTIONS;
    } else if (equalsIgnoreCase(verb, "zero", 4)) {
        out.kind = CommandKind::ZERO_POSITION;
    } else if (equalsIgnoreCase(verb, "stepper", 7)) {
        out.kind = CommandKind::STEPPER_STATUS;
    } else if (equalsIgnoreCase(verb, "resume", 6)) {
        out.kind = CommandKind::RESUME;
    } else if (equalsIgnoreCase(verb, "debug", 5)) {
        if (equalsIgnoreCase(rest, "on", 2)) {
            out.kind = CommandKind::DEBUG_ON;
        } else if (equalsIgnoreCase(rest, "off", 3)) {
            out.kind = CommandKind::DEBUG_OFF;
        } else {
            out.kind = CommandKind::UNKNOWN;
        }
    } else {
        out.kind = CommandKind::UNKNOWN;
    }
    return true;
}

}  // namespace nav
