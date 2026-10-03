// Console command parsing.
//
// This lives in the navigation core (not the firmware) because it is pure text
// logic with no hardware dependency, which makes it unit-testable on the host
// alongside the rest of the core.  The firmware feeds it console lines and acts
// on the result; the simulator feeds it scripted operator input.
//
// The parser is deliberately permissive about the "CMD:" prefix so position JSON
// and operator commands can share one serial link without ambiguity.
#pragma once

#include <cstddef>

namespace nav {

/// Command kinds understood on the console.
enum class CommandKind {
    NONE = 0,
    HELP,
    STATUS,
    GRAPH,
    ROUTE,
    POSITION,
    DESTINATION,
    START,
    STOP,
    CANCEL,
    ESTOP,
    CLEAR_ESTOP,
    REPLAN,
    DEBUG_ON,
    DEBUG_OFF,
    // Stepper bench control (NEMA 17 + A4988): position, direction and RPM.
    RPM,
    MOVE_REVOLUTIONS,
    ZERO_POSITION,
    STEPPER_STATUS,
    RESUME,
    UNKNOWN,
};

struct ParsedCommand {
    CommandKind kind{CommandKind::NONE};
    /// Integer argument (node id), or -1 when absent/invalid.
    int argument{-1};
    /// First numeric argument as a float, for commands that take a value such as
    /// an RPM or a revolution count.  NaN when absent/invalid.
    float arg1{0.0f};
    bool has_arg1{false};
    /// Second numeric argument (the right-hand axis).  NaN when absent.
    float arg2{0.0f};
    bool has_arg2{false};
    /// The command verb and argument as received (truncated, NUL terminated).
    char raw[64]{};
};

/// Parse one console line.
///
/// Returns false when the line is *not* a command and should be offered to the
/// position parser instead: empty lines, JSON objects and "POS:" lines.
bool parseCommand(const char* line, ParsedCommand& out);

/// Parse a line of known length (NUL termination not required).
bool parseCommand(const char* line, std::size_t length, ParsedCommand& out);

/// Human readable name of a command kind.
const char* commandName(CommandKind kind);

/// Help text for the `help` command.
const char* commandHelpText();

}  // namespace nav
