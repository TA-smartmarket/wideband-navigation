// Unit tests: console command parsing and the shared route fixtures.
//
// The parser is the boundary between an operator (or a host tool) and the
// navigation state machine, so its behaviour is tested exhaustively: a
// misparsed "estop" or a position line mistaken for a command would be a safety
// problem, not a cosmetic one.
#include <unity.h>

#include <cstdio>
#include <cstring>

#include "navigation/command_parser.hpp"
#include "navigation/config_loader.hpp"
#include "navigation/dijkstra.hpp"
#include "navigation/json_parser.hpp"

using namespace nav;

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Command parsing
// ---------------------------------------------------------------------------

static void test_position_json_is_not_a_command() {
    ParsedCommand command;
    const char* line =
        "{\"schema_version\":1,\"trolley_id\":\"TROLLEY_01\",\"frame_id\":\"smart_market_map\","
        "\"timestamp_ms\":1000,\"position\":{\"x_m\":1.2,\"y_m\":2.3},\"quality\":0.95,"
        "\"valid\":true}";
    TEST_ASSERT_FALSE(parseCommand(line, command));
    TEST_ASSERT_FALSE(parseCommand("POS:{\"schema_version\":1}", command));
    TEST_ASSERT_FALSE(parseCommand("", command));
    TEST_ASSERT_FALSE(parseCommand("   ", command));
    TEST_ASSERT_FALSE(parseCommand(nullptr, command));
}

static void test_simple_commands() {
    ParsedCommand command;
    struct Case {
        const char* text;
        CommandKind kind;
    };
    const Case cases[] = {
        {"help", CommandKind::HELP},
        {"status", CommandKind::STATUS},
        {"graph", CommandKind::GRAPH},
        {"route", CommandKind::ROUTE},
        {"position", CommandKind::POSITION},
        {"start", CommandKind::START},
        {"stop", CommandKind::STOP},
        {"cancel", CommandKind::CANCEL},
        {"estop", CommandKind::ESTOP},
        {"clear_estop", CommandKind::CLEAR_ESTOP},
        {"replan", CommandKind::REPLAN},
        // Case-insensitive and tolerant of surrounding whitespace.
        {"STATUS", CommandKind::STATUS},
        {"  Estop  ", CommandKind::ESTOP},
        {"Help", CommandKind::HELP},
    };
    for (const Case& item : cases) {
        TEST_ASSERT_TRUE(parseCommand(item.text, command));
        TEST_ASSERT_EQUAL_INT_MESSAGE(static_cast<int>(item.kind), static_cast<int>(command.kind),
                                      item.text);
    }
}

static void test_destination_with_argument() {
    ParsedCommand command;
    TEST_ASSERT_TRUE(parseCommand("destination 11", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::DESTINATION), static_cast<int>(command.kind));
    TEST_ASSERT_EQUAL_INT(11, command.argument);

    TEST_ASSERT_TRUE(parseCommand("dest 3", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::DESTINATION), static_cast<int>(command.kind));
    TEST_ASSERT_EQUAL_INT(3, command.argument);

    // Missing or malformed argument: the kind is still recognised, the argument
    // stays -1 so the caller can reject it without guessing.
    TEST_ASSERT_TRUE(parseCommand("destination", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::DESTINATION), static_cast<int>(command.kind));
    TEST_ASSERT_EQUAL_INT(-1, command.argument);

    TEST_ASSERT_TRUE(parseCommand("destination abc", command));
    TEST_ASSERT_EQUAL_INT(-1, command.argument);

    TEST_ASSERT_TRUE(parseCommand("destination 7 extra", command));
    TEST_ASSERT_EQUAL_INT(-1, command.argument);
}

static void test_cmd_prefix_is_accepted() {
    ParsedCommand command;
    TEST_ASSERT_TRUE(parseCommand("CMD:destination 11", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::DESTINATION), static_cast<int>(command.kind));
    TEST_ASSERT_EQUAL_INT(11, command.argument);

    TEST_ASSERT_TRUE(parseCommand("CMD: estop", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::ESTOP), static_cast<int>(command.kind));

    TEST_ASSERT_TRUE(parseCommand("CMD:status", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::STATUS), static_cast<int>(command.kind));
}

static void test_debug_command() {
    ParsedCommand command;
    TEST_ASSERT_TRUE(parseCommand("debug on", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::DEBUG_ON), static_cast<int>(command.kind));
    TEST_ASSERT_TRUE(parseCommand("debug off", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::DEBUG_OFF), static_cast<int>(command.kind));
    // Bare "debug" is recognised as a command but not a valid sub-command.
    TEST_ASSERT_TRUE(parseCommand("debug", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::UNKNOWN), static_cast<int>(command.kind));
}

static void test_unknown_command_is_reported_not_ignored() {
    ParsedCommand command;
    TEST_ASSERT_TRUE(parseCommand("frobnicate 3", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::UNKNOWN), static_cast<int>(command.kind));
    TEST_ASSERT_TRUE(std::strlen(command.raw) > 0);
}

static void test_parser_handles_non_terminated_buffer() {
    ParsedCommand command;
    const char* text = "CMD:destination 11\nEXTRA";
    // Only the first 18 characters ("CMD:destination 11") are a line.
    TEST_ASSERT_TRUE(parseCommand(text, 18, command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::DESTINATION), static_cast<int>(command.kind));
    TEST_ASSERT_EQUAL_INT(11, command.argument);
}

static void test_stepper_commands_parse_numeric_arguments() {
    ParsedCommand command;

    // RPM with one value: both axes use the same value.
    TEST_ASSERT_TRUE(parseCommand("rpm 60", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::RPM), static_cast<int>(command.kind));
    TEST_ASSERT_TRUE(command.has_arg1);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 60.0f, command.arg1);
    TEST_ASSERT_FALSE(command.has_arg2);

    // RPM with two values, including a signed and fractional one.
    TEST_ASSERT_TRUE(parseCommand("CMD:rpm -12.5 30", command));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -12.5f, command.arg1);
    TEST_ASSERT_TRUE(command.has_arg2);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 30.0f, command.arg2);

    // Revolution count.
    TEST_ASSERT_TRUE(parseCommand("move 2.5", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::MOVE_REVOLUTIONS),
                          static_cast<int>(command.kind));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.5f, command.arg1);
    TEST_ASSERT_TRUE(parseCommand("turn -1", command));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, command.arg1);

    // Position reset, status and resume.
    TEST_ASSERT_TRUE(parseCommand("zero", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::ZERO_POSITION),
                          static_cast<int>(command.kind));
    TEST_ASSERT_TRUE(parseCommand("stepper", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::STEPPER_STATUS),
                          static_cast<int>(command.kind));
    TEST_ASSERT_TRUE(parseCommand("resume", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::RESUME), static_cast<int>(command.kind));

    // A malformed value must not be reported as valid.
    TEST_ASSERT_TRUE(parseCommand("rpm abc", command));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandKind::RPM), static_cast<int>(command.kind));
    TEST_ASSERT_FALSE(command.has_arg1);

    // The help text documents them.
    TEST_ASSERT_TRUE(std::strstr(commandHelpText(), "rpm") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(commandHelpText(), "move") != nullptr);
}

static void test_command_names_are_stable() {
    TEST_ASSERT_TRUE(boundedEquals(commandName(CommandKind::ESTOP), "estop"));
    TEST_ASSERT_TRUE(boundedEquals(commandName(CommandKind::CLEAR_ESTOP), "clear_estop"));
    TEST_ASSERT_TRUE(std::strstr(commandHelpText(), "estop") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(commandHelpText(), "destination") != nullptr);
}

// ---------------------------------------------------------------------------
// Shared route fixtures (test_data/route_fixtures.json)
// ---------------------------------------------------------------------------

static void test_shared_route_fixture_matches_core() {
    // The fixture is generated by scripts/generate_route_fixtures.py from the
    // C++ core itself, and the Python simulator reads the same file, so this
    // test pins the contract that keeps both languages in agreement.
    std::FILE* handle = std::fopen("test_data/route_fixtures.json", "rb");
    if (handle == nullptr) {
        // Also try the path used when the tests run from the repository root.
        handle = std::fopen("../test_data/route_fixtures.json", "rb");
    }
    if (handle == nullptr) {
        TEST_IGNORE_MESSAGE("test_data/route_fixtures.json not found; run "
                            "scripts/generate_route_fixtures.py");
    }
    char buffer[4096];
    const std::size_t read = std::fread(buffer, 1, sizeof(buffer) - 1, handle);
    std::fclose(handle);
    buffer[read] = '\0';

    const JsonValue root = jsonParse(buffer, read);
    TEST_ASSERT_TRUE(root.isObject());
    const JsonValue fixtures = root.member("fixtures");
    TEST_ASSERT_TRUE(fixtures.isArray());
    TEST_ASSERT_TRUE(fixtures.size() > 0);

    char graph_path[256];
    TEST_ASSERT_TRUE(root.member("graph_path").asString(graph_path, sizeof(graph_path)));
    std::FILE* graph_handle = std::fopen(graph_path, "rb");
    if (graph_handle == nullptr) {
        graph_handle = std::fopen("../config/graph.json", "rb");
    }
    TEST_ASSERT_TRUE_MESSAGE(graph_handle != nullptr, "cannot open the graph for the fixtures");
    char graph_buffer[8192];
    const std::size_t graph_read = std::fread(graph_buffer, 1, sizeof(graph_buffer) - 1, graph_handle);
    std::fclose(graph_handle);
    graph_buffer[graph_read] = '\0';

    Graph graph;
    char error[256] = {};
    TEST_ASSERT_TRUE(loadGraph(graph_buffer, graph, error, sizeof(error)));

    for (int i = 0; i < fixtures.size(); ++i) {
        const JsonValue fixture = fixtures.element(i);
        const int start_node = fixture.member("start_node").asInt(-1);
        const int destination = fixture.member("destination_node").asInt(-1);
        TEST_ASSERT_TRUE(start_node >= 0);
        TEST_ASSERT_TRUE(destination >= 0);

        const PathResult result = dijkstra(graph, start_node, destination);
        TEST_ASSERT_TRUE(result.success);

        const JsonValue route = fixture.member("route");
        TEST_ASSERT_TRUE(route.isArray());
        TEST_ASSERT_EQUAL_INT(route.size(), result.node_count);
        for (int step = 0; step < route.size(); ++step) {
            TEST_ASSERT_EQUAL_INT(route.element(step).asInt(-1), result.node_ids[step]);
        }
        TEST_ASSERT_FLOAT_WITHIN(0.01f, fixture.member("distance_m").asFloat(0.0f),
                                 result.total_distance_m);
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_position_json_is_not_a_command);
    RUN_TEST(test_simple_commands);
    RUN_TEST(test_destination_with_argument);
    RUN_TEST(test_cmd_prefix_is_accepted);
    RUN_TEST(test_debug_command);
    RUN_TEST(test_unknown_command_is_reported_not_ignored);
    RUN_TEST(test_parser_handles_non_terminated_buffer);
    RUN_TEST(test_stepper_commands_parse_numeric_arguments);
    RUN_TEST(test_command_names_are_stable);
    RUN_TEST(test_shared_route_fixture_matches_core);
    return UNITY_END();
}
