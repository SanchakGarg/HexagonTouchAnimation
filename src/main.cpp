#include <Arduino.h>
#include <PCF8574.h>
#include <WiFi.h>
#include <FastLED.h>
#include <vector>
#include <unordered_map>
#include <queue>
#include <set>
#include <tuple>
#include <freertos/FreeRTOS.h>

#define LED1_PIN 32
#define LED2_PIN 25
#define LED3_PIN 27

#define NUM_LEDS1 150
#define NUM_LEDS2 120
#define NUM_LEDS3 180

#define SEGMENT_SIZE 15
#define FLOW_DELAY 2 // milliseconds between lighting each LED
#define DEBOUNCE_DELAY 1

CRGB leds1[NUM_LEDS1];
CRGB leds2[NUM_LEDS2];
CRGB leds3[NUM_LEDS3];

struct Connection
{
    int toNode;
    String ledStrip;
    int segment;
    int direction;
};

using NodeConnections = std::vector<Connection>;
std::unordered_map<int, NodeConnections> graph;

void initializeGraph()
{
    graph = {
        {0, {{1, "leds1", 3, -1}, {3, "leds1", 4, 1}}},
        {1, {{0, "leds1", 3, 1}, {2, "leds1", 6, -1}, {7, "leds1", 7, 1}, {8, "leds1", 2, -1}}},
        {2, {{1, "leds1", 6, 1}, {3, "leds1", 5, -1}, {6, "leds2", 0, 1}}},
        {3, {{0, "leds1", 4, -1}, {2, "leds1", 5, 1}, {4, "leds3", 1, 1}, {5, "leds3", 0, -1}}},
        {4, {{3, "leds3", 1, -1}, {12, "leds3", 2, 1}}},
        {5, {{3, "leds3", 0, 1}, {12, "leds2", 5, -1}, {6, "leds2", 6, 1}}},
        {6, {{2, "leds2", 0, -1}, {5, "leds2", 6, -1}, {7, "leds1", 8, -1}, {10, "leds2", 7, 1}, {11, "leds1", 9, 1}, {15, "leds2", 1, 1}}},
        {7, {{1, "leds1", 7, -1}, {6, "leds1", 8, 1}, {9, "leds1", 0, 1}}},
        {8, {{1, "leds1", 2, 1}, {9, "leds1", 1, -1}}},
        {9, {{7, "leds1", 0, -1}, {8, "leds1", 1, 1}, {10, "leds3", 9, 1}, {17, "leds3", 8, -1}}},
        {10, {{6, "leds2", 7, -1}, {16, "leds3", 10, 1}, {9, "leds3", 9, -1}}},
        {11, {{6, "leds1", 9, -1}, {12, "leds2", 4, 1}, {14, "leds2", 3, -1}}},
        {12, {{4, "leds3", 2, -1}, {5, "leds2", 5, 1}, {11, "leds2", 4, -1}, {13, "leds3", 3, 1}}},
        {13, {{12, "leds3", 3, -1}, {14, "leds3", 4, 1}}},
        {14, {{13, "leds3", 4, -1}, {18, "leds3", 5, 1}, {11, "leds2", 3, 1}, {15, "leds2", 2, -1}}},
        {15, {{6, "leds2", 1, -1}, {14, "leds2", 2, 1}, {16, "leds3", 11, -1}}},
        {16, {{10, "leds3", 10, -1}, {15, "leds3", 11, 1}, {17, "leds3", 7, 1}, {18, "leds3", 6, -1}}},
        {17, {{9, "leds3", 8, 1}, {16, "leds3", 7, -1}}},
        {18, {{16, "leds3", 6, 1}, {14, "leds3", 5, -1}}}};
}

void setLED(String strip, int segment, int ledIndex, CRGB color)
{
    int actualIndex = segment * SEGMENT_SIZE + ledIndex;
    if (strip == "leds1" && actualIndex < NUM_LEDS1)
    {
        leds1[actualIndex] = color;
    }
    else if (strip == "leds2" && actualIndex < NUM_LEDS2)
    {
        leds2[actualIndex] = color;
    }
    else if (strip == "leds3" && actualIndex < NUM_LEDS3)
    {
        leds3[actualIndex] = color;
    }
}

void lightSegments(const std::vector<Connection> &connections, CRGB color)
{
    const int trailLength = 1;

    for (int i = 0; i < SEGMENT_SIZE + trailLength; i++)
    {
        for (const auto &conn : connections)
        {
            for (int j = 0; j < trailLength + 1; j++)
            {
                int ledIndex = conn.direction == 1 ? i - j : (SEGMENT_SIZE - 1 - (i - j));

                if (ledIndex >= 0 && ledIndex < SEGMENT_SIZE)
                {
                    CRGB pixelColor = (j == 0) ? color : CRGB::Black;
                    setLED(conn.ledStrip, conn.segment, ledIndex, pixelColor);
                }
            }
        }
        FastLED.show();
        delay(FLOW_DELAY);
    }
}
void traverseFromNode(int startNode)
{
    std::queue<int> nodeQueue;
    std::set<std::tuple<int, int, String>> visitedConnections;
    CRGB animationColor = CRGB(random(256), random(256), random(256));

    nodeQueue.push(startNode);

    while (!nodeQueue.empty())
    {
        int levelSize = nodeQueue.size();
        std::vector<Connection> currentLevelConnections;

        for (int i = 0; i < levelSize; ++i)
        {
            int currentNode = nodeQueue.front();
            nodeQueue.pop();

            for (const auto &conn : graph[currentNode])
            {
                auto connectionKey = std::make_tuple(currentNode, conn.toNode, conn.ledStrip);
                auto reverseConnectionKey = std::make_tuple(conn.toNode, currentNode, conn.ledStrip);

                if (visitedConnections.find(connectionKey) == visitedConnections.end() &&
                    visitedConnections.find(reverseConnectionKey) == visitedConnections.end())
                {
                    currentLevelConnections.push_back(conn);
                    nodeQueue.push(conn.toNode);
                    visitedConnections.insert(connectionKey);
                    visitedConnections.insert(reverseConnectionKey);
                }
            }
        }

        if (!currentLevelConnections.empty())
        {
            lightSegments(currentLevelConnections, animationColor);
            // delay(500); // Add a delay between levels for visibility
        }
    }
}

void resetLEDs()
{
    fill_solid(leds1, NUM_LEDS1, CRGB::Black);
    fill_solid(leds2, NUM_LEDS2, CRGB::Black);
    fill_solid(leds3, NUM_LEDS3, CRGB::Black);
    FastLED.show();
}

PCF8574 board1(0x20, 21, 22);
PCF8574 board2(0x21, 21, 22);
PCF8574 board3(0x22, 21, 22);
const int single_pin = 26;

int sensorStatus[19];
unsigned long lastDebounceTime[19]; // To store the last time a sensor was toggled
bool lastSensorState[19];

void sendTouchInput()
{
    int sensorValues[19] = {
        digitalRead(26),        // 0
        board3.digitalRead(P2), // 1
        board3.digitalRead(P4), // 2
        board1.digitalRead(P4), // 3
        board1.digitalRead(P2), // 4
        board1.digitalRead(P3), // 5
        board1.digitalRead(P1), // 6
        board3.digitalRead(P3), // 7
        board3.digitalRead(P0), // 8
        board3.digitalRead(P1), // 9
        board2.digitalRead(P7), // 10
        board2.digitalRead(P3), // 11
        board1.digitalRead(P0), // 12
        board2.digitalRead(P2), // 13
        board2.digitalRead(P1), // 14
        board2.digitalRead(P0), // 15
        board2.digitalRead(P5), // 16
        board2.digitalRead(P6), // 17
        board2.digitalRead(P4)  // 18
    };

    unsigned long currentTime = millis();

    for (int i = 0; i < 19; i++)
    {
        if (sensorValues[i] != lastSensorState[i])
        {
            lastDebounceTime[i] = currentTime;
        }

        if ((currentTime - lastDebounceTime[i]) > DEBOUNCE_DELAY)
        {
            if (sensorValues[i] != sensorStatus[i])
            {
                sensorStatus[i] = sensorValues[i];

                if (sensorValues[i] == HIGH)
                {
                    traverseFromNode(i);
                }

            }
            
        }

        lastSensorState[i] = sensorValues[i];
        delay(1);
    }
}

void setup()
{   
    FastLED.addLeds<NEOPIXEL, LED1_PIN>(leds1, NUM_LEDS1);
    FastLED.addLeds<NEOPIXEL, LED2_PIN>(leds2, NUM_LEDS2);
    FastLED.addLeds<NEOPIXEL, LED3_PIN>(leds3, NUM_LEDS3);

    randomSeed(analogRead(0)); // Initialize random number generator

    initializeGraph();
    Serial.begin(115200);

    // pin stuff
    board1.pinMode(P0, INPUT);
    board1.pinMode(P1, INPUT);
    board1.pinMode(P2, INPUT);
    board1.pinMode(P3, INPUT);
    board1.pinMode(P4, INPUT);
    board1.begin();

    board2.pinMode(P0, INPUT);
    board2.pinMode(P1, INPUT);
    board2.pinMode(P2, INPUT);
    board2.pinMode(P3, INPUT);
    board2.pinMode(P4, INPUT);
    board2.pinMode(P5, INPUT); 
    board2.pinMode(P6, INPUT);
    board2.pinMode(P7, INPUT);
    board2.begin();

    board3.pinMode(P0, INPUT);
    board3.pinMode(P1, INPUT);
    board3.pinMode(P2, INPUT);
    board3.pinMode(P3, INPUT);
    board3.pinMode(P4, INPUT);
    board3.begin();

    pinMode(single_pin, INPUT);
}

void loop()
{
    sendTouchInput();
}