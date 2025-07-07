// Declare used libraries
#include <Arduino.h>
#include <Wire.h>
#include "RTClib.h"
#include <TimerOne.h>
#include <avr/pgmspace.h>

//Declare hardware pins
RTC_DS3231 rtc;
int clearpin = 2;
int Aserialdata = 3;
int Ashiftclock = 4;
int Alatchclock = 5;
int Bserialdata = 6;
int Bshiftclock = 7;
int Blatchclock = 8;
int starpin = 9;
int plateone = 14;
int platetwo = 15;
int platethree = 16;
int platefour = 17;

// Multiplexing archetecture variables
volatile int currentRow = 0;
int multiplextime = 0;             // Delay between multiplexing each row, initially used for testing
uint16_t tempRowData[11];
volatile uint16_t rowData[11];     // rowData holds information about which LEDs we want on from each row.
const uint16_t MINUTE_BITS_MASK = 0b1111100000000000; 
// In my hardware, all LEDs are controlled by a 16-digit binary number. 
// The five leftmost bits are my logo LED and the four corner LEDs. (1 = off, 0 = on)
// The remaining 11 bits are the 11 rows of LEDs. (1 = on, 0 = off)

// Determines the mode the word clock is in
int mode = 0;

// Touch sensor variables
unsigned long pressStart = 0;      // Time when plate press started
bool longPressHandled = false;     // True is plate is pressed for more than 3000 ms
int lastPlate = 0;                 // Plate previously held
int currentPlate = 0;              // Plate currently held
int shortPressedPlate = 0;         // Plate number of plate that was short pressed


// Function that clears all shift registers
void clear() {
    shiftOut(Bserialdata, Bshiftclock, LSBFIRST, 0B00011111);
    shiftOut(Bserialdata, Bshiftclock, LSBFIRST, 0B00000000);
    digitalWrite(Blatchclock, HIGH);
    digitalWrite(Blatchclock, LOW);
  }

// Function that latches the output of the shift registers
void latchOutput() {
    digitalWrite(Alatchclock, HIGH);
    digitalWrite(Blatchclock, HIGH);
    digitalWrite(Alatchclock, LOW);
    digitalWrite(Blatchclock, LOW);
}

// Function that poweres on a row based on its row number 'num'
void shiftOutMultiplex(int num) {
    shiftOut(Aserialdata, Ashiftclock, LSBFIRST, ~((0b10000000000 >> num) << 5));
    shiftOut(Aserialdata, Ashiftclock, LSBFIRST, ~((0b10000000000 >> num) >> 3));
}

// Core multiplexing function that cycles through each row at a time.
// All LEDs' anodes in a desired row is connected to 5V.
// Only desired LEDs' cathodes in that row is connected to GND.
// In this way, we can control which LEDs we want on in any row.
void multiplexInterruptHandler() {
    clear();  // clear display before switching row

    // Connects all anodes to 5V in a row
    shiftOutMultiplex(currentRow);

    // Get the 16-bit 'code' for this row from rowData. This 'code' tells us which LEDs in that row we want on (which cathodes to connect to GND).
    uint16_t value = rowData[currentRow];

    // Connects the desired cathodes to GND
    shiftOut(Bserialdata, Bshiftclock, MSBFIRST, (value >> 8) & 0xFF);  // upper 8 bits
    shiftOut(Bserialdata, Bshiftclock, MSBFIRST, value & 0xFF);         // lower 8 bits

    // Latches the output of the shift registers
    latchOutput();

    // Cycles through the rows in a loop
    currentRow++;
    if (currentRow >= 11) currentRow = 0;
}





// Mode 1 - Time

//Declaring mode-specific variables
static bool modeOneChanged = true;
int previousRoundedMin = 0;

// Locations on the LED matrix for words for the time display
#define WORD_AM             {0, 0b00000001100}   // row 0, bits 7-8
#define WORD_PM             {0, 0b00000000011}   // row 0, bits 9-10
#define WORD_IT             {0, 0b11000000000}   // row 0, bits 0-1
#define WORD_IS             {0, 0b00011000000}   // row 0, bits 3-4
#define WORD_QUARTER_MINUTE {1, 0b00111111100}   // row 1, bits 2-8
#define WORD_TWENTY_MINUTE  {2, 0b11111100000}   // row 2, bits 0-6
#define WORD_FIVE_MINUTE    {2, 0b00000011110}   // row 2, bits 6-9
#define WORD_HALF           {3, 0b11110000000}   // row 3, bits 0-4
#define WORD_TEN_MINUTE     {3, 0b00000111000}   // row 3, bits 5-7
#define WORD_TO             {3, 0b00000000011}   // row 3, bits 9-10
#define WORD_PAST           {4, 0b11110000000}   // row 4, bits 0-3
#define WORD_NINE_HOUR      {4, 0b00000001111}   // row 4, bits 7-10
#define WORD_ONE_HOUR       {5, 0b11100000000}   // row 5, bits 0-2
#define WORD_SIX_HOUR       {5, 0b00011100000}   // row 5, bits 3-5
#define WORD_THREE_HOUR     {5, 0b00000011111}   // row 5, bits 6-10
#define WORD_FOUR_HOUR      {6, 0b11110000000}   // row 6, bits 0-3
#define WORD_FIVE_HOUR      {6, 0b00001111000}   // row 6, bits 4-7
#define WORD_TWO_HOUR       {6, 0b00000000111}   // row 6, bits 8-10
#define WORD_EIGHT_HOUR     {7, 0b11111000000}   // row 7, bits 0-4
#define WORD_ELEVEN_HOUR    {7, 0b00000111111}   // row 7, bits 5-10
#define WORD_SEVEN_HOUR     {8, 0b11111000000}   // row 8, bits 0-4
#define WORD_TWELVE_HOUR    {8, 0b00000111111}   // row 8, bits 5-10
#define WORD_TEN_HOUR       {9, 0b11100000000}   // row 9, bits 0-2
#define WORD_OCLOCK         {9, 0b00000111111}  // row 10, bits 5-10

// Defining our own datatype to simplify the code
struct Word {
    byte row;
    uint16_t mask;
};

const Word WORDS_AM = WORD_AM;
const Word WORDS_PM = WORD_PM;
const Word WORDS_IT = WORD_IT;
const Word WORDS_IS = WORD_IS;
const Word WORDS_QUARTER_MINUTE = WORD_QUARTER_MINUTE;
const Word WORDS_TWENTY_MINUTE = WORD_TWENTY_MINUTE;
const Word WORDS_FIVE_MINUTE = WORD_FIVE_MINUTE;
const Word WORDS_HALF = WORD_HALF;
const Word WORDS_TEN_MINUTE = WORD_TEN_MINUTE;
const Word WORDS_TO = WORD_TO;
const Word WORDS_PAST = WORD_PAST;
const Word WORDS_NINE_HOUR = WORD_NINE_HOUR;
const Word WORDS_ONE_HOUR = WORD_ONE_HOUR;
const Word WORDS_SIX_HOUR = WORD_SIX_HOUR;
const Word WORDS_THREE_HOUR = WORD_THREE_HOUR;
const Word WORDS_FOUR_HOUR = WORD_FOUR_HOUR;
const Word WORDS_FIVE_HOUR = WORD_FIVE_HOUR;
const Word WORDS_TWO_HOUR = WORD_TWO_HOUR;
const Word WORDS_EIGHT_HOUR = WORD_EIGHT_HOUR;
const Word WORDS_ELEVEN_HOUR = WORD_ELEVEN_HOUR;
const Word WORDS_SEVEN_HOUR = WORD_SEVEN_HOUR;
const Word WORDS_TWELVE_HOUR = WORD_TWELVE_HOUR;
const Word WORDS_TEN_HOUR = WORD_TEN_HOUR;
const Word WORDS_OCLOCK = WORD_OCLOCK;

// Function to alter rowData with new information about which LEDs we want on from each row.
void showWord(const Word& w) {
    rowData[w.row] |= w.mask;
}

// Core function in this mode that caluculates what words to display based on RTC time
void time() {
    
    showWord(WORDS_IT);
    showWord(WORDS_IS);
    
    DateTime now = rtc.now();
    int minute = now.minute();
    int roundedMin = 5 * ((minute) / 5);

    int displayHour24 = now.hour();
    if (roundedMin >= 35) displayHour24 = (displayHour24 + 1) % 24;

    int displayHour = displayHour24 % 12;
    if (displayHour == 0) displayHour = 12;

    // If mode one has just been selected as the desired mode, reset rowData to all off to clear previous data
    if (modeOneChanged) {
        for (int i = 0; i < 11; i++) {
            rowData[i] = MINUTE_BITS_MASK;
        }
        modeOneChanged = false;
        previousRoundedMin = roundedMin;
    }

    // If the minute has changed, reset rowData to all off to clear previous data
    if (previousRoundedMin != roundedMin) {
        for (int i = 0; i < 11; i++) {
            rowData[i] = MINUTE_BITS_MASK;
        }
        previousRoundedMin = roundedMin;
    }

    if (displayHour24 <= 12) {
        showWord(WORDS_AM);
    } else {
        showWord(WORDS_PM);
    }

    // Display the correct word for the minute
    switch (roundedMin) {
        case 0:
            showWord(WORDS_OCLOCK);
        break;
        case 5:
            showWord(WORDS_FIVE_MINUTE);
            showWord(WORDS_PAST);
        break;
        case 10:
            showWord(WORDS_TEN_MINUTE);
            showWord(WORDS_PAST);
        break;
        case 15:
            showWord(WORDS_QUARTER_MINUTE);
            showWord(WORDS_PAST);
        break;
        case 20:
            showWord(WORDS_TWENTY_MINUTE);
            showWord(WORDS_PAST);
        break;
        case 25:
            showWord(WORDS_TWENTY_MINUTE);
            showWord(WORDS_FIVE_MINUTE);
            showWord(WORDS_PAST);
        break;
        case 30:
            showWord(WORDS_HALF);
            showWord(WORDS_PAST);
        break;
        case 35:
            showWord(WORDS_TWENTY_MINUTE);
            showWord(WORDS_FIVE_MINUTE);
            showWord(WORDS_TO);
        break;
        case 40:
            showWord(WORDS_TWENTY_MINUTE);
            showWord(WORDS_TO);
        break;
        case 45:
            showWord(WORDS_QUARTER_MINUTE);
            showWord(WORDS_TO);
        break;
        case 50:
            showWord(WORDS_TEN_MINUTE);
            showWord(WORDS_TO);
        break;
        case 55:
            showWord(WORDS_FIVE_MINUTE);
            showWord(WORDS_TO);
        break;
    }

    // Display the correct word for the hour
    switch (displayHour) {
        case 1:
            showWord(WORDS_ONE_HOUR);
        break;
        case 2:
            showWord(WORDS_TWO_HOUR);
        break;
        case 3:
            showWord(WORDS_THREE_HOUR);
        break;
        case 4:
            showWord(WORDS_FOUR_HOUR);
        break;
        case 5:
            showWord(WORDS_FIVE_HOUR);
        break;
        case 6:
            showWord(WORDS_SIX_HOUR);
        break;
        case 7:
            showWord(WORDS_SEVEN_HOUR);
        break;
        case 8:
            showWord(WORDS_EIGHT_HOUR);
        break;
        case 9:
            showWord(WORDS_NINE_HOUR);
        break;
        case 10:
            showWord(WORDS_TEN_HOUR);
        break;
        case 11:
            showWord(WORDS_ELEVEN_HOUR);
        break;
        case 12:
            showWord(WORDS_TWELVE_HOUR);
        break;
    }

    // If the minute is not a multiple of 5, display the correct LEDs for the leftover minutes
    int leftoverMins = minute % 5;
        for (int i = 0; i <= 11; i++) {
            if (leftoverMins == 1) rowData[i] &= ~0b0001000000000000;
            if (leftoverMins == 2) rowData[i] &= ~0b0011000000000000;
            if (leftoverMins == 3) rowData[i] &= ~0b0111000000000000;
            if (leftoverMins == 4) rowData[i] &= ~0b0111100000000000;
        }
}






// Mode 2 - Snake

// Snake direction
enum Direction { UP, DOWN, LEFT, RIGHT };

// Declaring our own data structure. Each LED has a x and y coordinate.
struct Point {
    byte x;
    byte y;
};

// Deplaring mode-specific variables
Point snakeArray[121];              // Holds the x and y coordinates of each LED in the snake
int snakeLength = 3;                // Initial length of the snake
Direction snakeDirection = UP;      // Initial direction of the snake
unsigned long lastMoveTime = 0;     // Time of last movement
unsigned long moveInterval = 750;   // Delay cooldown between direction changes through touch sensors
Point food;
bool foodExists = false;
static bool snakeInitialized = false;

// Function that spawns snake food if no food is present on the grid
void spawnFood() {
    if (foodExists) return;
    
    // Track all possible empty spots
    bool grid[11][11] = {false};
    
    // Mark snake positions as occupied
    for (int i = 0; i < snakeLength; i++) {
        grid[snakeArray[i].x][snakeArray[i].y] = true;
    }
    
    // Count empty spots
    int emptyCount = 0;
    for (byte y = 0; y < 11; y++) {
        for (byte x = 0; x < 11; x++) {
            if (!grid[x][y]) emptyCount++;
        }
    }
    
    if (emptyCount == 0) return; // No space left
    
    // Pick random empty spot
    int target = random(emptyCount);
    int count = 0;
    
    for (byte y = 0; y < 11; y++) {
        for (byte x = 0; x < 11; x++) {
            if (!grid[x][y]) {
                if (count == target) {
                    food = {x, y};
                    foodExists = true;
                    return;
                }
                count++;
            }
        }
    }
}

// Function that initializes the snake game
void initSnakeGame() {
    snakeLength = 3;
    snakeArray[0] = {5, 7};
    snakeArray[1] = {5, 8};
    snakeArray[2] = {5, 9};

    // Sets the rest of the snake to 255,255 (default / no data)
    for (int i = 3; i < 121; i++) {
        snakeArray[i] = {255, 255};
    }

    snakeDirection = UP;
    food = {5, 3};
    foodExists = true;
    lastMoveTime = millis();
}

// Function that renders the snake game by sending LED information to rowData
void drawSnakeToRowData() {
    // Clear all rows
    for (int i = 0; i < 11; i++) {
        rowData[i] = MINUTE_BITS_MASK;
    }

    // Draw snake (only up to snakeLength)
    for (int i = 0; i < snakeLength; i++) {
        if (snakeArray[i].x < 11 && snakeArray[i].y < 11) {
            rowData[snakeArray[i].y] |= (1 << (10 - snakeArray[i].x));
        }
    }

    // Draw snake food if present
    if (foodExists) {
        if (food.x < 11 && food.y < 11) {
            rowData[food.y] |= (1 << (10 - food.x));
        }
    }

}

// Core function in this mode that controls the snake game
void snake() {

    // If the snake game has not been initialized, initialize it
    if (!snakeInitialized) {
        initSnakeGame();
        snakeInitialized = true;
    }

    // If a touch sensor is short pressed, change the snake direction
    switch (shortPressedPlate) {
        case 1: if (snakeDirection != DOWN) snakeDirection = UP; break;
        case 2: if (snakeDirection != RIGHT) snakeDirection = LEFT; break;
        case 3: if (snakeDirection != LEFT) snakeDirection = RIGHT; break;
        case 4: if (snakeDirection != UP) snakeDirection = DOWN; break;
    }

    // If the last movement was too recent, return
    if ((millis() - lastMoveTime) < moveInterval) return;
    lastMoveTime = millis();

    // Get the new head of the snake
    Point newHead = snakeArray[0];
    switch (snakeDirection) {
        case UP:    newHead.y = (newHead.y == 0) ? 11 - 1 : newHead.y - 1; break;
        case DOWN:  newHead.y = (newHead.y + 1) % 11; break;
        case LEFT:  newHead.x = (newHead.x == 0) ? 11 - 1 : newHead.x - 1; break;
        case RIGHT: newHead.x = (newHead.x + 1) % 11; break;
    }

    // If the new head is the same position as the any snake segments, the snake has eaten itself
    for (int i = 0; i < snakeLength - 1; i++) {
        if (snakeArray[i].x == newHead.x && snakeArray[i].y == newHead.y) {
            initSnakeGame();
            return;
        }
    }

    // Shift the snake array along by 1 (moves the snake)
    for (int i = snakeLength; i > 0; i--) {
        snakeArray[i] = snakeArray[i - 1];
    }
    snakeArray[0] = newHead;

    // If the new head is the same position as the snake food, increase the snake length and remove the food
    if (newHead.x == food.x && newHead.y == food.y) {
        snakeLength++;
        foodExists = false;
        spawnFood();
    }

    // Render the snake game by sending LED information to rowData
    drawSnakeToRowData();

}





// Mode 3 - Tetris

//Declaring mode-specific variables
static uint16_t tetrisGrid[11] = {0}; // Holds the current state of the tetris grid
static int pieceIndex = 0;            // Index of the current tetromino
static int droppingTetromino_x = 0;   // x coordinate of the dropping tetromino
static int droppingTetromino_y = 0;   // y coordinate of the dropping tetromino
static unsigned long lastDrop = 0;    // Time of last drop
static bool tetrisActive = false;     // True if the tetris game is active
const unsigned long dropDelay = 1000; // Delay between moving the dropping tetromino by one row

// Defining our own datatype to simplify the code
typedef struct {
    byte shape[4][4];
    byte size;
} Tetromino;

// Declaring the tetrominoes
const Tetromino tetrominoes[7] = {
    {{{0,1,0,0},{0,1,0,0},{0,1,0,0},{0,1,0,0}}, 4}, // I
    {{{1,1,0,0},{1,1,0,0},{0,0,0,0},{0,0,0,0}}, 2}, // O
    {{{0,1,0,0},{1,1,1,0},{0,0,0,0},{0,0,0,0}}, 3}, // T
    {{{0,1,1,0},{1,1,0,0},{0,0,0,0},{0,0,0,0}}, 3}, // S
    {{{1,1,0,0},{0,1,1,0},{0,0,0,0},{0,0,0,0}}, 3}, // Z
    {{{1,0,0,0},{1,1,1,0},{0,0,0,0},{0,0,0,0}}, 3}, // J
    {{{0,0,1,0},{1,1,1,0},{0,0,0,0},{0,0,0,0}}, 3}  // L
};

// Check if piece can be placed at (x, y)
bool canPlaceTetromino(int x, int y, const Tetromino& t, uint16_t grid[]) {
    for (int row = 0; row < t.size; row++) {
        for (int col = 0; col < t.size; col++) {
            if (t.shape[row][col]) {
                int gx = x + col;
                int gy = y + row;
                if (gx < 0 || gx >= 11 || gy >= 11) return false;
                if (gy >= 0 && (grid[gy] & (1 << (10 - gx)))) return false;
            }
        }
    }
    return true;
}

// Lock piece into grid
void lockTetromino(int x, int y, const Tetromino& t, uint16_t grid[]) {
    for (int row = 0; row < t.size; row++) {
        for (int col = 0; col < t.size; col++) {
            if (t.shape[row][col]) {
                int gx = x + col;
                int gy = y + row;
                if (gx >= 0 && gx < 11 && gy >= 0 && gy < 11) {
                    grid[gy] |= (1 << (10 - gx));
                }
            }
        }
    }
}

// Clear completed lines
void clearTetrominoRow(uint16_t grid[]) {
    for (int y = 11-1; y >= 0; y--) {
        if ((grid[y] & 0x07FF) == 0x07FF) {  // all columns filled
            for (int j = y; j > 0; j--) {
                grid[j] = grid[j - 1];
            }
            grid[0] = 0;
            y++;
        }
    }
}

// Core function in this mode that controls the tetris game
void tetris() {
    static Tetromino activePiece;

    // Rotate piece 90 degrees clockwise
    auto rotatedTetromino = [](const Tetromino& t) -> Tetromino {
        Tetromino rotated;
        rotated.size = t.size;

        for (int y = 0; y < t.size; y++) {
            for (int x = 0; x < t.size; x++) {
                // Rotate 90° clockwise around center
                rotated.shape[y][x] = t.shape[t.size - 1 - x][y];
            }
        }

        return rotated;
    };

    // Spawn new piece
    auto spawnTetromino = [&]() {
        pieceIndex = random(7);
        activePiece = tetrominoes[pieceIndex];

        // Apply 0 to 3 random rotations
        int rotations = random(4);
        for (int i = 0; i < rotations; i++) {
            activePiece = rotatedTetromino(activePiece);
        }

        int topOffset = 0;
        bool found = false;
        for (int i = 0; i < activePiece.size && !found; i++) {
            for (int j = 0; j < activePiece.size; j++) {
                if (activePiece.shape[i][j]) {
                    topOffset = i;
                    found = true;
                    break;
                }
            }
        }

        // Calculate bounding box of rotated piece for X centering
        int minCol = activePiece.size, maxCol = -1;
        for (int y = 0; y < activePiece.size; y++) {
            for (int x = 0; x < activePiece.size; x++) {
                if (activePiece.shape[y][x]) {
                    if (x < minCol) minCol = x;
                    if (x > maxCol) maxCol = x;
                }
            }
        }
        int pieceWidth = maxCol - minCol + 1;
        int centerColumn = 5;  // Target center column

        int bias = 0;
        if (pieceWidth % 2 == 0) {
            bias = random(2);  // 0 = left bias, 1 = right bias
        }
        droppingTetromino_x = centerColumn - (pieceWidth / 2) - minCol + bias;

        droppingTetromino_y = -topOffset;

        if (!canPlaceTetromino(droppingTetromino_x, droppingTetromino_y, activePiece, tetrisGrid)) {
            for (int i = 0; i < 11; i++) tetrisGrid[i] = 0;
        }
    };

    if (!tetrisActive) {
        for (int i = 0; i < 11; i++) tetrisGrid[i] = 0;
        spawnTetromino();
        lastDrop = millis();
        tetrisActive = true;
    }

    // Touch sensor inputs
    if (shortPressedPlate == 1) { // Rotate
        Tetromino rotated = rotatedTetromino(activePiece);
        if (canPlaceTetromino(droppingTetromino_x, droppingTetromino_y, rotated, tetrisGrid)) {
            activePiece = rotated;
        }
    }
    else if (shortPressedPlate == 2) { // Move left
        if (canPlaceTetromino(droppingTetromino_x - 1, droppingTetromino_y, activePiece, tetrisGrid)) {
            droppingTetromino_x--;
        }
    }
    else if (shortPressedPlate == 3) { // Move right
        if (canPlaceTetromino(droppingTetromino_x + 1, droppingTetromino_y, activePiece, tetrisGrid)) {
            droppingTetromino_x++;
        }
    }
    else if (shortPressedPlate == 4) { // Hard drop
        while (canPlaceTetromino(droppingTetromino_x, droppingTetromino_y + 1, activePiece, tetrisGrid)) {
            droppingTetromino_y++;
        }
        lockTetromino(droppingTetromino_x, droppingTetromino_y, activePiece, tetrisGrid);
        clearTetrominoRow(tetrisGrid);
        spawnTetromino();
    }

    // Gravity drop timer
    if (millis() - lastDrop > dropDelay) {
        if (canPlaceTetromino(droppingTetromino_x, droppingTetromino_y + 1, activePiece, tetrisGrid)) {
            droppingTetromino_y++;
        } else {
            lockTetromino(droppingTetromino_x, droppingTetromino_y, activePiece, tetrisGrid);
            clearTetrominoRow(tetrisGrid);
            spawnTetromino();
        }
        lastDrop = millis();
    }

    noInterrupts();
    // Compose frame: start with grid, overlay active piece
    for (int y = 0; y < 11; y++) {
        rowData[y] = MINUTE_BITS_MASK | (tetrisGrid[y] & 0x07FF);
    }

    for (int row = 0; row < activePiece.size; row++) {
        for (int col = 0; col < activePiece.size; col++) {
            if (activePiece.shape[row][col]) {
                int gx = droppingTetromino_x + col;
                int gy = droppingTetromino_y + row;
                if (gx >= 0 && gx < 11 && gy >= 0 && gy < 11) {
                    rowData[gy] |= (1 << (10 - gx));
                }
            }
        }
    }
    interrupts();
}





//Mode 4 - various LED effects

int activeEffect = 1; // selects which effect to run

static bool scrollInitialized = false;
static bool glitchInitialized = false;
static bool rainInitialized = false;
static bool conwayInitialized = false;

// Declaring LED mapping for each letter in letter scrolling display mode.
// This is stored in the flash memory because there isn't enough RAM space.
const uint8_t font5x7[][5] PROGMEM = {
  // Letters A-Z (indexes 0-25)
  {0x7C,0x12,0x11,0x12,0x7C}, // A
  {0x7F,0x49,0x49,0x49,0x36}, // B
  {0x3E,0x41,0x41,0x41,0x22}, // C
  {0x7F,0x41,0x41,0x22,0x1C}, // D
  {0x7F,0x49,0x49,0x49,0x41}, // E
  {0x7F,0x09,0x09,0x09,0x01}, // F
  {0x3E,0x41,0x49,0x49,0x7A}, // G
  {0x7F,0x08,0x08,0x08,0x7F}, // H
  {0x00,0x41,0x7F,0x41,0x00}, // I
  {0x20,0x40,0x41,0x3F,0x01}, // J
  {0x7F,0x08,0x14,0x22,0x41}, // K
  {0x7F,0x40,0x40,0x40,0x40}, // L
  {0x7F,0x02,0x04,0x02,0x7F}, // M
  {0x7F,0x04,0x08,0x10,0x7F}, // N
  {0x3E,0x41,0x41,0x41,0x3E}, // O
  {0x7F,0x09,0x09,0x09,0x06}, // P
  {0x3E,0x41,0x51,0x21,0x5E}, // Q
  {0x7F,0x09,0x19,0x29,0x46}, // R
  {0x46,0x49,0x49,0x49,0x31}, // S
  {0x01,0x01,0x7F,0x01,0x01}, // T
  {0x3F,0x40,0x40,0x40,0x3F}, // U
  {0x1F,0x20,0x40,0x20,0x1F}, // V
  {0x3F,0x40,0x38,0x40,0x3F}, // W
  {0x63,0x14,0x08,0x14,0x63}, // X
  {0x07,0x08,0x70,0x08,0x07}, // Y
  {0x61,0x51,0x49,0x45,0x43}, // Z
  // Space (index 26)
  {0x00,0x00,0x00,0x00,0x00},
  
  // Numbers 0-9 (indexes 27-36)
  {0x3E,0x51,0x49,0x45,0x3E}, // 0
  {0x00,0x42,0x7F,0x40,0x00}, // 1
  {0x42,0x61,0x51,0x49,0x46}, // 2
  {0x22,0x41,0x49,0x49,0x36}, // 3
  {0x18,0x14,0x12,0x7F,0x10}, // 4
  {0x27,0x45,0x45,0x45,0x39}, // 5
  {0x3C,0x4A,0x49,0x49,0x30}, // 6
  {0x01,0x71,0x09,0x05,0x03}, // 7
  {0x36,0x49,0x49,0x49,0x36}, // 8
  {0x06,0x49,0x49,0x29,0x1E}, // 9
  
  // Common punctuation (indexes 37+)
  {0x00,0x00,0x5F,0x00,0x00}, // ! (37)
  {0x02,0x01,0x51,0x09,0x06}, // ? (38)
  {0x00,0x36,0x36,0x00,0x00}, // : (39)
  {0x00,0x60,0x60,0x00,0x00}, // . (40)
  {0x00,0x50,0x30,0x04,0x00}  // , (41)
};

// Function that returns the LED mapping for a given letter
uint8_t getFontColumn(char c, uint8_t col) {

    if (col >= 5) return 0;

    uint8_t index;

    if (c >= 'A' && c <= 'Z') {
        index = c - 'A';  // letters A-Z
    } 
    else if (c >= '0' && c <= '9') {
        index = 27 + (c - '0');  // numbers 0-9
    }
    else {

        switch (c) {
            case '!': index = 37; break;
            case '?': index = 38; break;
            case ':': index = 39; break;
            case '.': index = 40; break;
            case ',': index = 41; break;
            default:  index = 26;  // space or unknown chars (all zero columns)
        }
    }

    return pgm_read_byte(&font5x7[index][col]);
}

// Core function in this mode that controls the various LED effects
void miscellaneous() {

    unsigned long now = millis();

    // Effect 1 - Scrolling text
    if (activeEffect == 1) {

        static const char scrollText[] = "INSTITUTION OF ELECTRONICS"; // Scrolling text (uppercase only)
        static int scrollPos = -11;
        static unsigned long lastScrollUpdate = 0;
        const unsigned long scrollInterval = 150;
        const int charWidth = 6;  // 5 pixels font + 1 pixel spacing
        
        // If the scrolling text has not been initialized, initialize it
        if (!scrollInitialized) {
            for (int row = 0; row < 11; row++) {
                tempRowData[row] = MINUTE_BITS_MASK;
                scrollPos = -11;
            }
            scrollInitialized = true;
        }
        
        if (now - lastScrollUpdate >= scrollInterval) {
            lastScrollUpdate = now;
            
            // Clear buffer with reserved bits ON
            for (int row = 0; row < 11; row++) {
                tempRowData[row] = MINUTE_BITS_MASK;
            }
            
            // For each column of display (0..10)
            for (int col = 0; col < 11; col++) {
                int textPixelPos = scrollPos + col;
                int charIndex = textPixelPos / charWidth;
                int charCol = textPixelPos % charWidth;
                if (charCol < 0 || charCol >= 5) continue;
                
                if (scrollText[charIndex] == 0) {
                    // When the whole message + trailing spaces scrolled past,
                    // reset scrollPos to -11.
                    if (scrollPos >= (int)(strlen(scrollText) * charWidth)) {
                        scrollPos = -11;
                    }
                    break;
                }
                
                if (charCol < 5) {
                    uint8_t colData = getFontColumn(scrollText[charIndex], charCol);
                    
                    // font 7 rows tall, map to rows 2..8 to center vertically (0-based)
                    for (int row = 0; row < 7; row++) {
                        if (colData & (1 << row)) {
                            tempRowData[row + 2] |= (1 << (10 - col));
                        }
                    }
                }
            }
            
            // Increment the scroll position
            scrollPos++;
            
            // Atomically update rowData
            noInterrupts();
            for (int i = 0; i < 11; i++) {
                rowData[i] = tempRowData[i];
            }
            interrupts();
        }
    }

    // Effect 2 - LED decay effect
    else if (activeEffect == 2) {
        static uint16_t decayState[11];
        static uint8_t decayTimers[11][11]; // one timer per bit per row
        static unsigned long lastTick = 0;
        const unsigned long tickInterval = 100; // ms between timer decrements

        // If the glitch effect has not been initialized, initialize it
        if (!glitchInitialized) {
            for (int row = 0; row < 11; row++) {
                decayState[row] = MINUTE_BITS_MASK;
                for (int bit = 0; bit < 11; bit++) {
                    decayTimers[row][bit] = random(10, 30); // initial random timer values
                }
            }
            glitchInitialized = true;
        }

        // If the decay effect has been running for longer than the tick interval, update the state
        if (now - lastTick >= tickInterval) {
            lastTick = now;

            for (int row = 0; row < 11; row++) {
                uint16_t rowValue = decayState[row];
                for (int bit = 0; bit < 11; bit++) {
                    if (decayTimers[row][bit] == 0) {
                        rowValue ^= (1 << (10 - bit));          // toggle bit
                        decayTimers[row][bit] = random(10, 30); // reset timer (in ticks)
                    } else {
                        decayTimers[row][bit]--;
                    }
                }
                decayState[row] = rowValue;
                tempRowData[row] = rowValue;
            }

            // Atomically update rowData
            noInterrupts();
            for (int i = 0; i < 11; i++) {
                rowData[i] = tempRowData[i];
            }
            interrupts();
        }

    }

    // Effect 3 - Conway's Game of Life
    else if (activeEffect == 3) {
        static bool grid[11][11];
        static unsigned long conwayLastUpdate = 0;
        const unsigned long updateInterval = 750;
        static uint16_t history[5][11];
        static int historySize = 0;

        // Function that converts a 2D grid of booleans to a 1D array of uint16_t
        auto gridToRow = [](bool g[11][11], uint16_t out[11]) {
            for (int y = 0; y < 11; y++) {
                uint16_t row = MINUTE_BITS_MASK;
                for (int x = 0; x < 11; x++) {
                    if (g[y][x]) {
                        row |= (1 << (10 - x));
                    }
                }
                out[y] = row;
            }
        };

        // Function that checks if a given 1D array of uint16_t is a repeated pattern
        auto isRepeatedPattern = [&](uint16_t current[11]) -> bool {
            for (int i = 0; i < historySize; i++) {
                bool match = true;
                for (int j = 0; j < 11; j++) {
                    if (history[i][j] != current[j]) {
                        match = false;
                        break;
                    }
                }
                if (match) return true;
            }
            return false;
        };

        // If the conway effect has not been initialized, initialize it
        if (!conwayInitialized) {
            for (int y = 0; y < 11; y++) {
                for (int x = 0; x < 11; x++) {
                    grid[y][x] = random(100) < 25;
                }
            }
            historySize = 0;
            conwayInitialized = true;
        }

        if (millis() - conwayLastUpdate >= updateInterval) {
            conwayLastUpdate = millis();

            // Compute next gen
            bool next[11][11] = {false};

            for (int y = 0; y < 11; y++) {
                for (int x = 0; x < 11; x++) {
                    int neighbors = 0;
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && nx < 11 && ny >= 0 && ny < 11) {
                                if (grid[ny][nx]) neighbors++;
                            }
                        }
                    }

                    if (grid[y][x]) {
                        next[y][x] = (neighbors == 2 || neighbors == 3);
                    } else {
                        next[y][x] = (neighbors == 3);
                    }
                }
            }

            // Convert next to row format
            uint16_t currentRows[11];
            gridToRow(next, currentRows);

            // Check for loops
            if (isRepeatedPattern(currentRows)) {
                // Reset board
                for (int y = 0; y < 11; y++) {
                    for (int x = 0; x < 11; x++) {
                        grid[y][x] = random(100) < 25;
                    }
                }
                historySize = 0;
            } else {
                // Save to history
                if (historySize < 5) historySize++;
                for (int i = historySize - 1; i > 0; i--) {
                    memcpy(history[i], history[i - 1], sizeof(uint16_t) * 11);
                }
                memcpy(history[0], currentRows, sizeof(uint16_t) * 11);

                // Update grid state
                memcpy(grid, next, sizeof(grid));

                // Push to display
                for (int y = 0; y < 11; y++) {
                    tempRowData[y] = currentRows[y];
                }

                noInterrupts();
                for (int i = 0; i < 11; i++) {
                    rowData[i] = tempRowData[i];
                }
                interrupts();
            }
        }
    }

    // Effect 4 - Matrix rain
    else if (activeEffect == 4) {

        static unsigned long rainLastUpdate = 0;
        const unsigned long rainInterval = 50;
        static float dropPositions[11];
        static float dropVelocities[11];
        const float gravity = 0.03f;

        // If the rain effect has not been initialized, initialize it
        if (!rainInitialized) {
            for (int i = 0; i < 11; i++) {
                dropPositions[i] = -10.0f;
                dropVelocities[i] = 0.0f;
            }
            rainInitialized = true;
        }

        // If the rain effect has been running for longer than the rain interval, update the state
        if (now - rainLastUpdate >= rainInterval) {
            rainLastUpdate = now;

            // Clear temp buffer first
            for (int row = 0; row < 11; row++) {
                tempRowData[row] = MINUTE_BITS_MASK;
            }

            for (int col = 0; col < 11; col++) {
                // If the drop position is negative, randomly spawn a new drop
                if (dropPositions[col] < 0 && random(100) < 5) {
                    dropPositions[col] = 0.0f;
                    dropVelocities[col] = 0.1f + random(100) / 500.0f;
                }

                // If the drop position is positive, update the drop
                if (dropPositions[col] >= 0) {
                    int headRow = (int)dropPositions[col];
                    int tailRow = headRow - 3;

                    if (tailRow >= 11) {
                        // Whole strand passed bottom, reset drop
                        dropPositions[col] = -10.0f;
                        dropVelocities[col] = 0.0f;
                    } else {
                        // Draw head
                        if (headRow < 11 && headRow >= 0) {
                            tempRowData[headRow] |= (1 << (10 - col));
                        }
                        // Draw trail
                        for (int t = 1; t <= 3; t++) {
                            int trailRow = headRow - t;
                            if (trailRow >= 0 && trailRow < 11) {
                                tempRowData[trailRow] |= (1 << (10 - col));
                            }
                        }
                        // Accelerate and move drop down
                        dropVelocities[col] += gravity;
                        dropPositions[col] += dropVelocities[col];
                    }
                }
            }

            // Atomically update rowData
            noInterrupts();
            for (int i = 0; i < 11; i++) {
                rowData[i] = tempRowData[i];
            }
            interrupts();
        }
    }
}





//Main

// Initializing the hardware
void setup() {
    Serial.begin(9600);
    rtc.begin();
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    pinMode(clearpin, OUTPUT);
    pinMode(Aserialdata, OUTPUT);
    pinMode(Ashiftclock, OUTPUT);
    pinMode(Alatchclock, OUTPUT);
    pinMode(Bserialdata, OUTPUT);
    pinMode(Bshiftclock, OUTPUT);
    pinMode(Blatchclock, OUTPUT);
    pinMode(starpin, OUTPUT);
    pinMode(plateone, INPUT);
    pinMode(platetwo, INPUT);
    pinMode(platethree, INPUT);
    pinMode(platefour, INPUT);

    digitalWrite(clearpin, LOW);
    digitalWrite(clearpin, HIGH);
    clear();

    for (int i = 0; i < 11; i++) {
        rowData[i] = MINUTE_BITS_MASK;
    }

    // Initializing Timer1 for multiplexing
    Timer1.initialize(1000);
    Timer1.attachInterrupt(multiplexInterruptHandler);

}

// Main loop
void loop() {

    shortPressedPlate = 0;
    bool plateOne = (digitalRead(plateone) == LOW);
    bool plateTwo = (digitalRead(platetwo) == LOW);
    bool plateThree = (digitalRead(platethree) == LOW);
    bool plateFour = (digitalRead(platefour) == LOW);

    int newPlate = 0;
    if (plateOne) newPlate = 1;
    else if (plateTwo) newPlate = 2;
    else if (plateThree) newPlate = 3;
    else if (plateFour) newPlate = 4;

    // If a new plate is pressed, handle the press
    if (newPlate != 0) {
        // If the new plate is different from the last plate, handle the press
        if (newPlate != lastPlate) {
            pressStart = millis();
            longPressHandled = false;
        } else {
            // If the new plate is the same as the last plate, and the press has been held for longer than 2 seconds, change the mode
            if (!longPressHandled && millis() - pressStart >= 2000) {
                for (int i = 0; i < 11; i++) {
                    rowData[i] = MINUTE_BITS_MASK;
                }

                // Mode is changed; reset the mode-specific variables
                modeOneChanged = true;
                snakeInitialized = false;
                tetrisActive = false;
                scrollInitialized = false;
                glitchInitialized = false;
                rainInitialized = false;
                conwayInitialized = false;

                // Set the new mode
                mode = (mode == newPlate) ? 0 : newPlate;
                longPressHandled = true;

                // If the new mode is not 0, turn on the logo
                if (mode != 0) {
                    digitalWrite(starpin, HIGH);
                } else {
                    digitalWrite(starpin, LOW);
                }
                
            }
        }
    } else {
        // If the last plate is not 0, and the press has not been long pressed, set the shortPressedPlate to the pressed plate
        if (lastPlate != 0 && !longPressHandled) {
            shortPressedPlate = lastPlate;
        }
    }

    // Set the last plate to the new plate
    lastPlate = newPlate;

    // If the mode is 4 and a touch sensor is pressed, change the active effect
    if (mode == 4 && shortPressedPlate != 0) {
        if (shortPressedPlate == 1) {
            activeEffect = 1;
        } else if (shortPressedPlate == 2) {
            activeEffect = 2;
        } else if (shortPressedPlate == 3) {
            activeEffect = 3;
        } else if (shortPressedPlate == 4) {
            activeEffect = 4;
        }
        // Reset the mode-specific variables
        scrollInitialized = false;
        glitchInitialized = false;
        rainInitialized = false;
        conwayInitialized = false;
    }

    // Run the mode-specific function
    switch (mode) {
        case 1:
            time();
            break;
        case 2:
            snake();
            break;
        case 3:
            tetris();
            break;
        case 4:
            miscellaneous();
            break;
        default:
            break;
    }
}