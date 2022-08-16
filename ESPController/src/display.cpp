#include "defines.h"
#include "display.h"
#include <mutex>
#include <string>
#include <stdarg.h>

// This conflicts with (I THINK) mutex, where chrono.h also defines a min()
// macro, as well as something in M5Core2. So including it here explicitly
// rather than in defines.h, which goes everywhere. And making sure it goes
// LAST, as it has a conditional define, whereas chrono.h does not:
// https://github.com/m5stack/M5StickC/pull/139
#include <M5Core2.h>

/********************************************************************
 * Singleton class! Implemented using:
 * 
 * https://refactoring.guru/design-patterns/singleton/cpp/example#example-1
 * 
 ********************************************************************/

#define DISPLAY_TEXT_SIZE 2
#define DISPLAY_TEXT_COLOR WHITE
#define DISPLAY_BOOT_TEXT "Stack initializing"

static const char *TAG = "display";

/********************************************************************
 * 
 * Shape dimensions
 * 
 ********************************************************************/

/********************************************************************
 * Rough layout of the screen:
 * 
 * +----------------------------------------------------------------+
 * | +------------------------------------------------------------+ | 
 * | |                                                            | |
 * | |                      Battery + Charge %                    | |
 * | |                                                            | |
 * | +------------------------------------------------------------+ |
 * |                                                                |
 * |  Cell Temp (A)                              <Empty> (B)        |
 * |   Min / Max C                                                  |
 * |                                                                |
 * |  Cell Voltage Delta (C)                     Stack Voltage (D)  |
 * |    XYX mV                                     XYZ.AB V         |
 * |                                                                |
 * |                    Random Diagnostics Here                     |
 * +----------------------------------------------------------------+
 * 
 ********************************************************************/


// Make sure the LCD stack is initialized before accessing its properties.

static const int DISPLAY_BORDER_WIDTH = 10;
static int DISPLAY_USABLE_WIDTH;        // Set during init() - needs a lookup
static int DISPLAY_USABLE_HEIGHT;       // Set during init() - needs a lookup
static const int DISPLAY_CHAR_BUFFER = 256; // For 'printf' buffer

// Error colors
static const int DISPLAY_ERROR_TEXT_SIZE = 4;
static const int DISPLAY_ERROR_TEXT_COLOR = WHITE;
static const int DISPLAY_ERROR_BG_COLOR = RED;

// Battery display settings
static const int DISPLAY_BATTERY_BORDER_COLOR = LIGHTGREY;
static const int DISPLAY_BATTERY_CHARGE_COLOR = GREEN;
static const int DISPLAY_BATTERY_DRAIN_COLOR = DARKGREY;
static const int DISPLAY_BATTERY_BORDER = 5;
static const int DISPLAY_BATTERY_HEIGHT = 60;
static const int DISPLAY_BATTERY_TEXT_SIZE = 3;
static int DISPLAY_BATTERY_WIDTH;       // Set during init() - needs a lookup
static int DISPLAY_BATTERY_CURSOR_X;    // Set during init() - needs a lookup
static int DISPLAY_BATTERY_CURSOR_Y;    // Set during init() - needs a lookup

// Diagnostic display settings - we can use const because we're calculating from the bottom
static const int DISPLAY_DIAGNOSTIC_COLOR = DARKGREY;
static const int DISPLAY_DIAGNOSTIC_HEIGHT = 30;
static const int DISPLAY_DIAGNOSTIC_CURSOR_X = DISPLAY_BORDER_WIDTH;
static const int DISPLAY_DIAGNOSTIC_X = 0;  // Start all the way at the left
static int DISPLAY_DIAGNOSTIC_CURSOR_Y;  // Set during init() - needs a lookup
static int DISPLAY_DIAGNOSTIC_WIDTH;     // Set during init() - needs a lookup
static int DISPLAY_DIAGNOSTIC_Y;         // Set during init() - needs a lookup

static const int DISPLAY_STATUS_COLOR = WHITE;
static const int DISPLAY_STATUS_TEXT_SIZE = 2;
static const int _DISPLAY_STATUS_OFFSET = 20;   // extra padding from the borders
static int DISPLAY_STATUS_A_CURSOR_X = _DISPLAY_STATUS_OFFSET;   
static int DISPLAY_STATUS_A_CURSOR_Y;   // Set during init() - needs a lookup
static int DISPLAY_STATUS_B_CURSOR_X;   // Set during init() - needs a lookup
static int DISPLAY_STATUS_B_CURSOR_Y;   // Set during init() - needs a lookup
static int DISPLAY_STATUS_C_CURSOR_X = _DISPLAY_STATUS_OFFSET;
static int DISPLAY_STATUS_C_CURSOR_Y;   // Set during init() - needs a lookup
static int DISPLAY_STATUS_D_CURSOR_X;   // Set during init() - needs a lookup
static int DISPLAY_STATUS_D_CURSOR_Y;   // Set during init() - needs a lookup

display* display::display_{nullptr};
std::mutex display::mutex_;

/********************************************************************
*
* Initialization
*
********************************************************************/

/**
 * The first time we call GetInstance we will lock the storage location
 *      and then we make sure again that the variable is null and then we
 *      set the value. RU:
 */
 display *display::GetInstance() {
    std::lock_guard<std::mutex> lock(display::mutex_);
    
    // No instance yet - create one
    if( display_ == nullptr ) {
        ESP_LOGD(TAG, "Creating new instance");
        
        display_ = new display();
        display_->init();

    } else {
        ESP_LOGD(TAG, "Returning existing instance");
    }

    return display_;
}

// Private
display::display(void) {
}

void display::init(void) {
    ESP_LOGD(TAG, "Initializing display");
    
    // Make sure the screen is initialized
    M5.Lcd.begin();

    DISPLAY_USABLE_WIDTH  = M5.Lcd.width() - DISPLAY_BORDER_WIDTH * 2;  // we leave 10px on each side as a border
    DISPLAY_USABLE_HEIGHT = M5.Lcd.height() - DISPLAY_BORDER_WIDTH * 2; // we leave 10px on each side as a border
    
    DISPLAY_BATTERY_WIDTH = DISPLAY_USABLE_WIDTH;   // use the full width of the screen for the battery
    DISPLAY_BATTERY_CURSOR_X = (int) (DISPLAY_BATTERY_WIDTH / 2 - 1); // Center as much as possible in the battery; 1-2 digits + % sign
    // Also try to center this in the middle of the battery - bit of fudgery needed, as 'half way down' is the top left pixel of 
    // whatever we are printing, and 'text size' is not the same as pixels...
    DISPLAY_BATTERY_CURSOR_Y = (int) (DISPLAY_BORDER_WIDTH + (DISPLAY_BATTERY_HEIGHT / 2) - 10);

    // Full width for diagnostics
    DISPLAY_DIAGNOSTIC_WIDTH = M5.Lcd.width();
    DISPLAY_DIAGNOSTIC_Y = M5.Lcd.height() - DISPLAY_DIAGNOSTIC_HEIGHT;
    // Also try to center this in the middle of the band - bit of fudgery needed, as 'half way down' is the top left pixel of 
    // whatever we are printing, and 'text size' is not the same as pixels...
    DISPLAY_DIAGNOSTIC_CURSOR_Y = DISPLAY_DIAGNOSTIC_Y + 5;

    // The 4 Diagnostic quadrants
    // Offset into the center for the X axis, and then mirror that to the other side 
    // C & D use the same offset as A & B respectively
    DISPLAY_STATUS_B_CURSOR_X = (int) (DISPLAY_USABLE_WIDTH/2) + DISPLAY_STATUS_A_CURSOR_X;
    DISPLAY_STATUS_D_CURSOR_X = DISPLAY_STATUS_B_CURSOR_X;

    int status_height = DISPLAY_USABLE_HEIGHT - DISPLAY_BATTERY_HEIGHT - DISPLAY_DIAGNOSTIC_HEIGHT;
    DISPLAY_STATUS_A_CURSOR_Y = DISPLAY_BORDER_WIDTH + DISPLAY_BATTERY_HEIGHT + _DISPLAY_STATUS_OFFSET;
    DISPLAY_STATUS_B_CURSOR_Y = DISPLAY_STATUS_A_CURSOR_Y;
    DISPLAY_STATUS_C_CURSOR_Y = (int)(status_height/2) + DISPLAY_STATUS_A_CURSOR_Y;
    DISPLAY_STATUS_D_CURSOR_Y = DISPLAY_STATUS_C_CURSOR_Y;

    // Diagnostic code to show cell placement
    /*
    M5.Lcd.setTextSize(DISPLAY_STATUS_TEXT_SIZE);
    M5.Lcd.setTextColor(DISPLAY_STATUS_COLOR);    
    M5.Lcd.setCursor( DISPLAY_STATUS_A_CURSOR_X, DISPLAY_STATUS_A_CURSOR_Y ); // center on the battery
    M5.Lcd.printf( "Cell A");

    M5.Lcd.setTextSize(DISPLAY_STATUS_TEXT_SIZE);
    M5.Lcd.setTextColor(DISPLAY_STATUS_COLOR);    
    M5.Lcd.setCursor( DISPLAY_STATUS_B_CURSOR_X, DISPLAY_STATUS_B_CURSOR_Y ); // center on the battery
    M5.Lcd.printf( "Cell B");

    M5.Lcd.setTextSize(DISPLAY_STATUS_TEXT_SIZE);
    M5.Lcd.setTextColor(DISPLAY_STATUS_COLOR);    
    M5.Lcd.setCursor( DISPLAY_STATUS_C_CURSOR_X, DISPLAY_STATUS_C_CURSOR_Y ); // center on the battery
    M5.Lcd.printf( "Cell C");

    M5.Lcd.setTextSize(DISPLAY_STATUS_TEXT_SIZE);
    M5.Lcd.setTextColor(DISPLAY_STATUS_COLOR);    
    M5.Lcd.setCursor( DISPLAY_STATUS_D_CURSOR_X, DISPLAY_STATUS_D_CURSOR_Y ); // center on the battery
    M5.Lcd.printf( "Cell D");
    */

    // Set boot diagnostics, if any
    M5.Lcd.setTextSize(DISPLAY_TEXT_SIZE);
    M5.Lcd.setTextColor(DISPLAY_TEXT_COLOR);
    M5.Lcd.clear();
    M5.Lcd.println(DISPLAY_BOOT_TEXT);   
}

/********************************************************************
 * 
 * display information methods
 * 
 ********************************************************************/

void display::clear(void) {
    M5.lcd.clear();
}

void display::display_error(const char *format, ...) {
    this->clear();

    M5.Lcd.setTextColor(DISPLAY_ERROR_TEXT_COLOR);    
    M5.Lcd.setTextSize(DISPLAY_ERROR_TEXT_SIZE);    
    M5.Lcd.fillScreen(DISPLAY_ERROR_BG_COLOR);
    M5.Lcd.setCursor(DISPLAY_BORDER_WIDTH, DISPLAY_BORDER_WIDTH);

    // Effectively wrapping 'printf':
    // https://stackoverflow.com/questions/1056411/how-to-pass-variable-number-of-arguments-to-printf-sprintf
    char buffer[DISPLAY_CHAR_BUFFER];
    va_list args;
    va_start (args, format);
    vsnprintf (buffer, DISPLAY_CHAR_BUFFER -1, format, args);
    
    ESP_LOGD(TAG, "Error: %s", buffer);
    M5.Lcd.printf(buffer);    
    va_end(args);     
}

void display::_display_status_cell(int x, int y, const char *format, ...) {
    
    M5.Lcd.setTextSize(DISPLAY_STATUS_TEXT_SIZE);
    M5.Lcd.setTextColor(DISPLAY_STATUS_COLOR);    
    M5.Lcd.setCursor( x, y );
    
    // Effectively wrapping 'printf':
    // https://stackoverflow.com/questions/1056411/how-to-pass-variable-number-of-arguments-to-printf-sprintf
    char buffer[DISPLAY_CHAR_BUFFER];
    va_list args;
    va_start (args, format);
    vsnprintf (buffer, DISPLAY_CHAR_BUFFER -1, format, args);
    
    ESP_LOGD(TAG, "Status Cell: %s", buffer);
    M5.Lcd.printf(buffer);    
    va_end(args);     
}

// Cell A: "20/44 C"
void display::display_cell_temp(int min_temp, int max_temp) {
    this->_display_status_cell(
        DISPLAY_STATUS_A_CURSOR_X,
        DISPLAY_STATUS_A_CURSOR_Y,
        "%i/%i C", min_temp, max_temp
    );
}

// Cell B: "3 Cells"
void display::display_cell_count(int count) {
    this->_display_status_cell(
        DISPLAY_STATUS_B_CURSOR_X,
        DISPLAY_STATUS_B_CURSOR_Y,
        "%i Cells", count
    );
}


// Cell C: "301 mV"
void display::display_cell_voltage_delta(int delta) {
    int triangle_size = 12;
    int triangle_middle = (int)(triangle_size/2);
    int text_offset = triangle_middle;  // use the same offset as the middle.

    // void drawTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t color)
    M5.Lcd.drawTriangle(
        DISPLAY_STATUS_C_CURSOR_X, DISPLAY_STATUS_C_CURSOR_Y + triangle_size,
        DISPLAY_STATUS_C_CURSOR_X + triangle_size, DISPLAY_STATUS_C_CURSOR_Y + triangle_size,
        DISPLAY_STATUS_C_CURSOR_X + triangle_middle, DISPLAY_STATUS_C_CURSOR_Y,
        WHITE
    );

    this->_display_status_cell(
        DISPLAY_STATUS_C_CURSOR_X + triangle_size + text_offset,
        DISPLAY_STATUS_C_CURSOR_Y,        
        // If your platform supports it, you could use unicode escape characters. 
        // For Greek capital delta the code is \u0394:
        // XXX doens't work :(         
        "%i mV", delta
    );
}

// Cell D: "11.23 V"
void display::display_stack_voltage(float voltage) {
    this->_display_status_cell(
        DISPLAY_STATUS_D_CURSOR_X,
        DISPLAY_STATUS_D_CURSOR_Y,
        "%.2f V", voltage
    );
}

// Charge percentage, defaultc color
void display::display_battery(int charge) {
    this->display_battery( charge, DISPLAY_BATTERY_CHARGE_COLOR );
}

// Charge percentage, with specific color
void display::display_battery(int charge, int color) {

    // Draw the battery outline
    M5.Lcd.drawRect( 
        DISPLAY_BORDER_WIDTH,           // x start
        DISPLAY_BORDER_WIDTH,           // y start
        DISPLAY_BATTERY_WIDTH,          // Width
        DISPLAY_BATTERY_HEIGHT,         // Height
        DISPLAY_BATTERY_BORDER_COLOR    // Border color
    );

    // Fill the battery to charge
    int battery_width = (int)((DISPLAY_BATTERY_WIDTH - (DISPLAY_BATTERY_BORDER * 2)) * charge / 100 );
    M5.Lcd.fillRect( 
        DISPLAY_BORDER_WIDTH + DISPLAY_BATTERY_BORDER,          // x start - offset to get a border around the battery
        DISPLAY_BORDER_WIDTH + DISPLAY_BATTERY_BORDER,          // y start - offset to get a border around the battery
        battery_width,                                          // Width  - we fill in the above rect
        DISPLAY_BATTERY_HEIGHT - DISPLAY_BATTERY_BORDER * 2,    // Height - we fill in the above rect
        color                                                   // Content color
    );

    // Fill the battery 'drain'
    M5.Lcd.fillRect( 
        DISPLAY_BORDER_WIDTH + DISPLAY_BATTERY_BORDER + battery_width,  // x start - right next to the charge
        DISPLAY_BORDER_WIDTH + DISPLAY_BATTERY_BORDER,                  // y start - same as the charge
        DISPLAY_BATTERY_WIDTH - DISPLAY_BATTERY_BORDER - battery_width, // Width  - the reamining width
        DISPLAY_BATTERY_HEIGHT - DISPLAY_BATTERY_BORDER * 2,            // Height - we fill in the above rect
        DISPLAY_BATTERY_DRAIN_COLOR                                     // Content color
    );
    

    // Print the charge percentage
    M5.Lcd.setTextSize(DISPLAY_BATTERY_TEXT_SIZE);
    M5.Lcd.setCursor( DISPLAY_BATTERY_CURSOR_X, DISPLAY_BATTERY_CURSOR_Y ); // center on the battery
    M5.Lcd.printf( "%i%%", charge);

    // We changed the default, change it back to be a good citizen
    M5.Lcd.setTextSize(DISPLAY_TEXT_SIZE);
}

void display::display_diagnostics(std::string msg) {

    M5.Lcd.fillRect( 
        DISPLAY_DIAGNOSTIC_X,                                                 
        DISPLAY_DIAGNOSTIC_Y,
        DISPLAY_DIAGNOSTIC_WIDTH,
        DISPLAY_DIAGNOSTIC_HEIGHT,
        DISPLAY_DIAGNOSTIC_COLOR
    );

    M5.Lcd.setCursor( DISPLAY_DIAGNOSTIC_CURSOR_X, DISPLAY_DIAGNOSTIC_CURSOR_Y );
        
    // printf takes char*, not std::string
    char* buffer = const_cast<char*>(msg.c_str());
    ESP_LOGD(TAG, "Diagnostic: %s", buffer);
    M5.Lcd.printf(buffer);
        
}

void display::display_border(int color) {
    // Draw the battery outline
    M5.Lcd.drawRect( 
        0,                  // x start
        0,                  // y start
        M5.Lcd.width(),     // Width
        M5.Lcd.height(),    // Height
        color               // Border color
    );
}

void display::display_background(int color) {
    // Draw the battery outline
    M5.Lcd.fillScreen(color);
}
