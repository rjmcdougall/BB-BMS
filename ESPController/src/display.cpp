#include "defines.h"
#include "display.h"
#include <mutex>

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

static const int TASK_SIZE = TASK_STACK_SIZE_LARGE;
static const int TASK_INTERVAL = 1000; // ms

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
 * |  Total Current                              Module Temp        |
 * |      XY V                                     Min / Max        |
 * |                                                                |
 * |  Cell Voltage                                 Can Bus          |
 * |      XY V                                     Min / Max        |
 * |                                                                |
 * |                    Random Diagnostics Here                     |
 * +----------------------------------------------------------------+
 * 
 ********************************************************************/


// Make sure the LCD stack is initialized before accessing its properties.

static const int DISPLAY_BORDER_WIDTH = 10;
static int DISPLAY_USABLE_WIDTH;        // Set during init() - needs a lookup
static int DISPLAY_USABLE_HEIGHT;       // Set during init() - needs a lookup

// Battery display settings
static const int DISPLAY_BATTERY_BORDER_COLOR = LIGHTGREY;
static const int DISPLAY_BATTERY_CHARGE_COLOR = GREEN;
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


TaskHandle_t display::display_task_handle = NULL;

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
    DISPLAY_DIAGNOSTIC_CURSOR_Y = DISPLAY_DIAGNOSTIC_Y + 10;

    //M5.Lcd.setTextFont(2);
    M5.Lcd.setTextSize(DISPLAY_TEXT_SIZE);
    M5.Lcd.setTextColor(DISPLAY_TEXT_COLOR);

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

// Charge percentage
void display::display_battery(int charge) {

    // Draw the battery outline
    M5.Lcd.drawRect( 
        DISPLAY_BORDER_WIDTH,           // x start
        DISPLAY_BORDER_WIDTH,           // y start
        DISPLAY_BATTERY_WIDTH,          // Width
        DISPLAY_BATTERY_HEIGHT,         // Height
        DISPLAY_BATTERY_BORDER_COLOR     // Border color
    );

    // Fill the battery to charge
    int battery_width = (int) ((DISPLAY_BATTERY_WIDTH / 100) * charge);
    M5.Lcd.fillRect( 
        DISPLAY_BORDER_WIDTH + DISPLAY_BATTERY_BORDER,          // x start - offset to get a border around the battery
        DISPLAY_BORDER_WIDTH + DISPLAY_BATTERY_BORDER,          // y start - offset to get a border around the battery
        battery_width,                                          // Width  - we fill in the above rect
        DISPLAY_BATTERY_HEIGHT - DISPLAY_BATTERY_BORDER * 2,    // Height - we fill in the above rect
        DISPLAY_BATTERY_CHARGE_COLOR                             // Border color
    );

    // Print the charge percentage
    M5.Lcd.setTextSize(DISPLAY_BATTERY_TEXT_SIZE);
    M5.Lcd.setCursor( DISPLAY_BATTERY_CURSOR_X, DISPLAY_BATTERY_CURSOR_Y ); // center on the battery
    M5.Lcd.printf( "%i%%", charge);

    // We changed the default, change it back to be a good citizen
    M5.Lcd.setTextSize(DISPLAY_TEXT_SIZE);
}

void display::display_diagnostics(void) {

    M5.Lcd.fillRect( 
        DISPLAY_DIAGNOSTIC_X,                                                 
        DISPLAY_DIAGNOSTIC_Y,
        DISPLAY_DIAGNOSTIC_WIDTH,
        DISPLAY_DIAGNOSTIC_HEIGHT,
        DISPLAY_DIAGNOSTIC_COLOR
    );

    M5.Lcd.setCursor( DISPLAY_DIAGNOSTIC_CURSOR_X, DISPLAY_DIAGNOSTIC_CURSOR_Y );

    M5.Lcd.printf("Hello world");
}    
