#include "defines.h"
#include "audio.h"
#include <mutex>

/********************************************************************
 * Singleton class! Implemented using:
 * 
 * https://refactoring.guru/design-patterns/singleton/cpp/example#example-1
 * 
 ********************************************************************/


static const char *TAG = "audio";

static const int TASK_SIZE = TASK_STACK_SIZE_LARGE;
static const int TASK_INTERVAL = 1000; // ms


TaskHandle_t audio::audio_task_handle = NULL;

audio* audio::audio_{nullptr};
std::mutex audio::mutex_;


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
 audio *audio::GetInstance() {
    std::lock_guard<std::mutex> lock(audio::mutex_);
    
    // No instance yet - create one
    if( audio_ == nullptr ) {
        ESP_LOGD(TAG, "Creating new instance");
        
        audio_ = new audio();
        audio_->init();

    } else {
        ESP_LOGD(TAG, "Returning existing instance");
    }

    return audio_;
}

// Private
audio::audio(void) {
}

void audio::init(void) {
    ESP_LOGD(TAG, "Initializing Audio");
}


