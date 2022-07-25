class sample_task
{

public:
    void run(void);

private:
    static void task_one(void *param);

    // Private variables
    TaskHandle_t sample_task_handle;
};