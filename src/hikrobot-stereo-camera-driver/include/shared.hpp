#include <pthread.h>
#include <stdint.h>

namespace shared{
    constexpr int idx_base = 1234;
    constexpr int sharedH = 480;
    constexpr int sharedW = 640;
    constexpr int shared_img_size = sharedH * sharedW * 3 * sizeof(uint8_t);

    struct shared_struct{
        pthread_mutex_t shared_mutex;
        uint8_t shared_img[shared_img_size];
    };
}