# ESP32 Emergency Dash Camera Video Recorder

This project captures MJPEG frames from a camera module using the ESP32 and saves them as individual JPEGs on an SD card. The JPEGs are grouped as MJPEG sequence that can be encoded to other video format on the host using `ffmpeg`.

---

## 📁 Project Structure

```
.
├── components
│   ├── camera_handler         # Handles camera configuration and image capturing
│   │   ├── camera_handler.c
│   │   ├── camera_handler.h
│   │   └── CMakeLists.txt
│   └── sdcard                 # Manages SD card initialization and file system mount
│       ├── sdcard.c
│       ├── sdcard.h
│       └── CMakeLists.txt
├── main
│   ├── main.c                 # Application entry point
│   ├── CMakeLists.txt
│   └── idf_component.yml
├── CMakeLists.txt             # Top-level build configuration
├── partitions.csv             # Partition table
├── sdkconfig                  # ESP-IDF config
└── README.md                  # This file
```

---

## 🔧 Features

- Configurable camera capture resolution and quality
- Buffered MJPEG image storage
- Filesystem-based saving to `/sdcard/`
- Modular design (separate components for camera and SD card)
- Frame indexing and timestamping
- Circular buffer that on GPIO interrupt saves last 10 seconds of the driving session

---

## 🚀 Getting Started

### Prerequisites

- [ESP-IDF v5.0x](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
- An ESP32 board with camera support (e.g., ESP32-CAM)
- A formatted microSD card (FAT32)
- `ffmpeg` installed on your PC for post-processing

### Pin Configuration

Ensure your camera and SD card wiring matches the following (as set in `camera_handler.h` and `sdcard.h`):

#### Camera (OV2640)

```c
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
```

#### SD Card (SPI)

```c
#define SDCARD_MISO  (2)
#define SDCARD_MOSI  (15)
#define SDCARD_SCLK  (14)
#define SDCARD_CS    (13)
```

Also, to be able to store last 10 seconds of the video buffer, user needs to pull GPIO 12 high. GPIO pin can be manually remaped via the macro:

```c
#define CAMERA_HANDLER_GPIO_TRIGGER_PIN_NUM (12)
```
in the `camera_handler.h` header file.

---

## 🧠 Usage

### Initialization (inside `main.c`)

```c
#include "camera_handler.h"
#include "sdcard.h"

void app_main(void)
{
    sdcard_init();
    camera_handler_init();

#ifdef USE_DEMO
    demo_triggers();
#endif

    vTaskDelay(portMAX_DELAY);
}
```

### Output

Captured frames will be saved to:

```
/sdcard/crash_1.jpg
/sdcard/crash_2.jpg
...
```

---

## 🎥 Convert to AVI Using FFmpeg

Once you've captured the JPEG image sequence:

### 1. Copy Images from SD Card

Use a card reader or serial transfer tool to copy all `crash_XXXX.mjpeg` images to your computer into a single folder.

### 2. Convert to AVI (MJPEG Codec)

In the terminal, run:

```bash
ffmpeg -framerate 20 -i crash_%d.mjpeg -c:v mjpeg output.avi
```

> 📌 Notes:
> - `-framerate 20` should match `CAMERA_HANDLER_FRAMERATE_HZ`.
> - `%d` matches indexed filenames like `crash_1.mjpeg`.

### 3. Play the Video

```bash
ffplay output.avi
```

---

## 📜 License

This project is open-source under the MIT License.