#include "fseq.h"
#include "sd_card_file_system.h"

#include "esp_log.h"
#include "sys/stat.h"
#include "errno.h"

#include <string.h>

// https://github.com/FalconChristmas/fpp/blob/master/docs/FSEQ_Sequence_File_Format.txt
#define HEADER_SIZE 32
#define OFFSET_CHANNEL_DATA_START 4
#define OFFSET_MINOR_VERSION 6
#define OFFSET_MAJOR_VERSION 7
#define OFFSET_VARIABLE_HEADER_INDEX 8
#define OFFSET_CHANNEL_COUNT_PER_FRAME 10
#define OFFSET_NUM_FRAMES 14
#define OFFSET_STEP_TIME 18
#define OFFSET_COMPRESSION 20
#define OFFSET_NUM_SPARSE_RANGES 22
#define OFFSET_SEQUENCE_UUID 24

#define HIGH_NIBBLE(byte) ((byte) >> 4)
#define LOW_NIBBLE(byte) (0x0F & (byte))

static const char* TAG = "FSEQ";

// TODO: enable compression

// Open file "filename" and parse the header
// Returns a fseq_sequence_t initialized based on the header
// This function advances sequence_file to be at the beginning of the actual LED data
fseq_sequence_t open_and_parse_fseq_file(const char* filename) {
    fseq_sequence_t result;
    char file_path[256]; // Ensure this buffer is large enough to hold the full path
    snprintf(file_path, sizeof(file_path), "%s/%s", MOUNT_POINT, filename);
    ESP_LOGI(TAG, "Opening file %s", file_path);

    result.sequence_file = fopen(file_path, "rb");
    if (result.sequence_file == NULL) {
        ESP_LOGE(TAG, "Failed to open file: %s", strerror(errno));
        struct stat st;
        if (stat(file_path, &st) == 0) {
            ESP_LOGI(TAG, "File exists. Permissions: %lo", st.st_mode);
        }
        else {
            ESP_LOGE(TAG, "Stat failed: %s", strerror(errno));
        }
    }
    ESP_LOGI(TAG, "File opened successfully");

    ESP_LOGI(TAG, "Parsing .fseq file header");
    uint8_t header_data[HEADER_SIZE];
    size_t bytes_read = fread(header_data, 1, sizeof(header_data), result.sequence_file);
    if (bytes_read < sizeof(header_data)) {
        if (feof(result.sequence_file)) {
            ESP_LOGI(TAG, "Reached end of file.");
        }
        else if (ferror(result.sequence_file)) {
            ESP_LOGE(TAG, "Error reading file.");
        }
    }
    result.channel_data_offset = *(uint16_t*)(header_data + OFFSET_CHANNEL_DATA_START);
    result.minor_version = *(uint8_t*)(header_data + OFFSET_MINOR_VERSION);
    result.major_version = *(uint8_t*)(header_data + OFFSET_MAJOR_VERSION);
    result.index_to_first_variable_header = *(uint16_t*)(header_data + OFFSET_VARIABLE_HEADER_INDEX);
    result.channel_count_per_frame = *(uint32_t*)(header_data + OFFSET_CHANNEL_COUNT_PER_FRAME);
    result.num_frames = *(uint32_t*)(header_data + OFFSET_NUM_FRAMES);
    result.step_time_ms = *(header_data + OFFSET_STEP_TIME);
    result.compression_type = HIGH_NIBBLE(*(header_data + OFFSET_COMPRESSION));
    result.num_compression_blocks = ((uint16_t)LOW_NIBBLE(*(header_data + OFFSET_COMPRESSION)) << 8) | *(header_data + OFFSET_COMPRESSION + 1);
    result.num_sparse_ranges = *(uint8_t*)(header_data + OFFSET_NUM_SPARSE_RANGES);
    result.uuid = *(uint64_t*)(header_data + OFFSET_SEQUENCE_UUID);

    if (result.major_version < 2) {
        ESP_LOGE(TAG, "Error: only fseq version 2.0 and greater are supported");
    }
    if (result.compression_type != FSEQ_UNCOMPRESSED) {
        ESP_LOGE(TAG, "Error: fseq file compression is not currently supported");
    }

    // Move the file pointer to the data location
    if (fseek(result.sequence_file, result.channel_data_offset, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "Error seeking to data location.");
    }
    ESP_LOGI(TAG, "Success parsing and initializing fseq file and sequence");
    return result;
}

int32_t get_next_led_buffer(uint8_t* buffer, fseq_sequence_t sequence) {
    size_t bytes_read = fread(buffer, 1, sequence.channel_count_per_frame, sequence.sequence_file);
    if (bytes_read < sizeof(buffer)) {
        if (feof(sequence.sequence_file)) {
            ESP_LOGI(TAG, "Reached end of file.");
            return 0;
        }
        else if (ferror(sequence.sequence_file)) {
            ESP_LOGE(TAG, "Error reading file.");
            return 0;
        }
    }
    return 1;
}

int32_t close_sequence(fseq_sequence_t sequence) {
    return fclose(sequence.sequence_file);
}
