#ifndef LAB06_FSEQ_H
#define LAB06_FSEQ_H

#include <stdlib.h>
#include <stdio.h>

typedef struct {
    uint16_t channel_data_offset; // offset to the start of actual channel data
    uint8_t minor_version;
    uint8_t major_version;
    uint16_t index_to_first_variable_header;
    uint32_t channel_count_per_frame;
    uint32_t num_frames; // total number of frames in sequence
    uint8_t step_time_ms;
    enum {FSEQ_UNCOMPRESSED = 0, FSEQ_ZSTD, FSEQ_GZIP} compression_type; // Compression not supported currently
    uint16_t num_compression_blocks;
    uint8_t num_sparse_ranges;
    uint64_t uuid;

    FILE* sequence_file;
} fseq_sequence_t;

// Open filename and parse the header
// Returns a fseq_sequence_t initialized based on the header
// This function advances sequence_file to be at the beginning of the actual LED data
fseq_sequence_t open_and_parse_fseq_file(const char* filename);

// Reads data from sequence.sequence_file into a buffer
// The size of the buffer in bytes must be equal to sequence.channel_count_per_frame
int32_t get_next_led_buffer(uint8_t* buffer, fseq_sequence_t sequence);

// Destructor to close sequence_file
int32_t close_sequence(fseq_sequence_t sequence);

#endif //LAB06_FSEQ_H
