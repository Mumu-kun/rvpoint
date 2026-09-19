#pragma once

#include "io/streams/stream_types.h"

namespace rvpoint {

/**
 * @brief Ingestion Seam interface (ADR-0007).
 *
 * Decouples physical network transports and dataset replay from perception kernels.
 */
class StreamSource {
public:
    virtual ~StreamSource() = default;

    /**
     * @brief Poll for the next available frame.
     * @param[out] out Destination frame to populate.
     * @return true if a new frame was successfully polled, false otherwise.
     */
    virtual bool poll_frame(StreamFrame& out) = 0;
};

} // namespace rvpoint

