#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

struct asset {
        const uint8_t* buf;
        size_t size;

        asset()
            : buf(nullptr), size(0) {}

        explicit asset(const char* text)
            : buf(reinterpret_cast<const uint8_t*>(text)), size(std::strlen(text)) {}
};

#define PATH_ASSET(name, text) static asset name(text)
