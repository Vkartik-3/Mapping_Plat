// Copyright (C) 2025 Kartik Vadhawana
//
// Header-only CRC32 (IEEE 802.3, polynomial 0xEDB88320) implementation for
// LiDAR/sensor frame integrity validation in the HD map pipeline.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_UTIL_CRC32_HPP
#define MAP_MATCHING_2_UTIL_CRC32_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace map_matching_2::util {

    // Reflected CRC32 (polynomial 0xEDB88320), the same variant used by
    // zlib, PNG and Ethernet. init = 0xFFFFFFFF, final xor = 0xFFFFFFFF.
    class crc32 {

        // Compile-time generation of the 256-entry lookup table so the
        // header stays fully self-contained with no .cpp and no static init.
        // Exposed as a constexpr function (not a data member) so the whole
        // class remains usable in a constant expression across compilers.
        static constexpr std::array<std::uint32_t, 256> table() noexcept {
            std::array<std::uint32_t, 256> t{};
            for (std::uint32_t i = 0; i < 256; ++i) {
                std::uint32_t c = i;
                for (int k = 0; k < 8; ++k) {
                    c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
                }
                t[i] = c;
            }
            return t;
        }

    public:
        // Incremental update. Feed successive byte ranges, passing the
        // previous return value back in as `crc` (start with 0).
        [[nodiscard]] static constexpr std::uint32_t update(
                std::uint32_t crc, const void *data, std::size_t length) noexcept {
            constexpr auto lut = table();
            std::uint32_t c = crc ^ 0xFFFFFFFFu;
            const auto *bytes = static_cast<const std::uint8_t *>(data);
            for (std::size_t i = 0; i < length; ++i) {
                c = lut[(c ^ bytes[i]) & 0xFFu] ^ (c >> 8);
            }
            return c ^ 0xFFFFFFFFu;
        }

        // One-shot convenience over a raw byte buffer.
        [[nodiscard]] static constexpr std::uint32_t compute(
                const void *data, std::size_t length) noexcept {
            return update(0u, data, length);
        }

        [[nodiscard]] static constexpr std::uint32_t compute(
                std::string_view sv) noexcept {
            return compute(sv.data(), sv.size());
        }

    };

}

#endif //MAP_MATCHING_2_UTIL_CRC32_HPP
