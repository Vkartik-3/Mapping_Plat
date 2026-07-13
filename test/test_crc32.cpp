// Copyright (C) 2025 Kartik Vadhawana
#include <gtest/gtest.h>

#include "util/crc32.hpp"

using map_matching_2::util::crc32;

TEST(Crc32, KnownVectorCheck) {
    // Standard CRC32/ISO-HDLC check value for "123456789".
    EXPECT_EQ(crc32::compute(std::string_view("123456789")), 0xCBF43926u);
}

TEST(Crc32, EmptyInputIsZero) {
    EXPECT_EQ(crc32::compute(std::string_view("")), 0x00000000u);
}

TEST(Crc32, SingleCharKnownVector) {
    // CRC32 of "a".
    EXPECT_EQ(crc32::compute(std::string_view("a")), 0xE8B7BE43u);
}

TEST(Crc32, IncrementalMatchesOneShot) {
    const std::string a = "hello, ";
    const std::string b = "world";
    const std::string whole = a + b;
    std::uint32_t inc = crc32::update(0u, a.data(), a.size());
    inc = crc32::update(inc, b.data(), b.size());
    EXPECT_EQ(inc, crc32::compute(whole));
}

TEST(Crc32, Deterministic) {
    const std::string s = "the quick brown fox";
    EXPECT_EQ(crc32::compute(s), crc32::compute(s));
}
