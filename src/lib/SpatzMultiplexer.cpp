/**
 * @file SpatzMultiplexer.cpp
 * @author paulnykiel
 * @date 25.01.20
 */

#include "SpatzMultiplexer.h"

env::Spatz SpatzMultiplexer::lastSpatz;
std::mutex SpatzMultiplexer::mutex;

void SpatzMultiplexer::submitSpatz(const env::Spatz &spatz) {
    std::lock_guard<std::mutex> lockGuard{mutex};
    lastSpatz = spatz;
}

auto SpatzMultiplexer::getLastSpatz() -> env::Spatz {
    std::lock_guard<std::mutex> lockGuard{mutex};
    return lastSpatz;
}
