/**
 * @file SpatzMultiplexer.h
 * @author paulnykiel
 * @date 25.01.20
 * A small handler to share the spatz between multiple simulator input filters
 */

#ifndef SIMULATOR_FILTERS_SPATZMULTIPLEXER_H
#define SIMULATOR_FILTERS_SPATZMULTIPLEXER_H

#include <Spatz/lib/Spatz.h>
#include <mutex>

class SpatzMultiplexer {
  public:
    static void submitSpatz(const env::Spatz &spatz);
    static auto getLastSpatz() -> env::Spatz;

  private:
    static env::Spatz lastSpatz;
    static std::mutex mutex;
};


#endif // INC_2019_SPATZMULTIPLEXER_H
