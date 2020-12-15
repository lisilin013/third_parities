/*
 * @file KeyGenerator.cpp
 * @date Aug 19, 2014
 * @author Paul Furgale
 */

#include <atomic>

#include <mincurves/KeyGenerator.hpp>

namespace curves {

size_t KeyGenerator::getNextKey() {
  static std::atomic<size_t> key(0u);
  size_t new_key = key++;
  return new_key;
}


} // namespace curves
