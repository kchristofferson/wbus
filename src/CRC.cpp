/*
 * CRC.cpp
 *
 *  Created on: Apr 2, 2015
 *      Author: kurt
 */

#include <stdint.h>
#include <cstdio>
#include "CRC.h"

namespace ur {

CRC::CRC() {
  // TODO Auto-generated constructor stub

}

CRC::~CRC() {
  // TODO Auto-generated destructor stub
}

uint8_t CRC::checksum(const void *data_, size_t pos_, size_t len_) {
  const char *data = (const char *) data_;

  int32_t r = 0;                    // do the math as an int
  uint8_t result = 0;               // return just the lsb byte

  for ( size_t i = pos_; i < pos_ + len_; i++ )
  {
    r += data[i];
  }

  if ( r > 0xFF )
    r = r & 0xFF;  // only want the lsb 8 bits

  result = (uint8_t) 0xFF - r;
  return (result);
}

} /* namespace ur */
