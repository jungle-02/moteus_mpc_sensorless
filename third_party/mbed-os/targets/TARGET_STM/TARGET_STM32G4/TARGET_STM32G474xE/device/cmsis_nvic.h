// Copyright 2019 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef MBED_CMSIS_NVIC_H
#define MBED_CMSIS_NVIC_H

// CORE: 16 vectors = 64 bytes from 0x00 to 0x3F
// MCU Peripherals: 102 vectors = 408 bytes from 0x40 to 0x1d8
// Total: 118 vectors = 472 bytes (0x1d8) to be reserved in RAM
#define NVIC_NUM_VECTORS        118
#define NVIC_RAM_VECTOR_ADDRESS 0x20000000    // Vectors positioned at start of RAM

#endif
