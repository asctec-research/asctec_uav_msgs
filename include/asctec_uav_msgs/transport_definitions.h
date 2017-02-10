/*
 * Copyright (C) 2016 Ascending Technologies GmbH, Germany
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
namespace asctec_uav_msgs {
#endif

enum {
  TRANSPORT_FLAG_ACK_REQUEST = 0x01,
  TRANSPORT_FLAG_ACK_RESPONSE = 0x02
};

#pragma pack(push,1) // <== make sure all declarations are within this and "#pragma pack(pop)" below !!!

typedef struct
{
  uint32_t id;
  uint16_t flags;
  uint16_t ackId;
} TransportHeader;

#pragma pack(pop)

#ifdef __cplusplus
} // end namespace asctec_uav_msgs
#endif
