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
extern "C" {
#endif

#pragma pack(push,1) // <== make sure all declarations are within this and "#pragma pack(pop)" below !!!

typedef union
{
  struct
  {
    int32_t x, y, z;
  };
  int32_t data[3];
} Vector3i;

typedef union
{
  struct
  {
    int32_t x, y;
  };
  int32_t data[2];
} Vector2i;

typedef union
{
  struct
  {
    float w, x, y, z;
  };
  float data[4];
} Quaternion;

typedef union
{
  struct
  {
    float x, y, z;
  };
  float data[3];
} Vector3f;

typedef union
{
  struct
  {
    float x, y;
  };
  float data[2];
} Vector2f;

#pragma pack(pop)

#ifdef __cplusplus
}
} // end namespace asctec_uav_msgs
#endif

