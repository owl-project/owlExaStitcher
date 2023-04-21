// ======================================================================== //
// Copyright 2022-2022 Stefan Zellmann                                      //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <utility>

struct IniFile
{
  using key_type = std::string;
  struct value_type {
    std::string section;
    std::string value;
  };

  enum error_code {
    Ok, ParseError, NonExistent, MultipleEntries,
  };

  IniFile(std::string filename);

  bool good() const;

  void clear();

  error_code get_bool(std::string key, bool& value);

  error_code get_int8(std::string key, int8_t& value);
  error_code get_int16(std::string key, int16_t& value);
  error_code get_int32(std::string key, int32_t& value);
  error_code get_int64(std::string key, int64_t& value);

  error_code get_uint8(std::string key, uint8_t& value);
  error_code get_uint16(std::string key, uint16_t& value);
  error_code get_uint32(std::string key, uint32_t& value);
  error_code get_uint64(std::string key, uint64_t& value);

  error_code get_float(std::string key, float& value);
  error_code get_double(std::string key, double& value);
  error_code get_long_double(std::string key, long double& value);

  error_code get_string(std::string key, std::string& value, bool remove_quotes = false);

  error_code get_vec3i(std::string key, int32_t& x, int32_t& y, int32_t& z);
  error_code get_vec3ui(std::string key, uint32_t& x, uint32_t& y, uint32_t& z);
  error_code get_vec3f(std::string key, float& x, float& y, float& z);
  error_code get_vec3d(std::string key, double& x, double& y, double& z);

  // Versions with default values
  error_code get_bool(std::string key, bool& value, bool dflt);

  error_code get_int8(std::string key, int8_t& value, int8_t dflt);
  error_code get_int16(std::string key, int16_t& value, int16_t dflt);
  error_code get_int32(std::string key, int32_t& value, int32_t dflt);
  error_code get_int64(std::string key, int64_t& value, int64_t dflt);

  error_code get_uint8(std::string key, uint8_t& value, uint8_t dflt);
  error_code get_uint16(std::string key, uint16_t& value, uint16_t dflt);
  error_code get_uint32(std::string key, uint32_t& value, uint32_t dflt);
  error_code get_uint64(std::string key, uint64_t& value, uint64_t dflt);

  error_code get_float(std::string key, float& value, float dflt);
  error_code get_double(std::string key, double& value, double dflt);
  error_code get_long_double(std::string key, long double& value, long double dflt);

  error_code get_string(std::string key, std::string& value, std::string dflt,
                        bool remove_quotes = false);

  error_code get_vec3i(std::string key, int32_t& x, int32_t& y, int32_t& z,
                       int32_t dfltX, int32_t dfltY, int32_t dfltZ);
  error_code get_vec3ui(std::string key, uint32_t& x, uint32_t& y, uint32_t& z,
                        uint32_t dfltX, uint32_t dfltY, uint32_t dfltZ);
  error_code get_vec3f(std::string key, float& x, float& y, float& z,
                       float dfltX, float dfltY, float dfltZ);
  error_code get_vec3d(std::string key, double& x, double& y, double& z,
                       double dfltX, double dfltY, double dfltZ);

private:

  std::ifstream file_;
  std::multimap<key_type, value_type> entries_;
  bool good_ = false;

};

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

