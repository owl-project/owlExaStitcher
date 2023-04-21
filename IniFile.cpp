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

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <sstream>
#include <vector>
#include <iostream>

#include "IniFile.h"

static std::string trim(std::string str, std::string ws = " \t")
{
  // Remove leading whitespace
  auto first = str.find_first_not_of(ws);

  // Only whitespace found
  if (first == std::string::npos)
  {
    return "";
  }

  // Remove trailing whitespace
  auto last = str.find_last_not_of(ws);

  // No whitespace found
  if (last == std::string::npos)
  {
    last = str.size() - 1;
  }

  // Skip if empty
  if (first > last)
  {
    return "";
  }

  // Trim
  return str.substr(first, last - first + 1);
}

static std::vector<std::string> string_split(std::string s, char delim)
{
  std::vector<std::string> result;

  std::istringstream stream(s);

  for (std::string token; std::getline(stream, token, delim); )
  {
    result.push_back(token);
  }

  return result;
}

static size_t count_whitespaces(std::string str)
{
  return std::count_if(
      str.begin(),
      str.end(),
      [](unsigned char c) { return std::isspace(c); }
      );
}

static std::string tolower(std::string str)
{
  std::transform(
      str.begin(),
      str.end(),
      str.begin(),
      [](unsigned char c) { return std::tolower(c); }
      );

  return str;
}

template <typename T>
inline bool as_T(std::string in, T& out)
{
  std::istringstream stream(in);
  return static_cast<bool>(stream >> out);
}

namespace detail {
template <typename Map>
inline IniFile::error_code get_string(Map const& map, std::string key, std::string& value)
{
  if (map.count(key) > 1)
  {
    return IniFile::MultipleEntries;
  }

  auto it = map.find(key);

  if (it == map.end())
  {
    return IniFile::NonExistent;
  }

  value = trim(it->second.value);

  return IniFile::Ok;
}
} // ::detail

template <typename Map, typename T>
inline IniFile::error_code get_T(Map const& map, std::string key, T& value)
{
  std::string str = "";
  IniFile::error_code err = detail::get_string(map, key, str);
  if (err != IniFile::Ok)
  {
    return err;
  }

  if (!as_T(str, value))
  {
    return IniFile::ParseError;
  }

  return IniFile::Ok;
}

template <typename Map, typename T>
inline IniFile::error_code get_T3(Map const& map, std::string key, T& x, T& y, T& z)
{
  std::string str = "";
  IniFile::error_code err = detail::get_string(map, key, str);
  if (err != IniFile::Ok)
  {
    return err;
  }

  // Also remove enclosing parentheses
  str = trim(str, "()");
  str = trim(str, "[]");
  str = trim(str, "{}");

  std::string delims = " ,;|";

  for (auto c : delims)
  {
    auto tokens = string_split(str, c);

    if (tokens.size() >= 3)
    {
      std::string xyz[3];
      int c = 0;
      for (auto t : tokens)
      {
        if (count_whitespaces(t) < t.size())
        {
          xyz[c++] = t;
        }
      }

      if (c == 3 && as_T(xyz[0], x) && as_T(xyz[1], y) && as_T(xyz[2], z))
      {
        return IniFile::Ok;
      }
    }
  }

  return IniFile::ParseError;
}


//-------------------------------------------------------------------------------------------------
// Interface
//

IniFile::IniFile(std::string filename)
  : file_(filename)
{
  if (!file_.good())
  {
    return;
  }

  std::string section = "";

  for (std::string line; std::getline(file_, line); )
  {
    line = trim(line);
    if (line.empty())
    {
      continue;
    }

    // Skip comments
    if (line[0] == ';' || line[0] == '#')
    {
      continue;
    }

    // Section
    if (line[0] == '[' && line[line.size() - 1] == ']')
    {
      section = trim(line.substr(1, line.size() - 2));
      continue;
    }

    // Parse key/value pairs
    auto p = string_split(line, '=');

    if (p.size() != 2)
    {
      // TODO: error handling
      continue;
    }

    std::string key = /*tolower*/(trim(p[0]));
    std::string value = /*tolower*/(trim(p[1]));

    entries_.emplace(std::make_pair(key, value_type{ section, value }));
  }

  good_ = true;
}

bool IniFile::good() const
{
  return good_;
}

void IniFile::clear()
{
  entries_.clear();
}

IniFile::error_code IniFile::get_bool(std::string key, bool& value)
{
  std::string str = "";
  IniFile::error_code err = detail::get_string(entries_, key, str);
  if (err != IniFile::Ok)
  {
    return err;
  }

  str = tolower(str);

  if (str == "1" || str == "true" || str == "on")
  {
    value = true;
    return Ok;
  }
  else if (str == "0" || str == "false" || str == "off")
  {
    value = false;
    return Ok;
  }
  else
  {
    return ParseError;
  }
}

IniFile::error_code IniFile::get_int8(std::string key, int8_t& value)
{
  return get_T(entries_, key, value);
}

IniFile::error_code IniFile::get_int16(std::string key, int16_t& value)
{
  return get_T(entries_, key, value);
}

IniFile::error_code IniFile::get_int32(std::string key, int32_t& value)
{
  return get_T(entries_, key, value);
}

IniFile::error_code IniFile::get_int64(std::string key, int64_t& value)
{
  return get_T(entries_, key, value);
}

IniFile::error_code IniFile::get_uint8(std::string key, uint8_t& value)
{
  return get_T(entries_, key, value);
}

IniFile::error_code IniFile::get_uint16(std::string key, uint16_t& value)
{
  return get_T(entries_, key, value);
}

IniFile::error_code IniFile::get_uint32(std::string key, uint32_t& value)
{
  return get_T(entries_, key, value);
}

IniFile::error_code IniFile::get_uint64(std::string key, uint64_t& value)
{
  return get_T(entries_, key, value);
}

IniFile::error_code IniFile::get_float(std::string key, float& value)
{
  return get_T(entries_, key, value);
}

IniFile::error_code IniFile::get_double(std::string key, double& value)
{
  return get_T(entries_, key, value);
}

IniFile::error_code IniFile::get_long_double(std::string key, long double& value)
{
  return get_T(entries_, key, value);
}

IniFile::error_code IniFile::get_string(std::string key, std::string& value, bool remove_quotes)
{
  error_code err = detail::get_string(entries_, key, value);

  if (err == Ok && remove_quotes)
  {
    // TODO: check that we removed *2* (and both the same) quotation marks
    value = trim(value, "'");
    value = trim(value, "\"");
  }

  return err;
}

IniFile::error_code IniFile::get_vec3i(std::string key, int32_t& x, int32_t& y, int32_t& z)
{
  return get_T3(entries_, key, x, y, z);
}

IniFile::error_code IniFile::get_vec3ui(std::string key, uint32_t& x, uint32_t& y, uint32_t& z)
{
  return get_T3(entries_, key, x, y, z);
}

IniFile::error_code IniFile::get_vec3f(std::string key, float& x, float& y, float& z)
{
  return get_T3(entries_, key, x, y, z);
}

IniFile::error_code IniFile::get_vec3d(std::string key, double& x, double& y, double& z)
{
  return get_T3(entries_, key, x, y, z);
}


// Versions with default values

IniFile::error_code IniFile::get_int8(std::string key, int8_t& value, int8_t dflt)
{
  value = dflt;
  return get_int8(key, value);
}

IniFile::error_code IniFile::get_int16(std::string key, int16_t& value, int16_t dflt)
{
  value = dflt;
  return get_int16(key, value);
}

IniFile::error_code IniFile::get_int32(std::string key, int32_t& value, int32_t dflt)
{
  value = dflt;
  return get_int32(key, value);
}

IniFile::error_code IniFile::get_int64(std::string key, int64_t& value, int64_t dflt)
{
  value = dflt;
  return get_int64(key, value);
}

IniFile::error_code IniFile::get_uint8(std::string key, uint8_t& value, uint8_t dflt)
{
  value = dflt;
  return get_uint8(key, value);
}

IniFile::error_code IniFile::get_uint16(std::string key, uint16_t& value, uint16_t dflt)
{
  value = dflt;
  return get_uint16(key, value);
}

IniFile::error_code IniFile::get_uint32(std::string key, uint32_t& value, uint32_t dflt)
{
  value = dflt;
  return get_uint32(key, value);
}

IniFile::error_code IniFile::get_uint64(std::string key, uint64_t& value, uint64_t dflt)
{
  value = dflt;
  return get_uint64(key, value);
}

IniFile::error_code IniFile::get_float(std::string key, float& value, float dflt)
{
  value = dflt;
  return get_float(key, value);
}

IniFile::error_code IniFile::get_double(std::string key, double& value, double dflt)
{
  value = dflt;
  return get_double(key, value);
}

IniFile::error_code IniFile::get_long_double(std::string key, long double& value, long double dflt)
{
  value = dflt;
  return get_long_double(key, value);
}

IniFile::error_code IniFile::get_string(std::string key, std::string& value, std::string dflt,
                                        bool remove_quotes)
{
  value = dflt;
  return get_string(key, value, remove_quotes);
}

IniFile::error_code IniFile::get_vec3i(std::string key, int32_t& x, int32_t& y, int32_t& z,
                                       int32_t dfltX, int32_t dfltY, int32_t dfltZ)
{
  x = dfltX; y = dfltY; z = dfltZ;
  return get_vec3i(key, x, y, z);
}

IniFile::error_code IniFile::get_vec3ui(std::string key, uint32_t& x, uint32_t& y, uint32_t& z,
                                        uint32_t dfltX, uint32_t dfltY, uint32_t dfltZ)
{
  x = dfltX; y = dfltY; z = dfltZ;
  return get_vec3ui(key, x, y, z);
}

IniFile::error_code IniFile::get_vec3f(std::string key, float& x, float& y, float& z,
                                       float dfltX, float dfltY, float dfltZ)
{
  x = dfltX; y = dfltY; z = dfltZ;
  return get_vec3f(key, x, y, z);
}

IniFile::error_code IniFile::get_vec3d(std::string key, double& x, double& y, double& z,
                                       double dfltX, double dfltY, double dfltZ)
{
  x = dfltX; y = dfltY; z = dfltZ;
  return get_vec3d(key, x, y, z);
}

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

