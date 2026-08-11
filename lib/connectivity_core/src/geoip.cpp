#include "connectivity/geoip.hpp"

#include <cmath>
#include <cstdlib>
#include <optional>
#include <string>

namespace connectivity {
namespace {

bool isJsonDelimiter(char character) {
  return character == ',' || character == '}' || character == ']' || character == ' ' ||
         character == '\t' || character == '\r' || character == '\n';
}

std::optional<std::string_view> findMemberValue(std::string_view response,
                                                std::string_view memberName) {
  const std::string quotedName = std::string("\"") + std::string(memberName) + "\"";
  const std::size_t member = response.find(quotedName);
  const bool memberMissing = member == std::string_view::npos;
  if (memberMissing) {
    return std::nullopt;
  }

  const std::size_t colon = response.find(':', member + quotedName.size());
  const bool colonMissing = colon == std::string_view::npos;
  if (colonMissing) {
    return std::nullopt;
  }

  std::size_t valueStart = colon + 1;
  while (valueStart < response.size()) {
    const char character = response[valueStart];
    const bool isWhitespace =
        character == ' ' || character == '\t' || character == '\r' || character == '\n';
    if (!isWhitespace) {
      break;
    }
    ++valueStart;
  }
  const bool valueMissing = valueStart == response.size();
  if (valueMissing) {
    return std::nullopt;
  }
  return response.substr(valueStart);
}

std::optional<double> parseNumberMember(std::string_view response, std::string_view memberName) {
  const std::optional<std::string_view> value = findMemberValue(response, memberName);
  const bool valueMissing = !value.has_value();
  if (valueMissing) {
    return std::nullopt;
  }

  std::size_t length = 0;
  const bool hasSign = (*value)[length] == '-' || (*value)[length] == '+';
  if (hasSign) {
    ++length;
  }

  std::size_t integerDigits = 0;
  while (length < value->size() && (*value)[length] >= '0' && (*value)[length] <= '9') {
    ++length;
    ++integerDigits;
  }
  const bool integerMissing = integerDigits == 0;
  if (integerMissing) {
    return std::nullopt;
  }

  const bool hasFraction = length < value->size() && (*value)[length] == '.';
  if (hasFraction) {
    ++length;
    std::size_t fractionDigits = 0;
    while (length < value->size() && (*value)[length] >= '0' && (*value)[length] <= '9') {
      ++length;
      ++fractionDigits;
    }
    const bool fractionMissing = fractionDigits == 0;
    if (fractionMissing) {
      return std::nullopt;
    }
  }

  const bool hasExponent =
      length < value->size() && ((*value)[length] == 'e' || (*value)[length] == 'E');
  if (hasExponent) {
    ++length;
    const bool exponentHasSign =
        length < value->size() && ((*value)[length] == '-' || (*value)[length] == '+');
    if (exponentHasSign) {
      ++length;
    }
    std::size_t exponentDigits = 0;
    while (length < value->size() && (*value)[length] >= '0' && (*value)[length] <= '9') {
      ++length;
      ++exponentDigits;
    }
    const bool exponentMissing = exponentDigits == 0;
    if (exponentMissing) {
      return std::nullopt;
    }
  }

  const bool delimiterMissing = length < value->size() && !isJsonDelimiter((*value)[length]);
  if (delimiterMissing) {
    return std::nullopt;
  }

  const std::string token(value->substr(0, length));
  char* parsedEnd = nullptr;
  const double parsed = std::strtod(token.c_str(), &parsedEnd);
  const bool wholeTokenParsed = parsedEnd == token.c_str() + token.size();
  const bool parsedValueIsValid = wholeTokenParsed && std::isfinite(parsed);
  if (!parsedValueIsValid) {
    return std::nullopt;
  }
  return parsed;
}

bool responseSucceeded(std::string_view response) {
  const std::optional<std::string_view> value = findMemberValue(response, "success");
  const bool valueMissing = !value.has_value();
  if (valueMissing) {
    return false;
  }
  constexpr std::string_view kTrue = "true";
  const bool beginsWithTrue = value->substr(0, kTrue.size()) == kTrue;
  if (!beginsWithTrue) {
    return false;
  }

  const bool endsAfterTrue = value->size() == kTrue.size();
  if (endsAfterTrue) {
    return true;
  }
  return isJsonDelimiter((*value)[kTrue.size()]);
}

} // namespace

bool isValidLocation(double latitudeDegrees, double longitudeEastDegrees) {
  const bool valuesAreFinite =
      std::isfinite(latitudeDegrees) && std::isfinite(longitudeEastDegrees);
  const bool latitudeIsValid = latitudeDegrees >= -90.0 && latitudeDegrees <= 90.0;
  const bool longitudeIsValid = longitudeEastDegrees >= -180.0 && longitudeEastDegrees <= 180.0;
  return valuesAreFinite && latitudeIsValid && longitudeIsValid;
}

GeoIpLocation parseGeoIpResponse(std::string_view response) {
  GeoIpLocation result;
  const bool requestSucceeded = responseSucceeded(response);
  if (!requestSucceeded) {
    return result;
  }

  const std::optional<double> latitude = parseNumberMember(response, "latitude");
  const std::optional<double> longitude = parseNumberMember(response, "longitude");
  const bool coordinatesPresent = latitude.has_value() && longitude.has_value();
  if (!coordinatesPresent) {
    return result;
  }

  const bool coordinatesAreValid = isValidLocation(*latitude, *longitude);
  if (!coordinatesAreValid) {
    return result;
  }

  result.latitudeDegrees = *latitude;
  result.longitudeEastDegrees = *longitude;
  result.valid = true;
  return result;
}

std::array<char, 17> stackchanHostName(const std::array<std::uint8_t, 6>& macAddress) {
  constexpr char kPrefix[] = "stackchan-";
  constexpr char kHex[] = "0123456789abcdef";
  std::array<char, 17> result{};
  for (std::size_t index = 0; index < sizeof(kPrefix) - 1; ++index) {
    result[index] = kPrefix[index];
  }
  for (std::size_t byte = 3; byte < macAddress.size(); ++byte) {
    const std::size_t output = sizeof(kPrefix) - 1 + (byte - 3) * 2;
    result[output] = kHex[macAddress[byte] >> 4];
    result[output + 1] = kHex[macAddress[byte] & 0x0F];
  }
  return result;
}

} // namespace connectivity
