#include <eeros/core/Version.hpp>
#include <eeros/config.hpp>

using namespace eeros;

const int Version::major = EEROS_VERSION_MAJOR;
const int Version::minor = EEROS_VERSION_MINOR;
const int Version::patch = EEROS_VERSION_PATCH;
const int Version::tweak = EEROS_VERSION_TWEAK;
const char *Version::string = EEROS_VERSION;
