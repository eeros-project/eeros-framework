#ifndef ORG_EEROS_CORE_VERSION_HPP_
#define ORG_EEROS_CORE_VERSION_HPP_

namespace eeros {

/**
 * This class contains the version string. It will be set, when compiling eeros.
 *
 * @since v0.6
 */  
class Version {
 public:
  static const int major; /**< This field holds the major number of the current eeros version. */
  static const int minor; /**< This field holds the minor number of the current eeros version. */
  static const int patch; /**< This field holds the patch number of the current eeros version. */
  static const int tweak; /**< This field holds the tweak number of the current eeros version. */
  static const char *string;  /**< This field holds the version string of the current eeros version. */
 };

};

#endif /* ORG_EEROS_CORE_VERSION_HPP_ */
