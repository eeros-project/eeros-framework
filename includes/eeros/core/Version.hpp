#ifndef ORG_EEROS_CORE_VERSION_HPP_
#define ORG_EEROS_CORE_VERSION_HPP_

namespace eeros {

	class Version {
	public:
		static const int major;
		static const int minor;
		static const int patch;
		static const int tweak;
		static const char *string;
	};

};

#endif /* ORG_EEROS_CORE_VERSION_HPP_ */
