#ifndef ORG_EEROS_TEST_CONTROL_UTILS_HPP_
#define ORG_EEROS_TEST_CONTROL_UTILS_HPP_

class Utils {
	public:
		/** compare
		 * @param tolerance relative to ref
		 */
		static bool compareApprox(double ref, double val, double tolerance) {
			double diff = (ref - val);
			if (diff < 0) diff = -diff;
			if (ref < 0) ref = -ref;
			return (diff < tolerance*ref);
		}
};
		
#endif // ORG_EEROS_TEST_CONTROL_UTILS_HPP_