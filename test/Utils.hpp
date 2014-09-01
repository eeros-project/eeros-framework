#ifndef ORG_EEROS_TEST_CONTROL_UTILS_HPP_
#define ORG_EEROS_TEST_CONTROL_UTILS_HPP_

class Utils {
	public:
		
		static bool compareApprox(double ref, double val, double tolerance) {
			if(ref == val) {
				return true;
			}
			else {
				double diff = abs(ref - val);
				return (diff < abs(tolerance));
			}
		}
		
		static double abs(double a) {
			if(a >= 0) {
				return a;
			}
			else {
				return -a;
			}
		}
};
		
#endif // ORG_EEROS_TEST_CONTROL_UTILS_HPP_
