#ifndef ORG_EEROS_CONTROL_ZTRANSFERFUNCTION_HPP_
#define ORG_EEROS_CONTROL_ZTRANSFERFUNCTION_HPP_

#include <eeros/core/System.hpp>
#include <eeros/control/Block1i1o.hpp>
#include <eeros/math/Fraction.hpp>
#include <vector>

namespace eeros {
	namespace control {

		template < int ORDER >
		class ZTransferFunction: public eeros::control::Block1i1o<> {
			static constexpr int N = (ORDER + 1);
			
			public:
				ZTransferFunction(const eeros::math::Fraction<ORDER>& copy) :
					fraction(copy) { }
					
				ZTransferFunction(const std::vector<double> a, const std::vector<double> b) :
					fraction(b,a) { }
				
				static ZTransferFunction<1> PT1(double Ts, double K, double T1) {
					return ZTransferFunction<1>( { Ts+T1, -T1 }, { K*Ts } );
				}
				
				static ZTransferFunction<1> D(double Ts, double Tv) {
					return ZTransferFunction<1>( { Ts }, { Tv, -Tv } );
				}
				
				static ZTransferFunction<1> I(double Ts, double Tn) {
					return ZTransferFunction<1>( { Tn, -Tn }, { Ts } );
				}
				
				static ZTransferFunction<1> DT1(double Ts, double Tv, double T1) {
					return ZTransferFunction<1>( { Ts+T1, -T1 }, { Tv, -Tv } );
				}
				
				static ZTransferFunction<2> PID(double Ts, double Kp, double Tn, double Tv, double Tv1) {
					return (I(Ts, Tn) + DT1(Ts, Tv, Tv1) + 1)*Kp;
				}
				
				template < int RORDER >
				ZTransferFunction<ORDER+RORDER> operator *(const ZTransferFunction<RORDER> right) {
					return ZTransferFunction<ORDER+RORDER>(fraction * right.fraction);
				}
			
				ZTransferFunction<ORDER> operator *(double right) {
					return ZTransferFunction<ORDER>(fraction * right);
				}
				
				template < int RORDER >
				ZTransferFunction<ORDER+RORDER> operator +(const ZTransferFunction<RORDER> right) {
					return ZTransferFunction<ORDER+RORDER>(fraction + right.fraction);
				}
			
				ZTransferFunction<ORDER> operator +(double right) {
					return ZTransferFunction<ORDER>(fraction + right);
				}
				
				virtual void run() {
					last_in[0] = in.getSignal().getValue();
					last_out[0] = last_in[0] * fraction.numerator.c[0];
					for (int i = 1; i < N; i++) {
						last_out[0] += (last_in[i] * fraction.numerator.c[i] - last_out[i] * fraction.denominator.c[i]);
					}
					last_out[0] /= fraction.denominator.c[0];
					
					out.getSignal().setValue(last_out[0]);
					out.getSignal().setTimestamp(eeros::System::getTimeNs());
					
					for (int i = (N - 1); i >= 0; i--) {
						last_in[i] = last_in[i - 1];
						last_out[i] = last_out[i - 1];
					}
				}
				
			private:
				eeros::math::Fraction<ORDER> fraction;
				double last_in[N];
				double last_out[N];
		};
	};
};

#endif /* ORG_EEROS_CONTROL_ZTRANSFERFUNCTION_HPP_ */
