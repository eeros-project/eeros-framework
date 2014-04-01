#ifndef ORG_EEROS_MATH_FRACTION_HPP_
#define ORG_EEROS_MATH_FRACTION_HPP_

#include <vector>

namespace eeros {
	namespace math {
		
		template < int ORDER >
		class Polynome;

		template < int ORDER >
		class Fraction {
		public:
			Fraction() { }
			
			explicit Fraction(const std::vector<double> n) :
				numerator(n) { }
				
			Fraction(const std::vector<double>& n, const std::vector<double>& d) :
				numerator(n), denominator(d) { }
			
			template < int RORDER >
			Fraction<ORDER+RORDER> operator *(const Fraction<RORDER> right) {
				Fraction<ORDER+RORDER> p;
				p.numerator = numerator * right.numerator;
				p.denominator = denominator * right.denominator;
				return p;
			}
	
			Fraction<ORDER> operator *(double k) {
				Fraction<ORDER> f;
				f.numerator = numerator*k;
				f.denominator = denominator;
				return f;
			}
			
			template < int RORDER >
			Fraction<ORDER+RORDER> operator +(const Fraction<RORDER> right) {
				Fraction<ORDER+RORDER> p;
				p.numerator = numerator * right.denominator + denominator * right.numerator;
				p.denominator = denominator * right.denominator;
				return p;
			}
			
			Fraction<ORDER> operator +(const double right) {
				Fraction<ORDER> p;
				p.numerator = numerator + denominator * right;
				p.denominator = denominator;
				return p;
			}
			
			Polynome<ORDER> numerator;
			Polynome<ORDER> denominator;
		};

		template < int ORDER >
		class Polynome {
		public:
			static constexpr int N = (ORDER + 1);
			static constexpr int MAX(int x, int y) { return (x > y) ? x : y; }
			
			Polynome() {
				for (int i = 0; i < N; i++)
					this->c[i] = 0;
			}
			
			explicit Polynome(const std::vector<double>& c) {
				int n = c.size();

				for (int i = 0; i < N; i++)
					if (i < n)
						this->c[i] = c[i];
					else
						this->c[i] = 0;
			}
			
			template < int RORDER >
			explicit Polynome(const Polynome<RORDER>& copy) {
				int n = (RORDER+1);

				for (int i = 0; i < N; i++)
					if (i < n)
						this->c[i] = copy.c[i];
					else
						this->c[i] = 0;
			}
			
			template < int RORDER >
			Polynome<ORDER+RORDER> operator *(const Polynome<RORDER> right) {
				constexpr int M = Polynome<RORDER>::N;
				Polynome<ORDER+RORDER> p;
				
				for (int i = 0; i < N; i++)
					for (int j = 0; j < M; j++)
						p.c[i+j] += c[i]*right.c[j];
				
				return p;
			}
	
			Polynome<ORDER> operator *(double k) {
				Polynome<ORDER> p;
				
				for (int i = 0; i < N; i++)
						p.c[i] += c[i]*k;
				
				return p;
			}
			
			template < int RORDER >
			Polynome<MAX(ORDER,RORDER)> operator +(const Polynome<RORDER> right) {
				constexpr int M = Polynome<RORDER>::N;
				constexpr int n = Polynome<MAX(ORDER,RORDER)>::N;
				Polynome<MAX(ORDER,RORDER)> p;
				
				for (int i = 0; i < n; i++) {
					if (i < M && i < N)
						p.c[i] = c[i] + right.c[i];
					else if (i < N)
						p.c[i] = c[i];
					else if (i < M)
						p.c[i] = right.c[i];
				}
				
				return p;
			}
			
			Polynome<ORDER> operator +(double right) {
				Polynome<ORDER> p = *this;
				p.c[0] = c[0] + right;
				return p;
			}
			
			template < int RORDER >
			Fraction<MAX(ORDER,RORDER)> operator /(const Polynome<RORDER> right) {
				Fraction<MAX(ORDER,RORDER)> p;
				p.numerator = Polynome<MAX(ORDER,RORDER)>(*this);
				p.denominator = Polynome<MAX(ORDER,RORDER)>(right);
				return p;
			}
			
			double c[N];
		};
	}
}


#endif // ORG_EEROS_MATH_FRACTION_HPP_
