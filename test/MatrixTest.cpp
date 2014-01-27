#include <cstdlib>
#include <iostream>
using std::cout;
using std::endl;

#include <eeros/core/Matrix.hpp>
using namespace eeros::math;

template < int N, int M, typename T >
void print(Matrix<N,M,T> A, int indent = 1)
{
	for (int n = 1; n <= N; n++)
	{
		for (int i = 0; i < indent; i++) cout << '\t';
		for (int m = 1; m <= M; m++)
		{
			if (m > 1) cout << '\t';
			cout << A(n,m);
		}
		cout << endl;
	}
}

template < int N, int M, typename T >
void rot(int axis, Matrix<N,M,T> &A, T angle)
{
	if (axis == 0)
		A.rotx(angle);
	else if (axis == 1)
		A.roty(angle);
	else
		A.rotz(angle);
}

int main(int argc, char *argv[])
{
	int error = 0;
	int stage = 0;

	stage = 1;
	Matrix<3,3> m1;
	m1.zero();
	for (int n = 1; n <= 3; n++)
	{
		for (int m = 1; m <= 3; m++)
		{
			if (m1(n,m) != 0)
			{
				cout << stage << ": 0 expected, n = " << n << ", m = " << m << endl;
				error++;
			}
		}
	}

	stage = 2;
	Matrix<3,3> m2;
	m2.eye();
	double m2trace = m2.trace();
	if (m2trace != 3)
	{
		cout << stage << ": trace expected to be 3, m2.trace() = " << m2trace << endl;
		error++;
	}
	for (int n = 1; n <= 3; n++)
	{
		for (int m = 1; m <= 3; m++)
		{
			if (n == m)
			{
				if (m2(n,m) != 1)
				{
					cout << stage << ": 1 expected, n = " << n << ", m = " << m << endl;
					error++;
				}
			}
			else
			{
				if (m2(n,m) != 0)
				{
					cout << stage << ": 0 expected, n = " << n << ", m = " << m << endl;
					error++;
				}
			}
		}
	}
	if (m1 == m2)
	{
		cout << stage << ": == fails" << endl;
		error++;
	}
	if (!(m1 != m2))
	{
		cout << stage << ": != fails" << endl;
		error++;
	}

	stage = 3;
	Matrix<3,3> m3 = m1 + m2;
	if (m3 != m2)
	{
		cout << stage << ": eye expected" << endl;
		error++;
	}

	stage = 4;
	Matrix<3,3> m4;
	for (int j = 0; j < 3; j++)
	{
		for (int i = 0; i < 360; i++)
		{
			rot<3,3,double>(j, m4, i*3.14/180);
			Matrix<3,3> m4inv = !m4;
			Matrix<3,3> m4result = (m4 * m4inv);

			double sum = 0;
			for (int n = 1; n <= 3; n++)
			{
				for (int m = 1; m <= 3; m++)
				{
					double res = m4result(n,m);
					sum += (res > 0 ) ? res : -res;
				}
			}

//			if (sum != 3 || m4result != m2)
			if (sum != 3)
			{
				cout << stage << ": inverse fails, j = " << j << ", i = " << i << ", sum = " << sum << endl;
				error++;

//				cout << "  m4 " << endl;
//				print(m4);
//
//				cout << "  m4inv =" << endl;
//				print(m4inv);
//
//				cout << "  m4result =" << endl;
//				print(m4result);
//				break;
			}
		}
	}

	Matrix<2,4> m;
	std::cout << sizeof(m) << std::endl;
	
	if (error == 0)
		cout << "matrix test succeeded" << endl;
	else
		cout << "matrix test failed: " << error << " errors" << endl;

	return error;
}
