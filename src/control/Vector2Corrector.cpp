#include <eeros/control/Vector2Corrector.hpp>
using namespace eeros::control;
using namespace eeros::math;

#include <stdio.h>


bool Vector2Corrector::load(const char *filename) {
	bool result = false;
	double x, y;
	int nx, ny;
	FILE *file = fopen(filename, "r");
	if (file == nullptr) return false;

	std::vector<double> x_ref;
	std::vector<double> y_ref;
	std::vector<Vector2> mapped;
	
	int r = fscanf(file, "x: %lf", &x);
	if (r == 1) {
		x_ref.push_back(x);
		while (true) {
			r = fscanf(file, " , %lf", &x);
			if (r == 1) {
				x_ref.push_back(x);
			}
			else {
				break;
			}
		}
		nx = x_ref.size();
	
		r = fscanf(file, "y: %lf", &y);
		if (r == 1) {
			y_ref.push_back(y);
			while (true) {
				r = fscanf(file, " , %lf", &y);
				if (r == 1) {
					y_ref.push_back(y);
				}
				else {
					break;
				}
			}
			ny = y_ref.size();
			
			result = true;
			for (int j = 0; j < ny; j++) {
				for (int i = 0; i < nx; i++) {
					const char *format = ((i + 1) == nx) ? " ( %lf , %lf ) ; " : " ( %lf , %lf ) , ";
					r = fscanf(file, format, &x, &y);
					if (r != 2) {
						result = false;
						break;
					}
					mapped.push_back(Vector2(x, y));
				}
				if (r != 2) break;
			}
			if (result) {
				this->x_ref = x_ref;
				this->y_ref = y_ref;
				this->mapped = mapped;
			}
		}
	}
	fclose(file);
	return result;
}

Vector2 Vector2Corrector::get(const Vector2 &in) {
	int ix = find_interval(in[0], x_ref);
	if (ix < 0) return in;
	
	int iy = find_interval(in[1], y_ref);
	if (iy < 0) return in;
	
	double a = (in[0] - x_ref[ix]) / (x_ref[ix + 1] - x_ref[ix]);
	double b = (in[1] - y_ref[iy]) / (y_ref[iy + 1] - y_ref[iy]);
	
	Vector2 A = get_mapped(ix    , iy    );
	Vector2 B = get_mapped(ix + 1, iy    );
	Vector2 C = get_mapped(ix    , iy + 1);
	Vector2 D = get_mapped(ix + 1, iy + 1);
	
	Vector2 a1 = A + (B-A) * a;
	Vector2 a2 = C + (D-C) * a;
	Vector2 b1 = A + (C-A) * b;
	Vector2 b2 = B + (D-B) * b;
	
	Vector2 va = a2 - a1;
	Vector2 vb = b2 - b1;
	
	double len_va = sqrt(va[0]*va[0] + va[1]*va[1]);
	double len_vb = sqrt(vb[0]*vb[0] + vb[1]*vb[1]);
	
	va = va / len_va;
	vb = vb / len_vb;
	
	eeros::math::Matrix<2,2> G;
	G(0,0) = va[0];
	G(1,0) = va[1];
	G(0,1) = vb[0];
	G(1,1) = vb[1];
	
	if (!G.isInvertible()) return in;
	
	Vector2 h;
	h[0] = b1[0] - a1[0];
	h[1] = b1[1] - a1[1];
	
	Vector2 phi = !G * h;
	return a1 + phi[0] * va;
}

int Vector2Corrector::find_interval(double value, std::vector<double>& list) {
	int index = -1;
	int n = list.size();
	for (int i = 0; i < n; i++) {
		if (list[i] > value) break;
		index = i;
	}
	if ((index + 1) == n) return -1;
	return index;
}

Vector2 Vector2Corrector::get_mapped(int ix, int iy) {
	return mapped[ ix + iy * x_ref.size() ];
}
