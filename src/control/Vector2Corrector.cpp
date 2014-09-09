#include <eeros/control/Vector2Corrector.hpp>
using namespace eeros::control;
using namespace eeros::math;

#include <stdio.h>


bool Vector2Corrector::load(const char *filename) {
	FILE *file = fopen(filename, "r");
	if (file == nullptr) return false;

	bool result = true;
	std::vector<map> maps;
	while (result) {
		Vector2 pr[3], pm[3];
		
		for (int i = 0; i < 3; i++) {
			double xr, yr, xm, ym;
			int r = fscanf(file, " ( %lf , %lf ) => ( %lf , %lf ) ; ", &xr, &yr, &xm, &ym);
			if (r != 4) {
				result = false;
				break;
			}
			pr[i][0] = xr;
			pr[i][1] = yr;
			pm[i][0] = xm;
			pm[i][1] = ym;
		}
		
		if (result) {
			Matrix<2,2> Tr;
			Tr(0, 0) = pr[1][0] - pr[0][0];
			Tr(1, 0) = pr[1][1] - pr[0][1];
			Tr(0, 1) = pr[2][0] - pr[0][0];
			Tr(1, 1) = pr[2][1] - pr[0][1];
			
			if (Tr.isInvertible()) {
				Matrix<2,2> Tm;
				Tm(0, 0) = pm[1][0] - pm[0][0];
				Tm(1, 0) = pm[1][1] - pm[0][1];
				Tm(0, 1) = pm[2][0] - pm[0][0];
				Tm(1, 1) = pm[2][1] - pm[0][1];
				
				Vector2Corrector::map map;
				map.Aref = pr[0];
				map.Tiref = !Tr;
				map.Amapped = pm[0];
				map.Tmapped = Tm;
				maps.push_back(map);
			}
		}
	}
	
	result = false;
	if (maps.size() > 0) {
		this->maps = maps;
		this->mapped.reserve(this->maps.size());
		result = true;
	}
	fclose(file);
	return result;
}

int Vector2Corrector::count() {
	return maps.size();
}

Vector2 Vector2Corrector::get(const Vector2 &in) {
	int n = 0;
	for (map &m : this->maps) {
		Vector2 l = m.Tiref * (in - m.Aref);
		if (l[0] < 0 || l[0] > 1) continue;
		if (l[1] < 0 || l[1] > 1) continue;
		if ((l[0] + l[1]) > 1) continue;
		mapped[n++] = m.Tmapped * l + m.Amapped;
	}
	if (n == 0) return in;
	Vector2 out;
	out = 0;
	for (int i = 0; i < n; i++) {
		out = out + mapped[i];
	}
	out = out / static_cast<double>(n);
	return out;
}
