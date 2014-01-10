#ifndef ORG_EEROS_LOGGER_PRETTY_HPP_
#define ORG_EEROS_LOGGER_PRETTY_HPP_

#include <sstream>
#include <string>
#include <math.h>

namespace eeros
{
	namespace logger
	{
		std::string pretty(double x)
		{
			std::stringstream out;

			bool npositive = (x >= 0);
			if (!npositive) x = -x;

			if (x < 1e-12)
			{
				return "   0.000 ";
			}

			double l = log10(x);
			bool positive = (l >= 0);
			if (!positive) l = -l;

			int r = (((int)l)/3)*3;
			if (!positive) r += 3;

			double b = pow(10,r);

			if (positive)
				x /= b;
			else
				x *= b;

			int left = (int)x;
			int right = (int)((x - left)*1000);


			if (left < 10)
				out << "  ";
			else if (left < 100)
				out << " ";

			if (npositive)
				out << ' ';
			else
				out << '-';

			out << left;
			out << ".";

			if (right < 10)
				out << "00";
			else if (right < 100)
				out << "0";

			out << right;

			if (positive)
			{
				switch(r)
				{
					case 0:
						out << " ";
						break;

					case 3:
						out << "k";
						break;

					case 6:
						out << "M";
						break;

					case 9:
						out << "G";
						break;

					case 12:
						out << "T";
						break;

					default:
						out << "e";
						if (positive) out << '+'; else out << '-';
						out << r;
					break;
				}
			}
			else
			{
				switch(r)
				{
					case 0:
						out << " ";
						break;

					case 3:
						out << "m";
						break;

					case 6:
						out << "u";
						break;

					case 9:
						out << "n";
						break;

					case 12:
						out << "p";
						break;

					default:
						out << "e";
						if (positive) out << '+'; else out << '-';
						out << r;
					break;
				}
			}

			return out.str();
		}
	}
}

#endif /* ORG_EEROS_LOGGER_PRETTY_HPP_ */
