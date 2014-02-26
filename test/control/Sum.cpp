#include <eeros/core/Runnable.hpp>
#include <eeros/control/Sum.hpp>
using namespace eeros::control;

#include <iostream>
#include <fstream>
using namespace std;

template <int N, typename T = double>
class BlockTest {
	public:
		BlockTest() {
			for (int i = 0; i < N; i++) {
				uut.getIn(i).connect(in[i]);
			}
		}

		int run(const char* filepath)
		{
			ifstream file(filepath);
			
			if (!file.is_open()) return -2;
			
			int line = 0;
			int error = 0;
			
			while (!file.eof())
			{
				line++;
				double in0, in1, out;
				file >> in0 >> in1 >> out;
				in[0].getSignal().setValue(in0);
				in[1].getSignal().setValue(in1);
// 				for (int i = 0; i < N; i++) {
// 					in[i].getSignal().setValue(0.046403);
// 				}
				uut.run();
				
				double diff = (out - uut.getOut().getSignal().getValue());
				if (diff > 0.001*out || diff < -0.001*out)
				{
					error++;
					cout << "line " << line << " expecting " << out << " calculated " << uut.getOut().getSignal().getValue() << endl;
				}
			}
			
			file.close();
			return error;
		}
		
	protected:
		Output<T> in[N];
		Sum<N> uut;
};

int main(int argc, char* argv[]) {
	BlockTest<2> tester;
	
	if (argc == 2) {
		return tester.run(argv[1]);
	}
	
	return -1;
}