#ifndef ORG_EEROS_EXAMPLES_SIMPLEMOTORCONTROLER_COMEDIENCODER_HPP_
#define ORG_EEROS_EXAMPLES_SIMPLEMOTORCONTROLER_COMEDIENCODER_HPP_

#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>
#include <string>
#include <comedilib.h>

#define ENCODER_DEV "/dev/comedi0"
#define ENCODER_SUBDEV 11

namespace eeros {
	namespace examples {
		namespace simpleMotorController {

			class ComediEncoder: public control::Block1o {
			public:
				ComediEncoder();
				virtual ~ComediEncoder();

				virtual void run();

			private:
				int ni_gpct_start_encoder(comedi_t *device, unsigned subdevice, unsigned int initial_value, int a, int b, int z);
				
				comedi_t* it;
				std::string deviceName;
				int subDevice;
				int channelA, channelB, channelZ;
				lsampl_t data;
			};

		};
	};
};

#endif /* ORG_EEROS_EXAMPLES_SIMPLEMOTORCONTROLER_COMEDIENCODER_HPP_ */
