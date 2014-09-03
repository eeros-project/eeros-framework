#include <eeros/control/MouseInput.hpp>
#include <eeros/math/Matrix.hpp>

using namespace eeros::control;

MouseInput::MouseInput(std::string dev) {
        setInitPos(0, 0, 0, 0);
        j.open(dev.c_str());
        t = new std::thread([this](){ this->j.loop(); });
        first = true;
}

MouseInput::~MouseInput() {
        delete t;
        j.close();
}

void MouseInput::run() {
	if (first) {
		x = axisScale_x * -j.current.axis.y;
		y = axisScale_y * -j.current.axis.x;
		z = axisScale_z * -j.current.axis.z;
		r = axisScale_r * -j.current.axis.r;
		first = false;
	}

	double v;

	v = axisScale_x * j.current.axis.y;
	if ((x + v) < min_x)
		x = (min_x - v);
	else if ((x + v) > max_x)
		x = (max_x - v);
	v += x;
	outX.getSignal().setValue(v);
	v = axisScale_y * j.current.axis.x;
	if ((y + v) < min_y)
		y = (min_y - v);
	else if ((y + v) > max_y)
		y = (max_y - v);
	v += y;
	outY.getSignal().setValue(v);

	v = axisScale_z * j.current.axis.z;
	if ((z + v) < min_z)
		z = (min_z - v);
	else if ((z + v) > max_z)
		z = (max_z - v);
	v += z;
	outZ.getSignal().setValue(v);
	v = axisScale_r * j.current.axis.r;
	if ((r + v) < min_r)
		r = (min_r - v);
	else if ((r + v) > max_r)
		r = (max_r - v);
	v += r;
	outR.getSignal().setValue(v);

	uint64_t ts = eeros::System::getTimeNs();
	outX.getSignal().setTimestamp(ts);
	outY.getSignal().setTimestamp(ts);
	outZ.getSignal().setTimestamp(ts);
	outR.getSignal().setTimestamp(ts);

	out.getSignal().setValue(eeros::math::Matrix<4>{ 
		outX.getSignal().getValue(),
		outY.getSignal().getValue(),
		outZ.getSignal().getValue(),
		outR.getSignal().getValue(),
	});
	out.getSignal().setTimestamp(ts);
}

Output<double>& MouseInput::getOutX() {
        return outX;
}
Output<double>& MouseInput::getOutY() {
        return outY;
}

Output<double>& MouseInput::getOutZ() {
        return outZ;
}

Output<double>& MouseInput::getOutR() {
        return outR;
}

eeros::control::Output<eeros::math::Matrix<4>>& MouseInput::getOut() {
	return out;
}

void MouseInput::setInitPos(double x, double y, double z, double r) {
	reset(x, y, z, r);
	outX.getSignal().setValue(x);
	outY.getSignal().setValue(y);
	outZ.getSignal().setValue(z);
	outR.getSignal().setValue(r);
	out.getSignal().setValue(eeros::math::Matrix<4>{ x, y, z, r });
}

void MouseInput::setInitPos(eeros::math::Matrix<4> pos) {
	setInitPos(pos[0], pos[1], pos[2], pos[3]);
}

void MouseInput::reset(double x, double y, double z, double r) {
        this->x = (x - axisScale_x * j.current.axis.y);
        this->y = (y - axisScale_y * j.current.axis.x);
        this->z = (z - axisScale_z * j.current.axis.z);
        this->r = (r - axisScale_r * j.current.axis.r);
}
