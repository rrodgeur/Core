#include <arm_math.h>
#include <trigo.h>
class Chirp {
public:
	float32_t fmin;
	float32_t fmax;
	float32_t duration;
	float32_t Ts;
	Chirp(float32_t fmin, float32_t fmax, float32_t duration): fmin(fmin), fmax(fmax), duration(duration) 
	{
		a = (fmax-fmin)/duration;
		b = fmin;
	};
	float32_t generate(float32_t time)
	{
		float32_t frequency;
		frequency = a * time + b;
		if (frequency > fmax) frequency = fmax;
		return ot_sin( 2.0F * PI * frequency * time);
	};
private:
	float32_t a;
	float32_t b;
};
