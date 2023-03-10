#include <cmath>

// A 2nd order Butterworth filter
// Inspired by:
// https://youtu.be/HJ-C4Incgpw
// https://github.com/curiores/ArduinoTutorials/blob/81135c435d2dab093e971c9628cb457f1b95de3d/BasicFilters/Design/LowPass/ButterworthFilter.ipynb
// Derived from ChatGPT

class ButterworthFilter2ndOrder {
public:

    double cutoff_frequency;
    double sampling_frequency;

    ButterworthFilter2ndOrder(double cutoff_frequency = 10.0, double sampling_frequency = 1000.0) {
        reset(cutoff_frequency, sampling_frequency);
    }

    double filter(double x) {
        double y = b0 * x + b1 * x1 + b2 * x2 + a1 * y1 + a2 * y2;

        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;

        return y;
    }

    void reset(double cutoff_frequency, double sampling_frequency) {
        // double w0 = 2.0 * M_PI * cutoff_frequency / sampling_frequency;
        // double cw0 = std::cos(w0);
        // double sw0 = std::sin(w0);
        // double alpha = sw0 / (2.0 * 0.707);
        // double beta = std::sqrt(1.0 - alpha * alpha);

        // a0 = 1.0 + 2.0 * alpha + beta;
        // a1 = -2.0 * cw0;
        // a2 = 1.0 - 2.0 * alpha + beta;
        // b0 = (1.0 - cw0) / 2.0 / a0;
        // b1 = (1.0 - cw0) / a0;
        // b2 = (1.0 - cw0) / 2.0 / a0;
        // x1 = x2 = y1 = y2 = 0.0;

        this->cutoff_frequency = cutoff_frequency;
        this->sampling_frequency = sampling_frequency;

        const double ita =1.0/ tan(M_PI*cutoff_frequency/sampling_frequency);
        const double q=sqrt(2.0);
        b0 = 1.0 / (1.0 + q*ita + ita*ita);
        b1= 2*b0;
        b2= b0;
        a1 = 2.0 * (ita*ita - 1.0) * b0;
        a2 = -(1.0 - q*ita + ita*ita) * b0;
        x1 = x2 = y1 = y2 = 0.0;
    }

private:
    double a0, a1, a2, b0, b1, b2;
    double x1, x2, y1, y2;
};
