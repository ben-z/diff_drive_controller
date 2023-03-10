#include <deque>

class MovingAverage {
public:
    MovingAverage(int window_size = 1) : window_size(window_size), total(0.0) {}
    
    void add(double value) {
        if (buffer.size() == window_size) {
            total -= buffer.front();
            buffer.pop_front();
        }
        buffer.push_back(value);
        total += value;
    }
    
    double getAverage() const {
        if (buffer.empty()) {
            return 0.0;
        }
        return total / buffer.size();
    }
    
    void setWindowSize(int new_size) {
        window_size = new_size;
        while (buffer.size() > window_size) {
            total -= buffer.front();
            buffer.pop_front();
        }
    }
    
    void reset() {
        total = 0.0;
        buffer.clear();
    }
    
private:
    int window_size;
    double total;
    std::deque<double> buffer;
};
