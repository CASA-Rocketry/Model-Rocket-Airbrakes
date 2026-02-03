//Utility class to create time-based trigger
#include <functional>

class Trigger{
private: 
    static inline bool previousState{false};
    static inline unsigned long timeOfPressMillis{0};
public:
    static bool getHoldState(bool, unsigned long);
    static void reset();
};