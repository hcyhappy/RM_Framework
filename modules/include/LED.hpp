#ifndef RM_LED_HPP
#define RM_LED_HPP
namespace LED {
enum class Color { Blue, Green, Red };
void Set(Color color, bool on);
void Toggle(Color color);
}
#endif
