#include <sstream>
#include <string>

namespace Yolo3
{
    enum RoadsignClasses//using coco for default cfg and weights
    {
        SPEED_OTHER, CHILD_PROTECT, NO_LEFT, NO_RIGHT, NO_PARKING, CROSSWALK, NO_UTURN,
        ROTATE, SLOW, SPEED_BUST, UNPROTECTED_LEFT, UTURN, NO_STRAIGHT, SPEED_20, SPEED_30,
        SPEED_40, SPEED_50, SPEED_60, SPEED_70, SPEED_80, SPEED_90, SPEED_100,     
    };
}

template<typename _Tp> class RoadsignClassScore
{
public:
	_Tp x, y, w, h;
	_Tp score;
	unsigned int class_type;
	bool enabled;

	inline std::string toString()
	{
		std::ostringstream out;
		out << class_type << "(x:" << x << ", y:" << y << ", w:" << w << ", h:" << h << ") =" << score;
		return out.str();
	}
	inline std::string GetClassString()
	{
		switch (class_type)
		{
            case Yolo3::SPEED_OTHER: return "SPEED_OTHER";
            case Yolo3::CHILD_PROTECT: return "CHILD_PROTECT";
            case Yolo3::NO_LEFT: return "NO_LEFT";
            case Yolo3::NO_RIGHT: return "NO_RIGHT";
            case Yolo3::NO_PARKING: return "NO_PARKING";
            case Yolo3::CROSSWALK: return "CROSSWALK";
            case Yolo3::NO_UTURN: return "NO_UTURN";
            case Yolo3::ROTATE: return "ROTATE";
            case Yolo3::SLOW: return "SLOW";
            case Yolo3::SPEED_BUST: return "SPEED_BUST";
            case Yolo3::UNPROTECTED_LEFT: return "UNPROTECTED_LEFT";
            case Yolo3::UTURN: return "UTURN";
            case Yolo3::NO_STRAIGHT: return "NO_STRAIGHT";
            case Yolo3::SPEED_20: return "SPEED_20";
            case Yolo3::SPEED_30: return "SPEED_30";
            case Yolo3::SPEED_40: return "SPEED_40";
            case Yolo3::SPEED_50: return "SPEED_50";
            case Yolo3::SPEED_60: return "SPEED_60";
            case Yolo3::SPEED_70: return "SPEED_70";
            case Yolo3::SPEED_80: return "SPEED_80";
            case Yolo3::SPEED_90: return "SPEED_90";
            case Yolo3::SPEED_100: return "SPEED_100";
			default:return "error";
		}
	}
	inline int GetClassInt()
    {
        switch (class_type)
        {
            case Yolo3::SPEED_OTHER: return 0;
            case Yolo3::CHILD_PROTECT: return 1;
            case Yolo3::NO_LEFT: return 2;
            case Yolo3::NO_RIGHT: return 3;
            case Yolo3::NO_PARKING: return 4;
            case Yolo3::CROSSWALK: return 5;
            case Yolo3::NO_UTURN: return 6;
            case Yolo3::ROTATE: return 7;
            case Yolo3::SLOW: return 8;
            case Yolo3::SPEED_BUST: return 9;
            case Yolo3::UNPROTECTED_LEFT: return 10;
            case Yolo3::UTURN: return 11;
            case Yolo3::NO_STRAIGHT: return 12;
            case Yolo3::SPEED_20: return 13;
            case Yolo3::SPEED_30: return 14;
            case Yolo3::SPEED_40: return 15;
            case Yolo3::SPEED_50: return 16;
            case Yolo3::SPEED_60: return 17;
            case Yolo3::SPEED_70: return 18;
            case Yolo3::SPEED_80: return 19;
            case Yolo3::SPEED_90: return 20;
            case Yolo3::SPEED_100: return 21;
            default:return 0;
        }
	}
};