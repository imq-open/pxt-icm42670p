

#include "pxt.h"

extern "C" int main1();

//%
namespace icm42670p
{

    //%
    int poll()
    {
        return main1();
    }

} // namespace icm42670p
